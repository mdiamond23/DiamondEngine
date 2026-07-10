#include "GLEditorBackend.h"

// The GL editor render path, transplanted whole from the old main.cpp loop
// (editor-wiring step 3). The shared loop drives scene/systems/ImGui; this
// class owns everything that talks to OpenGL: window + context, the deferred
// render graph into viewport FBOs, shadow/IBL setup, shader hot-reload, debug
// draw, the in-game UI passes, and the particle preview FBO.

#include <glad/gl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include "EditorLayers.h"
#include "MeshThumbnailRenderer.h"
#include "Editor/ThumbnailService.h"
#include "Editor/MaterialAsset.h"
#include "Assets/ImageLoader.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"
#include "Scene/Physics/PhysicsAPI.h"
#include "Scene/ParticleSystem.h"
#include "Scene/UISystem.h"
#include "Scene/UIRenderSystem.h"
#include "Scene/UIInputSystem.h"
#include "Scene/UINavigationSystem.h"
#include "DebugDraw.h"
#include "Core/Camera.h"
#include "Core/Input.h"
#include "Renderer/MeshData.h"
#include "Renderer/Material.h"
#include "Renderer/TextureData.h"
#include "Renderer/RenderGraph.h"
#include "Renderer/Renderer2D.h"
#include "Renderer/ParticleRenderer.h"
#include "Renderer/Frustum.h"
#include "Profiling/GLRendererStats.h"
#include "Platform/OpenGL/Resources/OpenGLRenderTypes.h"
#include "Platform/OpenGL/Resources/OpenGLShader.h"
#include "Platform/OpenGL/Resources/ShaderLibrary.h"
#include "Platform/OpenGL/Resources/OpenGLTexture.h"
#include "Platform/OpenGL/Passes/IBL/OpenGLIBLPass.h"
#include "Platform/OpenGL/Passes/Shadows/OpenGLShadowPass.h"
#include "Platform/OpenGL/Passes/Shadows/OpenGLCSMPass.h"
#include "Platform/OpenGL/Passes/Deferred/OpenGLGBufferPass.h"
#include "Platform/OpenGL/Passes/Deferred/OpenGLSSAOPass.h"
#include "Platform/OpenGL/Passes/Deferred/OpenGLDeferredLightingPass.h"
#include "Platform/OpenGL/Passes/PostProcess/OpenGLTonemapPass.h"
#include "Platform/OpenGL/Passes/PostProcess/OpenGLBloomPass.h"
#include "Platform/OpenGL/Passes/PostProcess/OpenGLFXAAPass.h"
#include "Platform/OpenGL/Passes/Forward/OpenGLTransparencyPass.h"
#include "Animation/AnimationComponents.h"
#include "Animation/AnimationSampler.h"
#include "Animation/IKComponent.h"
#include "Scene/DebugVisualization.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace Diamond;

namespace {

// GL ThumbnailService (editor-wiring step 4): the mesh/material studio is the
// existing MeshThumbnailRenderer FBO bake; texture previews are plain GL
// uploads (moved here from ContentPanel so the panel is backend-agnostic).
// The service owns every returned texture id and deletes the survivors when
// the backend shuts down (while the GL context is still current).
class GLThumbnailService final : public Diamond::ThumbnailService {
public:
    ~GLThumbnailService() override {
        for (uint32_t id : m_Live) glDeleteTextures(1, &id);
    }

    uint64_t CreateTextureThumbnail(const std::string& path) override {
        Diamond::ImageData img = Diamond::ImageLoader::Load(path, false);
        if (img.Pixels.empty()) return 0;

        GLenum fmt;
        switch (img.Channels) {
            case 1:  fmt = GL_RED;  break;
            case 3:  fmt = GL_RGB;  break;
            case 4:  fmt = GL_RGBA; break;
            default: return 0;
        }

        uint32_t texID = 0;
        glGenTextures(1, &texID);
        glBindTexture(GL_TEXTURE_2D, texID);
        glTexImage2D(GL_TEXTURE_2D, 0, fmt, img.Width, img.Height, 0,
                     fmt, GL_UNSIGNED_BYTE, img.Pixels.data());
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
        return Track(texID);
    }

    uint64_t CreateMeshThumbnail(const std::vector<Diamond::MeshData>& meshes) override {
        return Track(m_Renderer.Render(meshes));
    }

    uint64_t CreateMaterialThumbnail(const Diamond::PBRMaterial& mat) override {
        return Track(m_Renderer.RenderMaterial(mat));
    }

    void Release(uint64_t id) override {
        auto texID = (uint32_t)id;
        if (!m_Live.erase(texID)) return;
        glDeleteTextures(1, &texID);
    }

private:
    uint64_t Track(uint32_t texID) {
        if (texID) m_Live.insert(texID);
        return texID;
    }

    MeshThumbnailRenderer        m_Renderer;
    std::unordered_set<uint32_t> m_Live;
};

// (Re)creates a color-only RGBA8 FBO + texture at w x h, deleting any prior pair.
// Used for offscreen editor panels (e.g. the particle preview) that render at
// their own size, outside the render graph.
void makeColorFBO(uint32_t& fbo, uint32_t& tex, int w, int h) {
    if (tex) glDeleteTextures(1,     &tex);
    if (fbo) glDeleteFramebuffers(1, &fbo);
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace

// Everything that requires a live GL context, created in Init() after glad
// loads and destroyed (in Shutdown) while the context is still current.
struct GLEditorBackend::Gfx {
    // --- Shaders ---
    // Registered by name in a library that owns them and supports hot-reload.
    // Each pointer below is a stable reference into the library, so the frame
    // code is unchanged by a reload (which swaps the program in place).
    ShaderLibrary shaders;
    OpenGLShader* gbufferShader           = nullptr;
    OpenGLShader* gbufferSkinnedShader    = nullptr;
    OpenGLShader* ssaoShader              = nullptr;
    OpenGLShader* ssaoBlurShader          = nullptr;
    OpenGLShader* lightingShader          = nullptr;
    OpenGLShader* backgroundShader        = nullptr;
    OpenGLShader* blurShader              = nullptr;
    OpenGLShader* bloomFinalShader        = nullptr;
    OpenGLShader* fxaaShader              = nullptr;
    OpenGLShader* depthShader             = nullptr;
    OpenGLShader* depthSkinnedShader      = nullptr;
    OpenGLShader* pointDepthShader        = nullptr;
    OpenGLShader* pointDepthSkinnedShader = nullptr;
    OpenGLShader* transparencyShader      = nullptr;

    // --- Pass objects ---
    OpenGLIBLPass              iblPass;
    OpenGLShadowPass           shadowPass;
    OpenGLCSMPass              csmPass;
    OpenGLGBufferPass          gbufferPass;
    OpenGLSSAOPass             ssaoPass;
    OpenGLDeferredLightingPass lightingPass;
    OpenGLBloomPass            bloomPass;
    OpenGLFXAAPass             fxaaPass;
    OpenGLTransparencyPass     transparencyPass;

    // 2D UI layer + particle billboards (see the old main.cpp for the story).
    std::unique_ptr<Renderer2D>       uiRenderer;
    std::unique_ptr<ParticleRenderer> particleRenderer;
    std::shared_ptr<Texture>          particleTex;   // procedural soft-dot default
    std::vector<RenderParticle>       particleScratch;

    // 1x1 blue-glass RGBA texture for the transparency pass' window quads.
    uint32_t windowTex = 0;

    // --- Render graph + transients ---
    RenderGraph       graph;
    GLRGTextureHandle gViewPos{}, gViewNormal{}, gAlbedo{}, gMaterial{}, gEmissive{};
    GLRGTextureHandle hSsaoRaw{}, hSsaoBlur{};
    GLRGTextureHandle hdrBuffer{}, brightBuffer{};
    GLRGTextureHandle pingPong[2] {};
    GLRGTextureHandle ldrBuffer{};

    // Viewport FBOs — managed outside the render graph so they aren't freed
    // between frames. outputFBO selects the FXAA sink target per Execute().
    uint32_t viewportFBO     = 0;
    uint32_t viewportTexture = 0;
    uint32_t gameViewportFBO = 0;
    uint32_t gameViewportTex = 0;
    uint32_t outputFBO       = 0;

    // Particle preview FBO — sized to the preview panel's content region.
    uint32_t previewFBO = 0, previewTex = 0;
    int      previewW   = 0, previewH   = 0;

    // Asset previews for the content browser / inspector (editor-wiring step 4).
    std::unique_ptr<GLThumbnailService> thumbnails;

    // --- Per-frame state captured by the render graph lambdas ---
    Scene* scene = nullptr;
    std::vector<DrawCall> allDraws;
    std::vector<DrawCall> shadowDraws;
    std::vector<DrawCall> transparentDraws;
    std::unordered_map<PBRMaterial*, std::vector<DrawCall>> materialBatches;
    std::unordered_map<PBRMaterial*, std::vector<DrawCall>> culledBatches;
    std::vector<glm::vec3> pointPositions, pointColors;
    std::vector<glm::vec3> spotPositions, spotDirs, spotColors;
    std::vector<float>     spotCosInner, spotCosOuter;
    SunLight  sun;
    glm::mat4 view { 1.0f }, proj { 1.0f };
    glm::vec3 camPos { 0.0f };
    int fbW = 0, fbH = 0;

    // GPU frame timing (Docs/profiler-panel-design.md, Phase 2): double-buffered
    // GL_TIME_ELAPSED query objects — query slot N this frame, read back slot
    // N's result from last frame (avoids stalling on the GPU).
    uint32_t timeQueries[2]     = { 0, 0 };
    bool     timeQueryIssued[2] = { false, false };
    int      timeQuerySlot      = 0;
};

GLEditorBackend::GLEditorBackend()  = default;
GLEditorBackend::~GLEditorBackend() { Shutdown(); }

bool GLEditorBackend::Init(uint32_t width, uint32_t height, const char* title)
{
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    m_Window = glfwCreateWindow((int)width, (int)height, title, nullptr, nullptr);
    if (!m_Window) return false;

    glfwMakeContextCurrent(m_Window);
    glfwSwapInterval(1);
    glfwSetInputMode(m_Window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    glfwSetWindowUserPointer(m_Window, this);
    glfwSetFramebufferSizeCallback(m_Window, [](GLFWwindow*, int w, int h) {
        glViewport(0, 0, w, h);
    });
    glfwSetDropCallback(m_Window, [](GLFWwindow* w, int count, const char** paths) {
        auto* self = static_cast<GLEditorBackend*>(glfwGetWindowUserPointer(w));
        if (self && self->m_DropHandler) self->m_DropHandler(count, paths);
    });

    if (!gladLoadGL(glfwGetProcAddress)) {
        glfwDestroyWindow(m_Window);
        m_Window = nullptr;
        return false;
    }

    ImGui_ImplGlfw_InitForOpenGL(m_Window, true);
    ImGui_ImplOpenGL3_Init("#version 410");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

    m_G = std::make_unique<Gfx>();
    Gfx& g = *m_G;

    glGenQueries(2, g.timeQueries);

    auto& s = g.shaders;
    g.gbufferShader           = &s.Load("gbuffer",           ENGINE_SHADERS_DIR "/Deferred/gbuffer.vert",            ENGINE_SHADERS_DIR "/Deferred/gbuffer.frag");
    g.gbufferSkinnedShader    = &s.Load("gbufferSkinned",    ENGINE_SHADERS_DIR "/Deferred/gbuffer_skinned.vert",    ENGINE_SHADERS_DIR "/Deferred/gbuffer.frag");
    g.ssaoShader              = &s.Load("ssao",              ENGINE_SHADERS_DIR "/Deferred/quad.vert",               ENGINE_SHADERS_DIR "/Deferred/ssao.frag");
    g.ssaoBlurShader          = &s.Load("ssaoBlur",          ENGINE_SHADERS_DIR "/Deferred/quad.vert",               ENGINE_SHADERS_DIR "/Deferred/ssao_blur.frag");
    g.lightingShader          = &s.Load("lighting",          ENGINE_SHADERS_DIR "/Deferred/quad.vert",               ENGINE_SHADERS_DIR "/Deferred/lighting.frag");
    OpenGLShader& equirect    =  s.Load("equirect",          ENGINE_SHADERS_DIR "/IBL/cubemap.vert",                 ENGINE_SHADERS_DIR "/IBL/equirect_to_cubemap.frag");
    OpenGLShader& irradiance  =  s.Load("irradiance",        ENGINE_SHADERS_DIR "/IBL/cubemap.vert",                 ENGINE_SHADERS_DIR "/IBL/irradiance.frag");
    OpenGLShader& prefilter   =  s.Load("prefilter",         ENGINE_SHADERS_DIR "/IBL/cubemap.vert",                 ENGINE_SHADERS_DIR "/IBL/prefilter.frag");
    OpenGLShader& brdf        =  s.Load("brdf",              ENGINE_SHADERS_DIR "/IBL/brdf.vert",                    ENGINE_SHADERS_DIR "/IBL/brdf.frag");
    g.backgroundShader        = &s.Load("background",        ENGINE_SHADERS_DIR "/IBL/background.vert",              ENGINE_SHADERS_DIR "/IBL/background.frag");
    g.blurShader              = &s.Load("blur",              ENGINE_SHADERS_DIR "/Bloom/blur.vert",                  ENGINE_SHADERS_DIR "/Bloom/blur.frag");
    g.bloomFinalShader        = &s.Load("bloomFinal",        ENGINE_SHADERS_DIR "/Bloom/bloomfinal.vert",            ENGINE_SHADERS_DIR "/Bloom/bloomfinal.frag");
    g.fxaaShader              = &s.Load("fxaa",              ENGINE_SHADERS_DIR "/PostProcess/tonemap.vert",         ENGINE_SHADERS_DIR "/PostProcess/fxaa.frag");
    g.depthShader             = &s.Load("depth",             ENGINE_SHADERS_DIR "/Shadows/depth.vert",               ENGINE_SHADERS_DIR "/Shadows/depth.frag");
    g.depthSkinnedShader      = &s.Load("depthSkinned",      ENGINE_SHADERS_DIR "/Shadows/depth_skinned.vert",       ENGINE_SHADERS_DIR "/Shadows/depth.frag");
    g.pointDepthShader        = &s.Load("pointDepth",        ENGINE_SHADERS_DIR "/Shadows/point_depth.vert",         ENGINE_SHADERS_DIR "/Shadows/point_depth.geom", ENGINE_SHADERS_DIR "/Shadows/point_depth.frag");
    g.pointDepthSkinnedShader = &s.Load("pointDepthSkinned", ENGINE_SHADERS_DIR "/Shadows/point_depth_skinned.vert", ENGINE_SHADERS_DIR "/Shadows/point_depth.geom", ENGINE_SHADERS_DIR "/Shadows/point_depth.frag");
    g.transparencyShader      = &s.Load("transparency",      ENGINE_SHADERS_DIR "/Forward/transparent.vert",         ENGINE_SHADERS_DIR "/Forward/transparent.frag");

    g.uiRenderer       = Renderer2D::Create();
    g.particleRenderer = ParticleRenderer::Create();
    g.thumbnails       = std::make_unique<GLThumbnailService>();

    // Procedural soft-dot sprite: white core with a radial alpha falloff, so the
    // billboards read as glowing points without depending on an asset file. Used
    // as the default when an emitter has no texture assigned.
    std::vector<uint8_t> dotPixels(64 * 64 * 4);
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 64; ++x) {
            float dx = (x + 0.5f) / 64.0f * 2.0f - 1.0f;
            float dy = (y + 0.5f) / 64.0f * 2.0f - 1.0f;
            float a  = glm::clamp(1.0f - std::sqrt(dx * dx + dy * dy), 0.0f, 1.0f);
            a *= a;  // soften the edge
            int i = (y * 64 + x) * 4;
            dotPixels[i + 0] = 255; dotPixels[i + 1] = 255;
            dotPixels[i + 2] = 255; dotPixels[i + 3] = (uint8_t)(a * 255.0f);
        }
    g.particleTex = Texture::CreateFromPixels(dotPixels.data(), 64, 64, 4);

    // IBL bake — one-time
    {
        OpenGLTexture envTex(ASSETS_DIR "/Textures/citrus_orchard_road_puresky_4k.hdr", true, true);
        g.iblPass.BakeEnvironment(envTex, equirect, irradiance, prefilter, brdf);
    }

    // Shadow maps — persistent, not managed by the graph
    g.csmPass.Setup(2048);
    g.shadowPass.SetupPointShadowMaps(4, 512, 25.0f);

    // 1x1 blue-glass RGBA texture — replace with a real window PNG when available
    glGenTextures(1, &g.windowTex);
    glBindTexture(GL_TEXTURE_2D, g.windowTex);
    uint8_t windowPixel[4] = { 100, 180, 230, 160 };
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, windowPixel);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    return true;
}

void GLEditorBackend::Shutdown()
{
    if (!m_Window) return;

    if (m_G) {
        Gfx& g = *m_G;
        if (g.viewportTexture) glDeleteTextures(1,     &g.viewportTexture);
        if (g.viewportFBO)     glDeleteFramebuffers(1, &g.viewportFBO);
        if (g.gameViewportTex) glDeleteTextures(1,     &g.gameViewportTex);
        if (g.gameViewportFBO) glDeleteFramebuffers(1, &g.gameViewportFBO);
        if (g.previewTex)      glDeleteTextures(1,     &g.previewTex);
        if (g.previewFBO)      glDeleteFramebuffers(1, &g.previewFBO);
        if (g.windowTex)       glDeleteTextures(1,     &g.windowTex);
        glDeleteQueries(2, g.timeQueries);
        m_G.reset();   // pass/shader/texture dtors need the context current
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(m_Window);
    m_Window = nullptr;
}

void GLEditorBackend::SetDropHandler(std::function<void(int, const char**)> handler)
{
    m_DropHandler = std::move(handler);
}

Diamond::ThumbnailService* GLEditorBackend::Thumbnails()
{
    return m_G ? m_G->thumbnails.get() : nullptr;
}

Diamond::RendererStats GLEditorBackend::GetStats() const
{
    return GLStats::GetStats();
}

void GLEditorBackend::BeginImGuiFrame()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void GLEditorBackend::OnImGuiPanels()
{
    if (ImGui::Begin("Renderer")) {
        ImGui::Text("Backend: OpenGL (deferred)");
        ImGui::Text("Scene output: %dx%d", m_LastFbW, m_LastFbH);
        ImGui::Separator();
        ImGui::Checkbox("FXAA", &m_FxaaEnabled);
    }
    ImGui::End();
}

Diamond::EditorViewImage GLEditorBackend::SceneViewImage() const
{
    // GL FBO textures are bottom-up — the panel flips the UVs.
    return { m_G ? (uint64_t)m_G->viewportTexture : 0, /*flipY*/ true };
}

Diamond::EditorViewImage GLEditorBackend::GameViewImage() const
{
    if (!m_G || !m_LastGameViewValid) return {};
    return { (uint64_t)m_G->gameViewportTex, /*flipY*/ true };
}

void GLEditorBackend::BuildGraph(int w, int h)
{
    Gfx& g = *m_G;
    g.fbW = w;
    g.fbH = h;
    g.graph.Clear();

    // G-buffer — 5 attachments sharing one FBO (gViewPos owns it)
    g.gViewPos    = g.graph.DeclareTexture("gViewPos",    { w, h, GL_RGBA16F, true  });
    g.gViewNormal = g.graph.DeclareTexture("gViewNormal", { w, h, GL_RGBA16F, false, g.gViewPos, 1 });
    g.gAlbedo     = g.graph.DeclareTexture("gAlbedo",     { w, h, GL_RGBA8,   false, g.gViewPos, 2 });
    g.gMaterial   = g.graph.DeclareTexture("gMaterial",   { w, h, GL_RGBA8,   false, g.gViewPos, 3 });
    g.gEmissive   = g.graph.DeclareTexture("gEmissive",   { w, h, GL_RGB16F,  false, g.gViewPos, 4 });

    // SSAO intermediates
    g.hSsaoRaw  = g.graph.DeclareTexture("ssaoRaw",  { w, h, GL_R16F, false });
    g.hSsaoBlur = g.graph.DeclareTexture("ssaoBlur", { w, h, GL_R16F, false });

    // HDR + bloom extract (MRT)
    g.hdrBuffer    = g.graph.DeclareTexture("hdrBuffer",    { w, h, GL_RGBA16F, true  });
    g.brightBuffer = g.graph.DeclareTexture("brightBuffer", { w, h, GL_RGBA16F, false, g.hdrBuffer, 1 });

    // Bloom ping-pong
    g.pingPong[0] = g.graph.DeclareTexture("bloomPing", { w, h, GL_RGBA16F, false });
    g.pingPong[1] = g.graph.DeclareTexture("bloomPong", { w, h, GL_RGBA16F, false });

    // LDR intermediate — bloom composite writes here, FXAA reads and outputs to viewport
    g.ldrBuffer = g.graph.DeclareTexture("ldrBuffer", { w, h, GL_RGBA8, true });

    // Viewport FBOs — recreate on resize
    makeColorFBO(g.viewportFBO,     g.viewportTexture, w, h);
    makeColorFBO(g.gameViewportFBO, g.gameViewportTex, w, h);

    // Shadow passes — sinks (no graph writes), always alive
    g.graph.AddPass("DirectionalShadow")
        .SetExecute([this] {
            Gfx& g = *m_G;
            g.csmPass.Render(*g.depthShader, *g.depthSkinnedShader, g.shadowDraws,
                             g.sun, g.view, g.proj, 0.1f, 100.0f);
        });

    g.graph.AddPass("PointShadow")
        .SetExecute([this] {
            Gfx& g = *m_G;
            g.shadowPass.RenderPointShadowPass(*g.pointDepthShader, *g.pointDepthSkinnedShader,
                g.shadowDraws, g.pointPositions.data(), (int)g.pointPositions.size());
        });

    // Geometry pass — fills all 5 G-buffer attachments
    g.graph.AddPass("GBuffer")
        .Write(g.gViewPos).Write(g.gViewNormal).Write(g.gAlbedo).Write(g.gMaterial).Write(g.gEmissive)
        .SetExecute([this] {
            Gfx& g = *m_G;
            // Clear once, then render all material batches into the same FBO
            glBindFramebuffer(GL_FRAMEBUFFER, g.graph.GetFBO(g.gViewPos));
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            for (auto& [mat, batch] : g.culledBatches)
                if (!batch.empty())
                    g.gbufferPass.Render(*g.gbufferShader, *g.gbufferSkinnedShader, batch, *mat,
                                         g.view, g.proj, g.graph.GetFBO(g.gViewPos), g.fbW, g.fbH);
        });

    // SSAO — raw occlusion
    g.graph.AddPass("SSAO")
        .Read(g.gViewPos).Read(g.gViewNormal)
        .Write(g.hSsaoRaw)
        .SetExecute([this] {
            Gfx& g = *m_G;
            g.ssaoPass.RenderSSAO(*g.ssaoShader,
                                  g.graph.GetTexture(g.gViewPos),
                                  g.graph.GetTexture(g.gViewNormal),
                                  g.proj,
                                  g.graph.GetFBO(g.hSsaoRaw),
                                  g.fbW, g.fbH);
        });

    // SSAO blur
    g.graph.AddPass("SSAOBlur")
        .Read(g.hSsaoRaw)
        .Write(g.hSsaoBlur)
        .SetExecute([this] {
            Gfx& g = *m_G;
            g.ssaoPass.RenderBlur(*g.ssaoBlurShader,
                                  g.graph.GetTexture(g.hSsaoRaw),
                                  g.graph.GetFBO(g.hSsaoBlur),
                                  g.fbW, g.fbH);
        });

    // Deferred lighting + skybox
    g.graph.AddPass("DeferredLighting")
        .Read(g.gViewPos).Read(g.gViewNormal).Read(g.gAlbedo).Read(g.gMaterial)
        .Read(g.hSsaoBlur).Read(g.gEmissive)
        .Write(g.hdrBuffer).Write(g.brightBuffer)
        .SetExecute([this] {
            Gfx& g = *m_G;
            g.lightingPass.Render(*g.lightingShader,
                                  g.graph.GetTexture(g.gViewPos),
                                  g.graph.GetTexture(g.gViewNormal),
                                  g.graph.GetTexture(g.gAlbedo),
                                  g.graph.GetTexture(g.gMaterial),
                                  g.graph.GetTexture(g.hSsaoBlur),
                                  g.graph.GetTexture(g.gEmissive),
                                  g.view, g.camPos,
                                  g.pointPositions, g.pointColors,
                                  g.spotPositions, g.spotDirs, g.spotColors,
                                  g.spotCosInner, g.spotCosOuter, g.sun,
                                  g.shadowPass, g.csmPass, g.iblPass,
                                  g.graph.GetFBO(g.hdrBuffer),
                                  g.fbW, g.fbH);

            // Blit G-buffer depth → HDR FBO so the skybox can depth-test
            glBindFramebuffer(GL_READ_FRAMEBUFFER, g.graph.GetFBO(g.gViewPos));
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, g.graph.GetFBO(g.hdrBuffer));
            glBlitFramebuffer(0, 0, g.fbW, g.fbH, 0, 0, g.fbW, g.fbH,
                              GL_DEPTH_BUFFER_BIT, GL_NEAREST);

            // Skybox renders into the HDR FBO using the blitted depth
            glBindFramebuffer(GL_FRAMEBUFFER, g.graph.GetFBO(g.hdrBuffer));
            g.lightingPass.DrawSkybox(*g.backgroundShader, g.iblPass, g.view, g.proj, g.fbW, g.fbH);
        });

    // Forward transparency — sorts glass draws back-to-front, blits depth internally.
    // Draws come from MeshComponents flagged transparent (rebuilt each frame).
    g.graph.AddPass("Transparency")
        .Read(g.gViewPos)
        .Read(g.hdrBuffer)
        .SetExecute([this] {
            Gfx& g = *m_G;
            g.transparencyPass.Render(*g.transparencyShader, g.transparentDraws, g.windowTex,
                                      g.graph.GetFBO(g.gViewPos),
                                      g.graph.GetFBO(g.hdrBuffer),
                                      g.view, g.proj, g.camPos,
                                      g.fbW, g.fbH);
        });

    // Particle billboards — drawn into the HDR buffer after Transparency so
    // they composite over opaques and bloom picks up bright sparks. Reads
    // gViewPos + hdrBuffer (no writes → sink, always alive); by insertion
    // order it sorts after Transparency and before BloomComposite. Depth is
    // already present in hdrBuffer (Transparency blits it from gViewPos), so
    // particles depth-test against the scene.
    g.graph.AddPass("Particles")
        .Read(g.gViewPos)
        .Read(g.hdrBuffer)
        .SetExecute([this] {
            Gfx& g = *m_G;
            if (!g.particleRenderer || !g.scene) return;
            glBindFramebuffer(GL_FRAMEBUFFER, g.graph.GetFBO(g.hdrBuffer));
            glViewport(0, 0, g.fbW, g.fbH);
            g.particleRenderer->Begin(g.view, g.proj);
            for (auto [e, em] : g.scene->View<ParticleEmitterComponent>().each()) {
                if (em.liveCount == 0) continue;
                g.particleScratch.clear();
                g.particleScratch.reserve(em.liveCount);
                for (std::size_t i = 0; i < em.liveCount; ++i) {
                    const auto& p = em.pool[i];
                    g.particleScratch.push_back({ p.pos, p.size, p.color, p.rotation });
                }
                const Texture& tex = em.texture ? *em.texture : *g.particleTex;
                g.particleRenderer->Draw(g.particleScratch, tex, em.blend);
            }
            g.particleRenderer->End();
        });

    // Bloom blur ping-pong
    g.graph.AddPass("BloomBlur")
        .Read(g.brightBuffer)
        .Write(g.pingPong[0]).Write(g.pingPong[1])
        .SetExecute([this] {
            Gfx& g = *m_G;
            bool horizontal = true;
            uint32_t src = g.graph.GetTexture(g.brightBuffer);
            for (int i = 0; i < 6; ++i) {
                int dst = i % 2;
                g.bloomPass.RenderBlurPass(*g.blurShader, src, g.graph.GetFBO(g.pingPong[dst]), horizontal);
                src        = g.graph.GetTexture(g.pingPong[dst]);
                horizontal = !horizontal;
            }
        });

    // Composite + tonemap → LDR intermediate
    g.graph.AddPass("BloomComposite")
        .Read(g.hdrBuffer).Read(g.pingPong[1])
        .Write(g.ldrBuffer)
        .SetExecute([this] {
            Gfx& g = *m_G;
            g.bloomPass.RenderCompositePass(*g.bloomFinalShader,
                g.graph.GetTexture(g.hdrBuffer),
                g.graph.GetTexture(g.pingPong[1]),
                true, 0.1f,
                g.graph.GetFBO(g.ldrBuffer));
        });

    // FXAA — sink pass, renders into whichever FBO outputFBO points to
    g.graph.AddPass("FXAA")
        .Read(g.ldrBuffer)
        .SetExecute([this] {
            Gfx& g = *m_G;
            g.fxaaPass.Render(*g.fxaaShader,
                g.graph.GetTexture(g.ldrBuffer),
                g.fbW, g.fbH, m_FxaaEnabled,
                g.outputFBO);
        });

    g.graph.Compile();
}

void GLEditorBackend::CullAndExecute(uint32_t targetFBO)
{
    Gfx& g = *m_G;
    Frustum frustum = Frustum::Extract(g.proj * g.view);
    g.culledBatches.clear();
    for (auto& [mat, batch] : g.materialBatches) {
        auto& out = g.culledBatches[mat];
        out.clear();
        for (const auto& dc : batch)
            if (frustum.TestAABB(dc.localBounds.Transform(dc.modelMatrix)))
                out.push_back(dc);

        GLStats::RecordVisible((uint32_t)out.size());
        GLStats::RecordCulled((uint32_t)(batch.size() - out.size()));
        if (!out.empty()) {
            GLStats::RecordMaterialBound(mat);
            auto tex = [](const std::shared_ptr<Texture>& t) {
                if (t) GLStats::RecordTextureUsed(t.get());
            };
            tex(mat->Albedo); tex(mat->Normal); tex(mat->Metallic);
            tex(mat->Roughness); tex(mat->AO); tex(mat->Emissive);
        }
    }
    g.outputFBO = targetFBO;
    g.graph.Execute();
}

void GLEditorBackend::RenderFrame(const Diamond::EditorFrameInput& in)
{
    Gfx& g = *m_G;
    Scene& scene = *in.scene;
    auto&  reg   = scene.GetRegistry();
    g.scene = in.scene;

    // Hot-reload: F5 force-reloads everything (shaders, textures, materials);
    // otherwise poll source file mtimes a couple times a second and reload
    // whatever changed on disk. Textures/materials are GL-only for now -- an
    // in-place swap isn't safe under Vulkan's in-flight frames/descriptor
    // caches (Docs/asset-pipeline-design.md, §2).
    if (Input::IsKeyPressed(Key::F5)) {
        g.shaders.ReloadAll();
        Assets::ReloadAll();
    } else {
        m_ShaderReloadTimer += in.dt;
        if (m_ShaderReloadTimer >= 0.5f) {
            g.shaders.ReloadChanged();
            Assets::ReloadChanged();
            m_ShaderReloadTimer = 0.0f;
        }
    }

    glfwGetFramebufferSize(m_Window, &g.fbW, &g.fbH);
    if (g.fbW <= 0 || g.fbH <= 0) return;   // minimized

    GLStats::Reset();

    // GPU frame timing (Phase 2): read back the slot written last frame (if its
    // result has landed yet — GL_QUERY_RESULT_AVAILABLE avoids a stall), then
    // begin this frame's query on the other slot.
    {
        const int slot = g.timeQuerySlot, prevSlot = 1 - slot;
        if (g.timeQueryIssued[prevSlot]) {
            GLuint available = 0;
            glGetQueryObjectuiv(g.timeQueries[prevSlot], GL_QUERY_RESULT_AVAILABLE, &available);
            if (available) {
                GLuint64 ns = 0;
                glGetQueryObjectui64v(g.timeQueries[prevSlot], GL_QUERY_RESULT, &ns);
                GLStats::SetGpuFrameMs((float)(ns / 1000000.0));
            }
        }
        glBeginQuery(GL_TIME_ELAPSED, g.timeQueries[slot]);
    }

    if (g.fbW != m_LastFbW || g.fbH != m_LastFbH) {
        BuildGraph(g.fbW, g.fbH);
        m_LastFbW = g.fbW;
        m_LastFbH = g.fbH;
    }

    // Build draw calls from scene — uses world matrices so parented entities
    // render correctly. Transparent meshes route to the forward pass instead
    // of the G-buffer.
    g.allDraws.clear();
    g.materialBatches.clear();
    g.transparentDraws.clear();
    for (auto [entity, mc] : reg.view<MeshComponent>().each()) {
        if (!mc.mesh || !mc.visible) continue;
        const glm::mat4 world = scene.GetTransformSystem().GetWorldMatrix(entity);
        if (mc.transparent) {
            g.transparentDraws.push_back({ mc.mesh.get(), world });
            continue;
        }
        if (!mc.material) continue;
        DrawCall dc{ mc.mesh.get(), world, mc.localBounds, mc.castsShadow };
        g.materialBatches[mc.material.get()].push_back(dc);
        g.allDraws.push_back(dc);
    }

    // Skinned meshes — each draw carries the animator's bone palette so both
    // the G-buffer and shadow passes select the skinning shader. Added to
    // allDraws too so the shadowDraws filter below picks up casters.
    for (auto [entity, smc] : reg.view<SkinnedMeshComponent>().each()) {
        if (!smc.visible || !smc.material || smc.meshes.empty()) continue;

        const glm::mat4* palette   = nullptr;
        int              boneCount = 0;
        if (auto* anim = reg.try_get<AnimatorComponent>(entity);
            anim && !anim->palette.empty()) {
            palette   = anim->palette.data();
            boneCount = std::min((int)anim->palette.size(), 100); // matches shader MAX_BONES
        }

        glm::mat4 world = scene.GetTransformSystem().GetWorldMatrix(entity);
        for (auto& mesh : smc.meshes) {
            DrawCall dc{ mesh.get(), world, smc.localBounds, smc.castsShadow };
            dc.bonePalette = palette;
            dc.boneCount   = boneCount;
            g.materialBatches[smc.material.get()].push_back(dc);
            g.allDraws.push_back(dc);
        }
    }

    // Shadow draws are a subset — only casters
    g.shadowDraws.clear();
    for (const auto& dc : g.allDraws)
        if (dc.castsShadow) g.shadowDraws.push_back(dc);

    // Gather lights from ECS
    g.pointPositions.clear();
    g.pointColors.clear();
    g.spotPositions.clear();
    g.spotDirs.clear();
    g.spotColors.clear();
    g.spotCosInner.clear();
    g.spotCosOuter.clear();
    g.sun = SunLight{ glm::vec3(0, -1, 0), glm::vec3(0) };
    bool foundSun = false;
    for (auto [entity, lc] : reg.view<LightComponent>().each()) {
        const glm::mat4& worldMat = scene.GetTransformSystem().GetWorldMatrix(entity);
        glm::vec3 worldPos = glm::vec3(worldMat[3]);
        glm::vec3 worldDir = glm::normalize(glm::mat3(worldMat) * glm::vec3(0.0f, -1.0f, 0.0f));

        if (lc.type == LightType::Point && (int)g.pointPositions.size() < 4) {
            g.pointPositions.push_back(worldPos);
            g.pointColors.push_back(lc.color * lc.intensity);
        } else if (lc.type == LightType::Spot && (int)g.spotPositions.size() < 4) {
            g.spotPositions.push_back(worldPos);
            g.spotDirs.push_back(worldDir);
            g.spotColors.push_back(lc.color * lc.intensity);
            g.spotCosInner.push_back(std::cos(glm::radians(lc.innerConeAngle)));
            g.spotCosOuter.push_back(std::cos(glm::radians(lc.outerConeAngle)));
        } else if (lc.type == LightType::Sun && !foundSun) {
            g.sun.direction = worldDir;
            g.sun.color     = lc.color * lc.intensity;
            foundSun = true;
        }
    }

    g.camPos = in.editorCamera->Position;

    // --- Scene viewport: editor camera ---
    g.view = in.view;
    g.proj = in.proj;
    CullAndExecute(g.viewportFBO);

    // Debug visualization — drawn into the editor viewport after the scene pass,
    // all behind the viewport's "Debug Draw" toggle (colliders, ragdoll bodies,
    // and IK chains/targets for the selected entity).
    glBindFramebuffer(GL_FRAMEBUFFER, g.viewportFBO);
    if (m_Editor && m_Editor->GetContext().showDebugDraw) {
        EditorContext& ctx = m_Editor->GetContext();
        Physics::DrawColliders(scene);
        if (scene.IsPlaying()) Physics::DrawRagdolls(scene);
        DrawIKDebug(scene, ctx.PrimarySelection(), ctx.activeIKChain);
        DrawAudioDebug(scene, ctx.PrimarySelection());
    }
    DebugDraw::Flush(g.proj * g.view);

    // UI canvas preview — draw the scene's widgets into the editor viewport so
    // HUDs can be authored without entering play mode. Visual only: no input or
    // gamepad nav pass, so nothing mutates button state here.
    if (g.uiRenderer && in.showUI) {
        glBindFramebuffer(GL_FRAMEBUFFER, g.viewportFBO);
        glViewport(0, 0, g.fbW, g.fbH);
        UISystem::Resolve(reg, { (float)g.fbW, (float)g.fbH });
        UIRenderSystem::Render(reg, *g.uiRenderer, { (float)g.fbW, (float)g.fbH });
    }

    // --- Game viewport: primary camera entity (play mode only) ---
    if (in.wantGameView) {
        g.view = in.gameView;
        g.proj = in.gameProj;
        CullAndExecute(g.gameViewportFBO);

        // In-game UI HUD: resolve the scene's canvases against the game
        // viewport, hit-test the cursor, and draw every widget on top of the
        // rendered frame. Same FBO, so it composites over the 3D scene.
        if (g.uiRenderer) {
            glBindFramebuffer(GL_FRAMEBUFFER, g.gameViewportFBO);
            glViewport(0, 0, g.fbW, g.fbH);

            UISystem::Resolve(reg, { (float)g.fbW, (float)g.fbH });

            // Map the game-viewport-local cursor (published by GameViewportPanel)
            // into UI framebuffer space for button interaction.
            glm::vec2 uiPointer(-1.0f);
            const glm::vec2 mNorm = in.gameMouseNorm;
            if (mNorm.x >= 0.0f && mNorm.y >= 0.0f && mNorm.x < 1.0f && mNorm.y < 1.0f)
                uiPointer = mNorm * glm::vec2((float)g.fbW, (float)g.fbH);
            UIInputSystem::Update(reg, uiPointer,
                                  Input::IsMouseButtonHeld(MouseButton::Left));

            // Gamepad focus navigation, after the mouse pass so the focus
            // highlight wins when the mouse is idle. Mouse activity releases
            // focus so the two input paths don't fight over button state.
            UINavigationSystem::UINavInput nav = UI::PollGamepadNav();
            nav.mouseMoved = glm::length(Input::GetMouseDelta()) > 0.0f ||
                             Input::IsMouseButtonHeld(MouseButton::Left);
            UINavigationSystem::Update(reg, nav);

            UIRenderSystem::Render(reg, *g.uiRenderer, { (float)g.fbW, (float)g.fbH });
        }
        m_LastGameViewValid = true;
    } else {
        m_LastGameViewValid = false;
    }

    // Restore editor camera matrices — the graph lambdas read these, and the
    // next frame's shadow passes expect the editor view until re-set.
    g.view = in.view;
    g.proj = in.proj;

    // --- Particle preview panel: simulate the selected emitter's working copy
    // (decoupled from play mode) and render it into its own FBO. The panel
    // reports its content-region size a frame late; recreate the FBO on change
    // so the projection aspect matches the displayed image with no distortion. ---
    if (m_Editor) {
        auto& preview = m_Editor->GetParticlePreviewPanel();
        preview.Tick(in.dt);

        glm::ivec2 want = preview.DesiredSize();
        want.x = std::max(want.x, 1);
        want.y = std::max(want.y, 1);
        if (want.x != g.previewW || want.y != g.previewH) {
            makeColorFBO(g.previewFBO, g.previewTex, want.x, want.y);
            g.previewW = want.x;
            g.previewH = want.y;
        }

        glBindFramebuffer(GL_FRAMEBUFFER, g.previewFBO);
        glViewport(0, 0, g.previewW, g.previewH);
        glm::vec3 bg = preview.BackgroundColor();
        glClearColor(bg.r, bg.g, bg.b, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        if (g.particleRenderer && preview.HasParticles()) {
            const auto& em = preview.Emitter();
            if (em.liveCount > 0) {
                g.particleScratch.clear();
                g.particleScratch.reserve(em.liveCount);
                for (std::size_t i = 0; i < em.liveCount; ++i) {
                    const auto& p = em.pool[i];
                    g.particleScratch.push_back({ p.pos, p.size, p.color, p.rotation });
                }
                const Texture& tex = em.texture ? *em.texture : *g.particleTex;
                g.particleRenderer->Begin(preview.ViewMatrix(), preview.ProjMatrix());
                g.particleRenderer->Draw(g.particleScratch, tex, em.blend);
                g.particleRenderer->End();
            }
        }
        preview.SetTexture((ImTextureID)(uintptr_t)g.previewTex, /*flipY*/ true);
    }

    // Clear backbuffer for ImGui — the scene lives in the viewport textures.
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, g.fbW, g.fbH);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glEndQuery(GL_TIME_ELAPSED);
    g.timeQueryIssued[g.timeQuerySlot] = true;
    g.timeQuerySlot = 1 - g.timeQuerySlot;

    glfwSwapBuffers(m_Window);
    GLStats::Finalize();
}
