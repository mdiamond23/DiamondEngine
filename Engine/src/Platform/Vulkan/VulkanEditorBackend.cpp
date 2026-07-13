#include "Editor/EditorBackend.h"

// The Vulkan implementation of the shared editor loop's backend interface
// (editor-wiring step 3). Lives inside the engine — like the demos — because
// the Vulkan backend's headers, defines, and volk include path are private to
// MyEngine; the app only sees CreateVulkanEditorBackend().
//
// This TU is compiled unconditionally: without the Vulkan backend it collapses
// to a stub factory (same pattern as SceneRenderer.cpp), so Sandbox links
// either way.

#ifdef DIAMOND_ENABLE_VULKAN

#include "Editor/ThumbnailService.h"
#include "Renderer/SceneRenderer.h"
#include "Renderer/RendererAPI.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHIEnums.h"
#include "Renderer/TextureData.h"
#include "Core/Camera.h"
#include "Core/Input.h"
#include "Scene/Scene.h"
#include "Scene/UISystem.h"
#include "Scene/UIRenderSystem.h"
#include "Scene/UIInputSystem.h"
#include "Scene/UINavigationSystem.h"
#include "Scene/DebugVisualization.h"
#include "Scene/Physics/PhysicsAPI.h"

#include "Platform/Vulkan/VulkanImGuiLayer.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Platform/Vulkan/Resources/VulkanRenderer2D.h"
#include "Platform/Vulkan/Resources/VulkanTexture2D.h"
#include "Platform/Vulkan/Resources/VulkanThumbnailRenderer.h"

#include "imgui.h"
#include "backends/imgui_impl_vulkan.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>

namespace Diamond {

namespace {

// Vulkan ThumbnailService (editor-wiring step 4): mesh/material previews bake
// through the VulkanThumbnailRenderer's one-shot ImmediateSubmit path; texture
// previews ride the routed Texture::Create upload. Every returned handle is an
// ImGui_ImplVulkan_AddTexture descriptor set, ready for ImGui::Image.
//
// Lifetime: created after the ImGui Vulkan binding, destroyed (in Shutdown)
// after WaitIdle and BEFORE the binding goes down — the destructor returns the
// live descriptor sets to ImGui's pool.
class VulkanThumbnailService final : public ThumbnailService {
public:
    explicit VulkanThumbnailService(VulkanRHIDevice* device)
        : m_Device(device), m_Renderer(device, DIAMOND_VULKAN_SHADER_DIR) {}

    ~VulkanThumbnailService() override {
        for (auto& [id, rec] : m_Records)
            ImGui_ImplVulkan_RemoveTexture(rec.set);
    }

    uint64_t CreateTextureThumbnail(const std::string& path) override {
        std::shared_ptr<Texture> tex = Texture::Create(path, false);
        const auto* vk = dynamic_cast<const VulkanTexture2D*>(tex.get());
        if (!vk || !vk->Rhi()) return 0;
        auto* rhi = static_cast<VulkanRHITexture*>(vk->Rhi());
        // Static upload: every frame slot aliases image 0.
        return Track(rhi->Sampler(), rhi->View(0), VK_NULL_HANDLE, std::move(tex));
    }

    uint64_t CreateMeshThumbnail(const std::vector<MeshData>& meshes) override {
        const auto thumb = m_Renderer.RenderMesh(meshes);
        if (!thumb.view) return 0;
        return Track(thumb.sampler, thumb.view, thumb.view, nullptr);
    }

    uint64_t CreateMaterialThumbnail(const PBRMaterial& mat) override {
        const auto thumb = m_Renderer.RenderMaterial(mat);
        if (!thumb.view) return 0;
        return Track(thumb.sampler, thumb.view, thumb.view, nullptr);
    }

    void Release(uint64_t id) override {
        auto it = m_Records.find(id);
        if (it == m_Records.end()) return;
        // The set (and for bakes, the image) may still be referenced by an
        // in-flight frame's draw data; releases are rare (material re-bakes
        // on inspector edit-release), so a WaitIdle keeps this simple.
        m_Device->WaitIdle();
        ImGui_ImplVulkan_RemoveTexture(it->second.set);
        if (it->second.bakedView) m_Renderer.Release(it->second.bakedView);
        m_Records.erase(it);
    }

private:
    struct Record {
        VkDescriptorSet          set       = VK_NULL_HANDLE;
        VkImageView              bakedView = VK_NULL_HANDLE;   // owned by m_Renderer
        std::shared_ptr<Texture> texture;                      // keeps uploads alive
    };

    uint64_t Track(VkSampler sampler, VkImageView view, VkImageView bakedView,
                   std::shared_ptr<Texture> tex) {
        VkDescriptorSet ds = ImGui_ImplVulkan_AddTexture(
            sampler, view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        if (!ds) return 0;
        const auto id = (uint64_t)(uintptr_t)ds;
        m_Records[id] = { ds, bakedView, std::move(tex) };
        return id;
    }

    VulkanRHIDevice*                     m_Device;
    VulkanThumbnailRenderer              m_Renderer;
    std::unordered_map<uint64_t, Record> m_Records;
};

class VulkanEditorBackend final : public EditorBackend {
public:
    ~VulkanEditorBackend() override { Shutdown(); }

    bool Init(uint32_t width, uint32_t height, const char* title) override {
        if (!glfwVulkanSupported()) {
            spdlog::critical("[VulkanEditor] GLFW reports no Vulkan loader present");
            return false;
        }

        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        m_Window = glfwCreateWindow((int)width, (int)height, title, nullptr, nullptr);
        if (!m_Window) {
            spdlog::critical("[VulkanEditor] glfwCreateWindow failed");
            return false;
        }

        m_Device = RHIDevice::Create(m_Window, RHIBackend::Vulkan);
        if (!m_Device) {
            spdlog::critical("[VulkanEditor] device creation failed");
            glfwDestroyWindow(m_Window);
            m_Window = nullptr;
            return false;
        }
        m_VkDevice = static_cast<VulkanRHIDevice*>(m_Device.get());

        glfwSetWindowUserPointer(m_Window, this);
        glfwSetFramebufferSizeCallback(m_Window, [](GLFWwindow* w, int, int) {
            auto* self = static_cast<VulkanEditorBackend*>(glfwGetWindowUserPointer(w));
            if (self && self->m_Device) self->m_Device->NotifyResize();
        });
        glfwSetDropCallback(m_Window, [](GLFWwindow* w, int count, const char** paths) {
            auto* self = static_cast<VulkanEditorBackend*>(glfwGetWindowUserPointer(w));
            if (self && self->m_DropHandler) self->m_DropHandler(count, paths);
        });

        // Route the static factories to the Vulkan backend before the app loads
        // any assets: Mesh::Create hands out CPU-retained meshes the
        // SceneRenderer uploads lazily, and the Texture factory goes through
        // this device so image/font bakes come out Vulkan-backed.
        RendererAPI::SetAPI(RendererAPI::API::Vulkan);
        Texture::SetResourceDevice(m_Device.get());

        // The scene renders offscreen at the window-framebuffer size (same
        // policy as the GL editor: panels stretch the image); Tonemap lands in
        // OutputColor(), which ImGui samples inside the viewport panel — the
        // backbuffer belongs to ImGui.
        int fbW = 0, fbH = 0;
        glfwGetFramebufferSize(m_Window, &fbW, &fbH);
        m_SceneW = (uint32_t)std::max(fbW, 64);
        m_SceneH = (uint32_t)std::max(fbH, 64);
        m_WantW  = m_SceneW;
        m_WantH  = m_SceneH;

        m_Renderer = SceneRenderer::Create(m_Device.get(), m_SceneW, m_SceneH,
                                           /*offscreen*/ true);
        if (!m_Renderer) {
            spdlog::critical("[VulkanEditor] SceneRenderer::Create returned null");
            ReleaseDevice();
            return false;
        }

        // In-game UI batchers, recorded by the SceneRenderer's UI2D passes into
        // the RGBA8 scene outputs. One per render view: both overlays run in
        // the same frame, and a Renderer2D's dynamic vertex stream is written
        // once per frame slot.
        m_R2D = std::make_unique<VulkanRenderer2D>(
            m_Device.get(), DIAMOND_VULKAN_SHADER_DIR, RHIFormat::RGBA8);
        m_R2DGame = std::make_unique<VulkanRenderer2D>(
            m_Device.get(), DIAMOND_VULKAN_SHADER_DIR, RHIFormat::RGBA8);

        // Editor-viewport UI preview — visual only (no input pass), gated by
        // the viewport's "Show UI" toggle, mirroring the GL editor.
        m_Renderer->SetUIOverlay([this](RHICommandList* cmd) {
            if (!m_Frame.scene || !m_Frame.showUI) return;
            auto& reg = m_Frame.scene->GetRegistry();
            const glm::vec2 screen { (float)m_SceneW, (float)m_SceneH };
            m_R2D->SetCommandList(cmd);
            UISystem::Resolve(reg, screen);
            UIRenderSystem::Render(reg, *m_R2D, screen);
        });

        // Game-view HUD — the interactive in-game UI during play: resolve the
        // canvases at the game view's size, hit-test the game-panel cursor,
        // run gamepad focus nav, then draw. Runs only when the game view
        // renders (play mode + primary camera), so button state mutates once
        // per frame, exactly like the GL editor's game viewport.
        m_Renderer->SetGameUIOverlay([this](RHICommandList* cmd) {
            if (!m_Frame.scene || !m_Frame.playing) return;
            auto& reg = m_Frame.scene->GetRegistry();
            const glm::vec2 screen { (float)m_GameW, (float)m_GameH };
            m_R2DGame->SetCommandList(cmd);
            UISystem::Resolve(reg, screen);

            glm::vec2 pointer { -1.0f, -1.0f };
            const glm::vec2 n = m_Frame.gameMouseNorm;
            if (n.x >= 0.0f && n.y >= 0.0f && n.x < 1.0f && n.y < 1.0f)
                pointer = n * screen;
            UIInputSystem::Update(reg, pointer,
                                  Input::IsMouseButtonHeld(MouseButton::Left));

            UINavigationSystem::UINavInput nav = UI::PollGamepadNav();
            nav.mouseMoved = glm::length(Input::GetMouseDelta()) > 0.0f ||
                             Input::IsMouseButtonHeld(MouseButton::Left);
            UINavigationSystem::Update(reg, nav);

            UIRenderSystem::Render(reg, *m_R2DGame, screen);
        });

        // Particle preview panel (editor-wiring step 4 tail): the loop hands us
        // the panel's already-extracted per-frame state (see EditorFrameInput);
        // this just runs Begin/Draw/End against whatever ParticlePreview target
        // EnsureParticlePreview built this frame.
        m_Renderer->SetParticlePreviewOverlay([this](ParticleRenderer& particles) {
            if (!m_Frame.previewActive || m_Frame.previewParticles.empty()) return;
            const std::shared_ptr<Texture> fallback = m_Renderer->DefaultParticleTexture();
            const Texture& tex = m_Frame.previewTexture ? *m_Frame.previewTexture : *fallback;
            particles.Begin(m_Frame.previewView, m_Frame.previewProj);
            particles.Draw(m_Frame.previewParticles, tex, m_Frame.previewBlend);
            particles.End();
        });

        m_ImGui.Init(m_Window, m_VkDevice);
        if (!m_ImGui.IsInitialized()) {
            spdlog::critical("[VulkanEditor] ImGui Vulkan binding failed");
            m_R2DGame.reset();
            m_R2D.reset();
            m_Renderer.reset();
            ReleaseDevice();
            return false;
        }

        RegisterViewImages(m_ViewportSets, m_Renderer->OutputColor());

        // Asset previews for the content browser / inspector — needs the ImGui
        // binding up (its descriptor pool backs AddTexture).
        m_Thumbs = std::make_unique<VulkanThumbnailService>(m_VkDevice);
        return true;
    }

    void Shutdown() override {
        if (!m_Device) return;
        // Teardown: everything VMA-backed dies before the device; ImGui's
        // descriptors (viewport/game sets included) die with its own pool, and
        // its Shutdown destroys the ImGui context. The thumbnail service goes
        // first — its destructor hands descriptor sets back to ImGui's pool.
        m_Device->WaitIdle();
        m_Thumbs.reset();
        m_ImGui.Shutdown();
        m_R2DGame.reset();
        m_R2D.reset();
        m_Renderer.reset();
        ReleaseDevice();
    }

    GLFWwindow* Window() const override { return m_Window; }
    const char* Name() const override { return "Vulkan"; }

    void SetDropHandler(std::function<void(int, const char**)> handler) override {
        m_DropHandler = std::move(handler);
    }

    void BeginImGuiFrame() override {
        // Apply settled window resizes BEFORE any ImGui state references the
        // old targets: SceneViewImage() hands this frame's descriptor to the
        // viewport panel, and re-registering (RemoveTexture) after
        // ImGui::Render would leave the baked draw data pointing at a freed
        // descriptor set. Resize's internal WaitIdle makes the swap safe here.
        ApplyPendingResize();
        m_ImGui.BeginFrame();
    }

    void DrawRendererSettings() override {
        ImGui::Text("Backend: Vulkan (deferred, RHI render graph)");
        ImGui::Text("Scene output: %ux%u", m_SceneW, m_SceneH);
        ImGui::Separator();
        // Global exposure (HDR scale before the ACES tonemap; 1.0 = GL parity).
        if (ImGui::SliderFloat("Exposure", &m_Exposure, 0.1f, 2.0f, "%.2f"))
            m_Renderer->SetExposure(m_Exposure);
    }

    void RenderFrame(const EditorFrameInput& input) override {
        // Consumed by the overlay lambdas while the graph records, later this call.
        m_Frame = input;

        // Shader hot-reload: F5 force-recompiles everything; otherwise poll
        // source file mtimes a couple times a second and reload whatever
        // changed on disk. Same policy as the GL editor's ShaderLibrary.
        if (Input::IsKeyPressed(Key::F5)) {
            m_Renderer->ReloadAllShaders();
        } else {
            m_ShaderReloadTimer += input.dt;
            if (m_ShaderReloadTimer >= 0.5f) {
                m_Renderer->ReloadChangedShaders();
                m_ShaderReloadTimer = 0.0f;
            }
        }

        // The game view renders only during play with a primary camera (the GL
        // editor's policy); it is lazily created on the first enable.
        m_Renderer->SetGameViewEnabled(input.wantGameView);
        if (input.wantGameView && !m_GameViewCreated) {
            m_GameViewCreated = true;
            m_GameW = m_SceneW;   // created at the main view's size
            m_GameH = m_SceneH;
            RegisterViewImages(m_GameSets, m_Renderer->GameViewColor());
        }

        // Particle preview panel: resize (WaitIdle + rebuild) must happen here,
        // before BeginFrame — never mid-record. Re-register ImGui's descriptors
        // only on an actual size change; the underlying texture is otherwise the
        // same pooled object every frame.
        const uint32_t previewW = (uint32_t)std::max(input.previewSize.x, 1);
        const uint32_t previewH = (uint32_t)std::max(input.previewSize.y, 1);
        if (previewW != m_PreviewW || previewH != m_PreviewH) {
            m_PreviewW = previewW;
            m_PreviewH = previewH;
            m_Renderer->EnsureParticlePreview(previewW, previewH, input.previewBgColor);
            RegisterViewImages(m_PreviewSets, m_Renderer->ParticlePreviewColor());
            m_PreviewCreated = true;
        }

        // Collider/ragdoll/IK/audio debug wireframes — accumulate into
        // DebugDraw's generic buffer before the frame records; the main view's
        // VulkanDebugDrawPass picks them up in RenderView via SetFrameData.
        // Mirrors the GL editor's DrawColliders/DrawRagdolls/DrawIKDebug/
        // DrawAudioDebug + DebugDraw::Flush ordering.
        if (input.showDebugDraw) {
            Physics::DrawColliders(*input.scene);
            if (input.scene->IsPlaying()) Physics::DrawRagdolls(*input.scene);
            DrawIKDebug(*input.scene, input.selectedEntity, input.activeIKChain);
            DrawAudioDebug(*input.scene, input.selectedEntity);
        }

        m_Renderer->RenderToSwapchain(*input.scene, *input.editorCamera,
            [this](RHICommandList* cmd) { m_ImGui.Submit(cmd); });
    }

    EditorViewImage SceneViewImage() const override {
        // Per-frame-in-flight descriptor: each frame ImGui must sample the copy
        // of the offscreen target the graph writes this frame.
        return { (uint64_t)(uintptr_t)m_ViewportSets[m_VkDevice->CurrentFrame()],
                 /*flipY*/ false };
    }

    EditorViewImage GameViewImage() const override {
        // GameViewActive reflects the last rendered frame; while false the
        // image holds no valid frame, so let the panel show its placeholder.
        if (!m_GameViewCreated || !m_Renderer->GameViewActive()) return {};
        return { (uint64_t)(uintptr_t)m_GameSets[m_VkDevice->CurrentFrame()],
                 /*flipY*/ false };
    }

    EditorViewImage ParticlePreviewImage() const override {
        if (!m_PreviewCreated) return {};
        return { (uint64_t)(uintptr_t)m_PreviewSets[m_VkDevice->CurrentFrame()],
                 /*flipY*/ false };
    }

    ThumbnailService* Thumbnails() override { return m_Thumbs.get(); }

    RendererStats GetStats() const override {
        return m_Renderer ? m_Renderer->GetStats() : RendererStats{};
    }

    void InvalidateMaterial(const PBRMaterial* mat) override {
        m_Renderer->InvalidateMaterial(mat);
    }

    void InvalidateSceneCaches() override {
        m_Renderer->InvalidateSceneCaches();
    }

    void WaitIdle() override {
        if (m_Device) m_Device->WaitIdle();
    }

private:
    static constexpr int kResizeSettleFrames = 15;

    // Track the window framebuffer size, debounced: a Resize is a WaitIdle +
    // pass rebuild, so only fire once the size has settled for a run of frames.
    void ApplyPendingResize() {
        int fbW = 0, fbH = 0;
        glfwGetFramebufferSize(m_Window, &fbW, &fbH);
        if (fbW <= 0 || fbH <= 0) return;   // minimized

        if ((uint32_t)fbW == m_WantW && (uint32_t)fbH == m_WantH) {
            ++m_SizeStableFrames;
        } else {
            m_WantW = (uint32_t)fbW;
            m_WantH = (uint32_t)fbH;
            m_SizeStableFrames = 0;
        }
        if ((m_WantW != m_SceneW || m_WantH != m_SceneH) &&
            m_SizeStableFrames >= kResizeSettleFrames) {
            m_SceneW = m_WantW;
            m_SceneH = m_WantH;
            m_Renderer->Resize(m_SceneW, m_SceneH);
            RegisterViewImages(m_ViewportSets, m_Renderer->OutputColor());
            if (m_GameViewCreated) {
                m_GameW = m_SceneW;
                m_GameH = m_SceneH;
                m_Renderer->ResizeGameView(m_GameW, m_GameH);
                RegisterViewImages(m_GameSets, m_Renderer->GameViewColor());
            }
        }
    }

    using ImageSets = std::array<VkDescriptorSet, VulkanRHIDevice::kFramesInFlight>;

    // One descriptor per frame-in-flight per scene image: the offscreen targets
    // are per-frame images internally, transitioned to SHADER_READ_ONLY before
    // ImGui draws (by the EditorOverlay pass's read for the main viewport, by
    // the SceneRenderer's post-execute transition for the game view).
    void RegisterViewImages(ImageSets& sets, RHITexture* rhiTex) {
        auto* tex = static_cast<VulkanRHITexture*>(rhiTex);
        for (uint32_t i = 0; i < VulkanRHIDevice::kFramesInFlight; ++i) {
            if (sets[i]) ImGui_ImplVulkan_RemoveTexture(sets[i]);
            sets[i] = ImGui_ImplVulkan_AddTexture(
                tex->Sampler(), tex->View(i), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
    }

    void ReleaseDevice() {
        Texture::SetResourceDevice(nullptr);
        m_Device.reset();
        m_VkDevice = nullptr;
        if (m_Window) {
            glfwDestroyWindow(m_Window);
            m_Window = nullptr;
        }
    }

    GLFWwindow*                    m_Window   = nullptr;
    std::unique_ptr<RHIDevice>     m_Device;
    VulkanRHIDevice*               m_VkDevice = nullptr;
    std::unique_ptr<SceneRenderer> m_Renderer;
    std::unique_ptr<VulkanRenderer2D> m_R2D;      // main-view UI preview
    std::unique_ptr<VulkanRenderer2D> m_R2DGame;  // game-view HUD
    VulkanImGuiLayer               m_ImGui;
    std::unique_ptr<VulkanThumbnailService> m_Thumbs;

    std::function<void(int, const char**)> m_DropHandler;

    EditorFrameInput m_Frame;   // latest frame's state, read by overlay lambdas

    uint32_t m_SceneW = 0, m_SceneH = 0;   // main-view render size
    uint32_t m_GameW  = 0, m_GameH  = 0;   // game-view render size
    uint32_t m_WantW  = 0, m_WantH  = 0;   // debounced framebuffer size
    int      m_SizeStableFrames = 0;
    bool     m_GameViewCreated  = false;

    uint32_t m_PreviewW = 0, m_PreviewH = 0;   // particle-preview render size
    bool     m_PreviewCreated = false;

    float m_ShaderReloadTimer = 0.0f;   // throttles the hot-reload mtime poll

    ImageSets m_ViewportSets {};
    ImageSets m_GameSets {};
    ImageSets m_PreviewSets {};

    float m_Exposure = 1.0f;
};

} // namespace

std::unique_ptr<EditorBackend> CreateVulkanEditorBackend() {
    return std::make_unique<VulkanEditorBackend>();
}

} // namespace Diamond

#else  // !DIAMOND_ENABLE_VULKAN

#include <spdlog/spdlog.h>

namespace Diamond {
std::unique_ptr<EditorBackend> CreateVulkanEditorBackend() {
    spdlog::critical("--vulkan requested but this build has no Vulkan backend "
                     "(DIAMOND_ENABLE_VULKAN is off)");
    return nullptr;
}
} // namespace Diamond

#endif
