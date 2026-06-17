#include "MeshThumbnailRenderer.h"

#ifndef ENGINE_SHADERS_DIR
#define ENGINE_SHADERS_DIR "."
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <glad/gl.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

MeshThumbnailRenderer::MeshThumbnailRenderer() {
    m_Shader = Diamond::Shader::Create(
        ENGINE_SHADERS_DIR "/Thumbnail/thumbnail.vert",
        ENGINE_SHADERS_DIR "/Thumbnail/thumbnail.frag");
    m_MaterialShader = Diamond::Shader::Create(
        ENGINE_SHADERS_DIR "/Thumbnail/material_preview.vert",
        ENGINE_SHADERS_DIR "/Thumbnail/material_preview.frag");

    if (!m_Shader) return;

    glGenFramebuffers(1, &m_FBO);
    glBindFramebuffer(GL_FRAMEBUFFER, m_FBO);

    // Shared depth renderbuffer — reattached each render, same size as kSize
    glGenRenderbuffers(1, &m_Depth);
    glBindRenderbuffer(GL_RENDERBUFFER, m_Depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, kSize, kSize);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_Depth);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

MeshThumbnailRenderer::~MeshThumbnailRenderer() {
    if (m_Depth) glDeleteRenderbuffers(1, &m_Depth);
    if (m_FBO)   glDeleteFramebuffers(1,  &m_FBO);
}

uint32_t MeshThumbnailRenderer::Render(const std::vector<Diamond::MeshData>& meshes) {
    if (!m_Shader || meshes.empty() || !m_FBO) return 0;

    // Compute aggregate AABB across all sub-meshes
    Diamond::AABB aabb;
    for (const auto& md : meshes) {
        Diamond::AABB b = md.ComputeAABB();
        aabb.min = glm::min(aabb.min, b.min);
        aabb.max = glm::max(aabb.max, b.max);
    }

    glm::vec3 center = (aabb.min + aabb.max) * 0.5f;
    float radius = glm::length(aabb.max - aabb.min) * 0.5f;
    if (radius < 1e-5f) radius = 1.0f;

    // Pick camera direction based on AABB extents: face the smallest axis so the
    // largest face of the bounding box fills the thumbnail.
    glm::vec3 ext = aabb.max - aabb.min;
    glm::vec3 camDir;
    if (ext.x <= ext.y && ext.x <= ext.z)
        camDir = glm::normalize(glm::vec3(1.0f, 0.5f, 0.4f)); // thin in X → look from side
    else if (ext.y <= ext.x && ext.y <= ext.z)
        camDir = glm::normalize(glm::vec3(0.5f, 1.0f, 0.5f)); // thin in Y → look from above
    else
        camDir = glm::normalize(glm::vec3(0.4f, 0.5f, 1.0f)); // thin in Z → look from front
    glm::vec3 camPos = center + camDir * (radius * 2.8f);

    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view  = glm::lookAt(camPos, center, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 proj  = glm::perspective(glm::radians(45.0f), 1.0f,
                                        radius * 0.01f, radius * 20.0f);
    glm::mat3 normalMat = glm::mat3(glm::transpose(glm::inverse(model)));

    // Create the destination texture
    GLuint texID = 0;
    glGenTextures(1, &texID);
    glBindTexture(GL_TEXTURE_2D, texID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, kSize, kSize, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Attach texture to FBO colour slot
    glBindFramebuffer(GL_FRAMEBUFFER, m_FBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, texID, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDeleteTextures(1, &texID);
        return 0;
    }

    // Save viewport, set ours
    GLint prevViewport[4];
    glGetIntegerv(GL_VIEWPORT, prevViewport);
    glViewport(0, 0, kSize, kSize);

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_Shader->Bind();
    m_Shader->SetMat4("model",        model);
    m_Shader->SetMat4("view",         view);
    m_Shader->SetMat4("projection",   proj);
    m_Shader->SetMat3("normalMatrix", normalMat);
    m_Shader->SetVec3("cameraPos",    camPos);

    for (const auto& md : meshes) {
        auto mesh = Diamond::Mesh::Create(md);
        mesh->Draw(*m_Shader);
    }

    m_Shader->Unbind();

    // Detach texture and restore state
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, 0, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);

    return texID;
}

uint32_t MeshThumbnailRenderer::RenderMaterial(const Diamond::PBRMaterial& mat) {
    if (!m_MaterialShader || !m_FBO) return 0;

    // Build the preview sphere once; reuse its bounds for framing every material.
    static const Diamond::MeshData kSphere = Diamond::MeshData::UVSphere();
    static const Diamond::AABB     kAabb   = kSphere.ComputeAABB();
    if (!m_Sphere) m_Sphere = Diamond::Mesh::Create(kSphere);

    glm::vec3 center = (kAabb.min + kAabb.max) * 0.5f;
    float     radius = glm::length(kAabb.max - kAabb.min) * 0.5f;
    if (radius < 1e-5f) radius = 1.0f;

    glm::vec3 camDir = glm::normalize(glm::vec3(0.4f, 0.5f, 1.0f));
    glm::vec3 camPos = center + camDir * (radius * 2.8f);

    glm::mat4 model     = glm::mat4(1.0f);
    glm::mat4 view      = glm::lookAt(camPos, center, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 proj      = glm::perspective(glm::radians(45.0f), 1.0f, radius * 0.01f, radius * 20.0f);
    glm::mat3 normalMat = glm::mat3(glm::transpose(glm::inverse(model)));

    GLuint texID = 0;
    glGenTextures(1, &texID);
    glBindTexture(GL_TEXTURE_2D, texID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, kSize, kSize, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, m_FBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texID, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDeleteTextures(1, &texID);
        return 0;
    }

    GLint prevViewport[4];
    glGetIntegerv(GL_VIEWPORT, prevViewport);
    glViewport(0, 0, kSize, kSize);

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto& sh = *m_MaterialShader;
    sh.Bind();
    sh.SetMat4("projection",   proj);
    sh.SetMat4("view",         view);
    sh.SetMat4("model",        model);
    sh.SetMat3("normalMatrix", normalMat);
    sh.SetVec3("cameraPos",    camPos);
    sh.SetFloat("uvScale",     mat.UVScale);

    // Bind only the maps that exist; the shader falls back to scalars otherwise.
    auto bindTex = [&](const std::shared_ptr<Diamond::Texture>& t, int slot,
                       const char* sampler, const char* hasFlag) {
        bool has = static_cast<bool>(t);
        sh.SetInt(hasFlag, has ? 1 : 0);
        if (has) { t->Bind(slot); sh.SetInt(sampler, slot); }
    };
    bindTex(mat.Albedo,    0, "albedoMap",    "hasAlbedo");
    bindTex(mat.Metallic,  2, "metallicMap",  "hasMetallic");
    bindTex(mat.Roughness, 3, "roughnessMap", "hasRoughness");
    bindTex(mat.AO,        4, "aoMap",        "hasAO");
    bindTex(mat.Emissive,  5, "emissiveMap",  "hasEmissive");
    sh.SetVec3("albedoFallback",     glm::vec3(0.8f, 0.8f, 0.8f));
    sh.SetFloat("metallicFallback",  0.0f);
    sh.SetFloat("roughnessFallback", 0.5f);
    sh.SetFloat("emissiveStrength",  mat.Emissive ? mat.EmissiveStrength : 0.0f);

    m_Sphere->Draw(sh);
    sh.Unbind();

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);

    return texID;
}
