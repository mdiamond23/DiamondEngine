#include "Platform/OpenGL/Passes/Deferred/OpenGLSSAOPass.h"
#include "Renderer/MeshData.h"
#include <glad/gl.h>
#include <glm/gtc/matrix_transform.hpp>
#include <random>
#include <string>

namespace Diamond {

OpenGLSSAOPass::OpenGLSSAOPass()
    : m_Quad(MeshData::FullscreenQuad())
{
    std::uniform_real_distribution<float> rng(0.0f, 1.0f);
    std::default_random_engine gen(42u);

    // Hemisphere kernel — 64 samples biased toward the origin
    m_Kernel.reserve(64);
    for (int i = 0; i < 64; ++i)
    {
        glm::vec3 sample(
            rng(gen) * 2.0f - 1.0f,
            rng(gen) * 2.0f - 1.0f,
            rng(gen)
        );
        sample = glm::normalize(sample) * rng(gen);

        // Accelerating lerp — denser samples near the origin
        float scale = float(i) / 64.0f;
        scale = glm::mix(0.1f, 1.0f, scale * scale);
        m_Kernel.push_back(sample * scale);
    }

    // 4×4 random rotation vectors (rotated around the view-space Z axis)
    std::vector<glm::vec3> noise;
    noise.reserve(16);
    for (int i = 0; i < 16; ++i)
        noise.push_back({ rng(gen) * 2.0f - 1.0f, rng(gen) * 2.0f - 1.0f, 0.0f });

    glGenTextures(1, &m_NoiseTex);
    glBindTexture(GL_TEXTURE_2D, m_NoiseTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, 4, 4, 0, GL_RGB, GL_FLOAT, noise.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
}

OpenGLSSAOPass::~OpenGLSSAOPass()
{
    if (m_NoiseTex) glDeleteTextures(1, &m_NoiseTex);
}

void OpenGLSSAOPass::RenderSSAO(
    const OpenGLShader& shader,
    uint32_t gViewPosTex,
    uint32_t gViewNormalTex,
    const glm::mat4& projection,
    uint32_t fbo,
    int w, int h)
{
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(0, 0, w, h);
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);

    shader.Bind();
    shader.SetInt("gViewPos",    0);
    shader.SetInt("gViewNormal", 1);
    shader.SetInt("texNoise",    2);
    shader.SetMat4("projection", projection);
    shader.SetVec2("noiseScale", glm::vec2(float(w) / 4.0f, float(h) / 4.0f));

    for (int i = 0; i < 64; ++i)
        shader.SetVec3("samples[" + std::to_string(i) + "]", m_Kernel[i]);

    glActiveTexture(GL_TEXTURE0); 
    glBindTexture(GL_TEXTURE_2D, gViewPosTex);
    
    glActiveTexture(GL_TEXTURE1); 
    glBindTexture(GL_TEXTURE_2D, gViewNormalTex);

    glActiveTexture(GL_TEXTURE2); 
    glBindTexture(GL_TEXTURE_2D, m_NoiseTex);

    m_Quad.Draw(shader);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void OpenGLSSAOPass::RenderBlur(
    const OpenGLShader& blurShader,
    uint32_t ssaoRawTex,
    uint32_t fbo,
    int w, int h)
{
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(0, 0, w, h);
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);

    blurShader.Bind();
    blurShader.SetInt("ssaoInput", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, ssaoRawTex);

    m_Quad.Draw(blurShader);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace Diamond
