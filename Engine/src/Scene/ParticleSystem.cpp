#include "Scene/ParticleSystem.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <random>

namespace {

std::mt19937 g_rng{ std::random_device{}() };

float RandRange(float a, float b)
{
    if (a >= b) return a;
    std::uniform_real_distribution<float> dist(a, b);
    return dist(g_rng);
}

glm::vec3 RandVec(const glm::vec3& a, const glm::vec3& b)
{
    return { RandRange(a.x, b.x), RandRange(a.y, b.y), RandRange(a.z, b.z) };
}

// Drop every emitter back to an empty pool — used at play start/stop so a session
// never inherits stale particles (and the editor isn't left with frozen ones).
void ResetPools(Scene& scene)
{
    for (auto [e, em] : scene.View<ParticleEmitterComponent>().each()) {
        em.pool.clear();
        em.liveCount        = 0;
        em.spawnAccumulator = 0.0f;
    }
}

} // namespace

void ParticleSystem::OnStart(Scene& scene)
{
    ResetPools(scene);
}

void ParticleSystem::OnDestroy(Scene& scene)
{
    ResetPools(scene);
}

void ParticleSystem::OnUpdate(Scene& scene, float dt)
{
    if (dt <= 0.0f) return;

    for (auto [e, em] : scene.View<ParticleEmitterComponent>().each()) {
        if (em.maxParticles == 0) { em.liveCount = 0; continue; }

        // Lazily size the pool to the configured cap; clamp the live span if the
        // cap shrank (e.g. edited at runtime).
        if (em.pool.size() != em.maxParticles)
            em.pool.resize(em.maxParticles);
        if (em.liveCount > em.maxParticles)
            em.liveCount = em.maxParticles;

        // Spawn origin = emitter world position. A one-frame-late transform is
        // fine here (TransformSystem updates after the gameplay systems).
        const glm::vec3 origin = glm::vec3(scene.GetTransformSystem().GetWorldMatrix(e)[3]);

        // --- Emit: accumulate fractional spawns so non-integer rates are exact. ---
        em.spawnAccumulator += em.spawnRate * dt;
        while (em.spawnAccumulator >= 1.0f) {
            em.spawnAccumulator -= 1.0f;
            if (em.liveCount >= em.maxParticles) { em.spawnAccumulator = 0.0f; break; }

            auto& p     = em.pool[em.liveCount++];
            p.pos       = origin;
            p.vel       = RandVec(em.velocityMin, em.velocityMax);
            p.lifetime  = RandRange(em.lifetimeRange.x, em.lifetimeRange.y);
            p.age       = 0.0f;
            p.startSize = RandRange(em.sizeRange.x, em.sizeRange.y);
            p.size      = p.startSize;
            p.color     = em.startColor;
            p.rotation  = RandRange(0.0f, glm::two_pi<float>());
        }

        // --- Integrate / age / lerp / kill. Swap-remove keeps the live span
        // contiguous; the swapped-in particle is re-processed at the same index. ---
        for (std::size_t i = 0; i < em.liveCount; ) {
            auto& p = em.pool[i];
            p.age += dt;
            if (p.age >= p.lifetime) {
                em.pool[i] = em.pool[--em.liveCount];
                continue;
            }

            p.vel += em.gravity * dt;
            p.pos += p.vel * dt;

            const float t = p.age / p.lifetime;
            p.color = glm::mix(em.startColor, em.endColor, t);
            p.size  = glm::mix(p.startSize, em.endSize, t);
            ++i;
        }
    }
}
