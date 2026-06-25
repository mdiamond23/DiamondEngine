#include "Scene/ParticleSystem.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <limits>
#include <random>

namespace {

std::mt19937 g_rng{ std::random_device{}() };

float RandRange(float a, float b)
{
    if (a >= b) return a;
    std::uniform_real_distribution<float> dist(a, b);
    return dist(g_rng);
}

// Uniformly distributed direction on the unit sphere.
glm::vec3 RandUnitVec()
{
    float z   = RandRange(-1.0f, 1.0f);
    float phi = RandRange(0.0f, glm::two_pi<float>());
    float r   = std::sqrt(std::max(0.0f, 1.0f - z * z));
    return { r * std::cos(phi), z, r * std::sin(phi) };
}

// Samples a local-space spawn offset and emission direction for the emitter's
// shape. Axis convention is local +Y; the caller rotates both into world space.
void SampleShape(const ParticleEmitterComponent& em, glm::vec3& outOffset, glm::vec3& outDir)
{
    switch (em.shape) {
        case EmissionShape::Point:
            outOffset = glm::vec3(0.0f);
            outDir    = glm::vec3(0.0f, 1.0f, 0.0f);
            break;

        case EmissionShape::Cone: {
            // Direction: uniform on the spherical cap within coneAngle of +Y.
            const float cosA = std::cos(glm::radians(em.coneAngle));
            const float u    = RandRange(cosA, 1.0f);             // cos(polar) along +Y
            const float phi  = RandRange(0.0f, glm::two_pi<float>());
            const float s    = std::sqrt(std::max(0.0f, 1.0f - u * u));
            outDir = glm::vec3(s * std::cos(phi), u, s * std::sin(phi));

            // Position: a point on the base disc in the local XZ plane (sqrt for
            // uniform area).
            const float br = em.coneRadius * std::sqrt(RandRange(0.0f, 1.0f));
            const float bp = RandRange(0.0f, glm::two_pi<float>());
            outOffset = glm::vec3(br * std::cos(bp), 0.0f, br * std::sin(bp));
            break;
        }

        case EmissionShape::Sphere: {
            const glm::vec3 dir = RandUnitVec();
            const float     rr  = em.sphereRadius * std::cbrt(RandRange(0.0f, 1.0f));
            outOffset = dir * rr;
            outDir    = dir;   // radially outward
            break;
        }

        case EmissionShape::Box:
            outOffset = glm::vec3(RandRange(-em.boxHalfExtents.x, em.boxHalfExtents.x),
                                  RandRange(-em.boxHalfExtents.y, em.boxHalfExtents.y),
                                  RandRange(-em.boxHalfExtents.z, em.boxHalfExtents.z));
            outDir    = glm::vec3(0.0f, 1.0f, 0.0f);
            break;
    }
}

// Initializes one pooled particle from the shape, oriented by `rot` (the
// emitter's orthonormalized world rotation) about `origin`. No-op if full.
void SpawnOne(ParticleEmitterComponent& em, const glm::vec3& origin, const glm::mat3& rot)
{
    if (em.liveCount >= em.maxParticles) return;

    glm::vec3 localOffset, localDir;
    SampleShape(em, localOffset, localDir);

    glm::vec3 worldDir = rot * localDir;
    const float len = glm::length(worldDir);
    worldDir = (len > 1e-6f) ? worldDir / len : glm::vec3(0.0f, 1.0f, 0.0f);

    auto& p     = em.pool[em.liveCount++];
    p.pos       = origin + rot * localOffset;
    p.vel       = worldDir * RandRange(em.speedRange.x, em.speedRange.y);
    p.lifetime  = RandRange(em.lifetimeRange.x, em.lifetimeRange.y);
    p.age       = 0.0f;
    p.startSize = RandRange(em.sizeRange.x, em.sizeRange.y);
    p.size      = p.startSize;
    p.color     = em.startColor;
    p.rotation  = RandRange(0.0f, glm::two_pi<float>());
}

// Drop every emitter back to an empty pool — used at play start/stop so a session
// never inherits stale particles (and the editor isn't left with frozen ones).
void ResetPools(Scene& scene)
{
    for (auto [e, em] : scene.View<ParticleEmitterComponent>().each()) {
        em.pool.clear();
        em.liveCount        = 0;
        em.spawnAccumulator = 0.0f;
        em.burstTimer       = 0.0f;   // first burst fires immediately on play
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

        // Emitter world transform: position is the spawn origin, the rotation
        // (orthonormalized to drop scale) orients the shape. One-frame-late is
        // fine — TransformSystem updates after the gameplay systems.
        const glm::mat4 world  = scene.GetTransformSystem().GetWorldMatrix(e);
        const glm::vec3 origin = glm::vec3(world[3]);
        glm::mat3 rot(world);
        rot[0] = glm::normalize(rot[0]);
        rot[1] = glm::normalize(rot[1]);
        rot[2] = glm::normalize(rot[2]);

        // --- Continuous emit: accumulate fractional spawns for exact rates. ---
        em.spawnAccumulator += em.spawnRate * dt;
        while (em.spawnAccumulator >= 1.0f) {
            em.spawnAccumulator -= 1.0f;
            if (em.liveCount >= em.maxParticles) { em.spawnAccumulator = 0.0f; break; }
            SpawnOne(em, origin, rot);
        }

        // --- Burst emit: a clump at once. Repeats every burstInterval, or fires
        // a single shot at start when the interval is non-positive. ---
        if (em.burstCount > 0) {
            em.burstTimer -= dt;
            if (em.burstTimer <= 0.0f) {
                for (std::size_t k = 0; k < em.burstCount; ++k) {
                    if (em.liveCount >= em.maxParticles) break;
                    SpawnOne(em, origin, rot);
                }
                em.burstTimer = (em.burstInterval > 0.0f)
                              ? em.burstTimer + em.burstInterval
                              : std::numeric_limits<float>::max();   // one-shot
            }
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
