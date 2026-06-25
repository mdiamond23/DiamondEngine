#pragma once
#include <glm/glm.hpp>

struct ParticleEmitterComponent;

// Renderer- and scene-agnostic particle simulation. The per-emitter step that
// drives play mode (via ParticleSystem) lives here as a free function so other
// callers — notably the editor's particle preview panel — can tick an emitter
// every frame, independent of the GameSystem lifecycle and play state.
namespace ParticleSim {

// Advances one emitter by dt: lazily sizes the pool to the cap, emits
// continuously + in bursts, integrates under gravity, lerps color/size over
// lifetime, and recycles dead slots via swap-remove. `world` supplies the spawn
// origin and the (orthonormalized) rotation that orients the emission shape.
// No-op when dt <= 0 or maxParticles == 0.
void Step(ParticleEmitterComponent& em, float dt, const glm::mat4& world);

// Drops the emitter back to an empty pool and resets the spawn/burst timers so
// the next Step starts a fresh session (first burst fires immediately).
void Reset(ParticleEmitterComponent& em);

} // namespace ParticleSim
