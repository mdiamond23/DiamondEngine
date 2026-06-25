#pragma once
#include "Scripting.h"

// Simulates every ParticleEmitterComponent during play. Each frame it spawns new
// particles at the emitter's rate (with a fractional accumulator so sub-1/frame
// rates work), integrates position/velocity under gravity, ages them, lerps
// color and size over each particle's lifetime, and recycles dead slots via
// swap-remove. The pool lives on the component (transient, unserialized); the
// engine's particle render pass reads pool[0, liveCount) to draw billboards.
//
// Runs after gameplay/physics so emitters attached to moving entities spawn from
// an up-to-date position.
class ParticleSystem : public GameSystem
{
public:
    void OnStart(Scene& scene) override;
    void OnUpdate(Scene& scene, float dt) override;
    void OnDestroy(Scene& scene) override;

    DECLARE_SYSTEM(ParticleSystem, 500)
};
