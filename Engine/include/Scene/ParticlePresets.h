#pragma once
#include "Scene/Components.h"

// Ready-made ParticleEmitterComponent configurations for the roadmap's named
// effects. A preset is purely an authoring convenience: each function returns a
// component with its *config* fields filled in for a particular look. Nothing
// here is runtime state — once applied (e.g. `c = ParticlePresets::Fire();` in
// the inspector, or emplaced directly by gameplay code), the values live in the
// component's already-serialized fields, so presets need no serialization of
// their own. The transient pool is left at its defaults (empty).
//
// The three presets deliberately span both blend modes: Additive light-emitters
// (sparks, fire) and an Alpha-blended soft volume (dust).
namespace ParticlePresets {

// Hit sparks: a one-shot additive burst of small, fast, short-lived points that
// fly out radially and fall. Meant to be triggered per impact (one-shot burst).
inline ParticleEmitterComponent SparkBurst()
{
    ParticleEmitterComponent c;
    c.shape          = EmissionShape::Sphere;
    c.sphereRadius   = 0.05f;
    c.speedRange     = { 4.0f, 9.0f };

    c.spawnRate      = 0.0f;          // burst only
    c.maxParticles   = 64;
    c.burstCount     = 30;
    c.burstInterval  = 0.0f;          // single shot at start

    c.lifetimeRange  = { 0.2f, 0.5f };
    c.sizeRange      = { 0.03f, 0.07f };
    c.endSize        = 0.0f;
    c.gravity        = { 0.0f, -9.0f, 0.0f };

    c.startColor     = { 1.0f, 0.9f, 0.5f, 1.0f };   // bright white-hot
    c.endColor       = { 1.0f, 0.35f, 0.05f, 0.0f }; // fades to ember orange
    c.blend          = Diamond::ParticleBlend::Additive;
    return c;
}

// Dust puff: a soft, slow, alpha-blended clump that drifts up slightly, grows as
// it dissipates, and fades out. One-shot burst — a footstep/impact dust kick.
inline ParticleEmitterComponent DustPuff()
{
    ParticleEmitterComponent c;
    c.shape          = EmissionShape::Cone;
    c.coneAngle      = 60.0f;         // wide, billowing
    c.coneRadius     = 0.1f;
    c.speedRange     = { 0.4f, 1.2f };

    c.spawnRate      = 0.0f;          // burst only
    c.maxParticles   = 64;
    c.burstCount     = 20;
    c.burstInterval  = 0.0f;          // single shot at start

    c.lifetimeRange  = { 0.6f, 1.4f };
    c.sizeRange      = { 0.2f, 0.4f };
    c.endSize        = 0.65f;         // grows as it thins out
    c.gravity        = { 0.0f, 0.3f, 0.0f };  // slight buoyant drift

    c.startColor     = { 0.62f, 0.57f, 0.50f, 0.55f }; // warm translucent gray
    c.endColor       = { 0.62f, 0.57f, 0.50f, 0.0f };  // same hue, fades to nothing
    c.blend          = Diamond::ParticleBlend::Alpha;
    return c;
}

// Fire: a continuous additive plume rising in a narrow cone, each particle
// shrinking from yellow-hot to deep red as it ages — a steady flame.
inline ParticleEmitterComponent Fire()
{
    ParticleEmitterComponent c;
    c.shape          = EmissionShape::Cone;
    c.coneAngle      = 18.0f;         // tight column
    c.coneRadius     = 0.15f;
    c.speedRange     = { 1.0f, 2.0f };

    c.spawnRate      = 60.0f;         // continuous
    c.maxParticles   = 256;
    c.burstCount     = 0;
    c.burstInterval  = 0.0f;

    c.lifetimeRange  = { 0.5f, 1.0f };
    c.sizeRange      = { 0.18f, 0.34f };
    c.endSize        = 0.0f;          // tapers to a point
    c.gravity        = { 0.0f, 1.5f, 0.0f };  // buoyancy: rises against gravity

    c.startColor     = { 1.0f, 0.8f, 0.3f, 1.0f };  // yellow-hot core
    c.endColor       = { 1.0f, 0.1f, 0.0f, 0.0f };  // fades through red to nothing
    c.blend          = Diamond::ParticleBlend::Additive;
    return c;
}

} // namespace ParticlePresets
