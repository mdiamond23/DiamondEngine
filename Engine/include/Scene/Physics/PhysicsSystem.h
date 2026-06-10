#pragma once
#include <memory>
#include "Scene/Scripting.h"
#include "Scene/Scene.h"

// Jolt types are never exposed here — kept PRIVATE via PIMPL so that
// Engine/include/ consumers (Sandbox) don't need the Jolt include path.
class ContactListener;

class PhysicsSystem : public GameSystem {
    DECLARE_SYSTEM(PhysicsSystem, 200)
public:
    struct Impl; // definition lives in PhysicsSystem.cpp — no Jolt types exposed here

    PhysicsSystem();
    ~PhysicsSystem();

    void OnStart(Scene& scene)            override;
    void OnUpdate(Scene& scene, float dt) override;
    void OnDestroy(Scene& scene)          override;

private:
    std::unique_ptr<Impl> m_impl;
    float                 m_accumulator = 0.0f;
};
