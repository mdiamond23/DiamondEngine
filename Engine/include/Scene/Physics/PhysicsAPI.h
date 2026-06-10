#pragma once
#include <glm/glm.hpp>
#include "Rigidbody.h"

class Scene;

// Physics manipulation — valid only during play mode (between OnStart and OnDestroy).
// All functions are silent no-ops if the physics session is not active or the body
// handle is invalid.  Include this header in game scripts that need to push bodies around.
namespace Physics {

    // ---- Forces (accumulated each step, zeroed after application) ----
    void AddForce(const RigidBodyComponent& rb, glm::vec3 force);
    void AddForceAtPoint(const RigidBodyComponent& rb, glm::vec3 force, glm::vec3 worldPoint);
    void AddTorque(const RigidBodyComponent& rb, glm::vec3 torque);

    // ---- Impulses (instant velocity change) ----
    void AddImpulse(const RigidBodyComponent& rb, glm::vec3 impulse);
    void AddImpulseAtPoint(const RigidBodyComponent& rb, glm::vec3 impulse, glm::vec3 worldPoint);
    void AddAngularImpulse(const RigidBodyComponent& rb, glm::vec3 impulse);

    // ---- Velocity ----
    void      SetLinearVelocity(const RigidBodyComponent& rb, glm::vec3 velocity);
    glm::vec3 GetLinearVelocity(const RigidBodyComponent& rb);
    void      SetAngularVelocity(const RigidBodyComponent& rb, glm::vec3 angularVelocity);
    glm::vec3 GetAngularVelocity(const RigidBodyComponent& rb);

    // ---- Sleep state ----
    void Activate(const RigidBodyComponent& rb);    // wake a sleeping body
    void Deactivate(const RigidBodyComponent& rb);  // allow body to sleep

    // ---- World settings ----
    // Default: (0, -9.81, 0). No-op if physics session is not active.
    void SetGravity(glm::vec3 gravity);

    // ---- Debug visualization ----
    // Submits wireframe collider shapes for all entities in the scene to DebugDraw.
    // Call Physics::DrawColliders(scene) then DebugDraw::Flush(vp) each frame.
    // Works in both edit and play mode — does not require an active physics session.
    void DrawColliders(Scene& scene, glm::vec3 color = {0.0f, 1.0f, 0.0f});
}
