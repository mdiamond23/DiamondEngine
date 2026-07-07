#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

// Immediate-mode debug line renderer.
// Accumulate geometry with Line/Box/Sphere/Capsule, then call Flush once per
// frame (after the main scene pass, before ImGui) to render and clear the buffer.
namespace DebugDraw {
    struct Vertex { glm::vec3 pos, color; };

    void Line(glm::vec3 a, glm::vec3 b, glm::vec3 color = {0.0f, 1.0f, 0.0f});
    void Box(glm::vec3 center, glm::quat rotation, glm::vec3 halfExtents,
             glm::vec3 color = {0.0f, 1.0f, 0.0f});
    void Sphere(glm::vec3 center, float radius,
                glm::vec3 color = {0.0f, 1.0f, 0.0f});
    void Capsule(glm::vec3 center, glm::quat rotation, float halfHeight, float radius,
                 glm::vec3 color = {0.0f, 1.0f, 0.0f});

    // GL: upload and draw all accumulated lines, then clear the buffer.
    void Flush(const glm::mat4& viewProjection);

    // Non-GL backends: hand back this frame's accumulated vertices and clear the
    // buffer, mirroring what Flush does internally for GL.
    std::vector<Vertex> TakeVertices();
}
