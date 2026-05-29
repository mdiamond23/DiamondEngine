#pragma once
#include <glm/glm.hpp>
#include <cfloat>

namespace Diamond {

// Axis-aligned bounding box in an object's local space.
struct AABB {
    glm::vec3 min{ FLT_MAX,  FLT_MAX,  FLT_MAX};
    glm::vec3 max{-FLT_MAX, -FLT_MAX, -FLT_MAX};

    // Fast Möller AABB transform: maps local AABB to world space in O(1)
    // without iterating corners. Works for any affine matrix (scale, rotate, translate).
    AABB Transform(const glm::mat4& m) const {
        glm::vec3 center = (min + max) * 0.5f;
        glm::vec3 h      = (max - min) * 0.5f;
        glm::vec3 tc     = glm::vec3(m * glm::vec4(center, 1.0f));
        // GLM is column-major: m[col][row], so m[c][r] = matrix element at row r, col c
        glm::vec3 th = {
            glm::abs(m[0][0]) * h.x + glm::abs(m[1][0]) * h.y + glm::abs(m[2][0]) * h.z,
            glm::abs(m[0][1]) * h.x + glm::abs(m[1][1]) * h.y + glm::abs(m[2][1]) * h.z,
            glm::abs(m[0][2]) * h.x + glm::abs(m[1][2]) * h.y + glm::abs(m[2][2]) * h.z,
        };
        return { tc - th, tc + th };
    }
};

// 6-plane view frustum. Extract once per frame from (proj * view), then test draw calls.
struct Frustum {
    glm::vec4 planes[6]; // left, right, bottom, top, near, far — plane eq: ax+by+cz+d=0

    // Gribb-Hartmann extraction from a combined VP matrix.
    // OpenGL NDC (-1..1 depth): near = row3 + row2.
    // Vulkan NDC (0..1 depth):  near = row2 only — change planes[4] accordingly.
    static Frustum Extract(const glm::mat4& vp) {
        // Build row vectors from column-major storage
        auto row = [&](int r) {
            return glm::vec4(vp[0][r], vp[1][r], vp[2][r], vp[3][r]);
        };
        glm::vec4 r0 = row(0), r1 = row(1), r2 = row(2), r3 = row(3);
        Frustum f;
        f.planes[0] = r3 + r0; // left
        f.planes[1] = r3 - r0; // right
        f.planes[2] = r3 + r1; // bottom
        f.planes[3] = r3 - r1; // top
        f.planes[4] = r3 + r2; // near
        f.planes[5] = r3 - r2; // far
        for (auto& p : f.planes)
            p /= glm::length(glm::vec3(p)); // normalize so distance is in world-space units
        return f;
    }

    // Returns true if the AABB is fully or partially inside the frustum.
    // Uses the p-vertex test: if the most-positive corner of the box is behind any
    // plane, the entire box is outside and can be culled.
    bool TestAABB(const AABB& box) const {
        for (const auto& plane : planes) {
            glm::vec3 pv = {
                plane.x >= 0.0f ? box.max.x : box.min.x,
                plane.y >= 0.0f ? box.max.y : box.min.y,
                plane.z >= 0.0f ? box.max.z : box.min.z,
            };
            if (glm::dot(glm::vec3(plane), pv) + plane.w < 0.0f)
                return false;
        }
        return true;
    }
};

} // namespace Diamond
