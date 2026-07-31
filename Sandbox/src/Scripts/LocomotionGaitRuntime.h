#pragma once

#include <glm/glm.hpp>

namespace Diamond::Locomotion
{
// The physical gait consumes intent rather than reading keys directly, so player,
// AI, or replay controllers can share the same locomotion implementation.
struct GaitCommand
{
    bool enabled = false;
    bool startRequested = false;
    bool stopRequested = false;

    // -1 starts on left support, +1 on right support, and 0 does not start.
    int initialSupportSide = 0;

    // Zero preserves the controller's settled forward basis. Runtime control will supply
    // a horizontal world-space direction when it starts an input-driven gait.
    glm::vec3 desiredForward { 0.0f };
    float desiredSpeed = 0.0f;

};
}
