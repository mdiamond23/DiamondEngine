#pragma once

#include <glm/glm.hpp>

namespace Diamond::Locomotion
{
// The gait consumes intent, not keys. Validation, player input, AI, and replay code can
// all produce the same command without changing the physical controller.
enum class RunLimit
{
    None,
    StepCount,
    Duration
};

struct GaitCommand
{
    bool enabled = false;
    bool startRequested = false;
    bool stopRequested = false;
    bool resetRequested = false;

    // -1 starts on left support, +1 on right support, and 0 does not start.
    int initialSupportSide = 0;

    // Zero preserves the controller's settled forward basis. Runtime control will supply
    // a horizontal world-space direction when it starts an input-driven gait.
    glm::vec3 desiredForward { 0.0f };
    float desiredSpeed = 0.0f;

    RunLimit runLimit = RunLimit::None;
    int stepLimit = 0;
    float durationLimit = 0.0f;
};

inline bool HasReachedRunLimit(const GaitCommand& command,
                               int completedSteps, float elapsedSeconds)
{
    switch (command.runLimit) {
        case RunLimit::StepCount:
            return completedSteps >= glm::max(command.stepLimit, 2);
        case RunLimit::Duration:
            return elapsedSeconds >= glm::max(command.durationLimit, 10.0f);
        case RunLimit::None:
            return false;
    }
    return false;
}

inline bool RunLimitSatisfied(const GaitCommand& command,
                              int completedSteps, float elapsedSeconds)
{
    return command.runLimit == RunLimit::None
        || HasReachedRunLimit(command, completedSteps, elapsedSeconds);
}

inline const char* RunLimitName(RunLimit limit)
{
    switch (limit) {
        case RunLimit::StepCount: return "TEN_STEP";
        case RunLimit::Duration: return "ENDURANCE";
        case RunLimit::None: return "UNBOUNDED";
    }
    return "UNBOUNDED";
}
}
