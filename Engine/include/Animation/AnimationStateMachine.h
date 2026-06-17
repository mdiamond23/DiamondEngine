#pragma once

#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace Diamond {

// A reusable animation state machine — the static graph definition loaded from an
// .animsm asset and shared across every entity that uses it. It is skeleton-
// agnostic: states reference clips by NAME, resolved per-entity at runtime against
// that entity's SkinnedMeshComponent::clips. Per-entity live state (current state,
// parameter values) lives in AnimStateMachineComponent, not here.

enum class AnimParamType { Float, Bool, Trigger };

// A named input that drives transitions. Float (e.g. "speed"), Bool (e.g.
// "isGrounded"), or Trigger — a bool that auto-resets the moment a transition
// consumes it, for one-shot events like "hit"/"jump".
struct AnimParameter {
    std::string   name;
    AnimParamType type         = AnimParamType::Float;
    float         defaultFloat = 0.0f;
    bool          defaultBool  = false;
};

// Comparison used by a transition condition. Float params use the ordered/equality
// ops against `value`; Bool uses IsTrue/IsFalse; Trigger uses Triggered.
enum class AnimCondOp {
    Greater, Less, GreaterEqual, LessEqual, Equal, NotEqual,
    IsTrue, IsFalse, Triggered
};

// One predicate against a parameter. A transition's conditions are ANDed together.
struct AnimCondition {
    std::string parameter;
    AnimCondOp  op    = AnimCondOp::Greater;
    float       value = 0.0f;   // threshold for Float ops; ignored for bool/trigger
};

// A node in the graph: plays one clip (by name) with the given speed/loop. nodePos
// is the editor canvas position, persisted so the graph layout survives reloads.
struct AnimState {
    std::string name;
    std::string clipName;
    float       speed = 1.0f;
    bool        loop  = true;
    glm::vec2   nodePos { 0.0f, 0.0f };
};

// A directed edge. When every condition passes, the runtime cross-fades from
// `fromState` to `toState` over `blendDuration` seconds. (AnyState as a source —
// fromState < 0 — and exit-time gating are reserved for a later pass.)
struct AnimTransition {
    int                        fromState     = -1;
    int                        toState       = -1;
    float                      blendDuration = 0.2f;
    std::vector<AnimCondition> conditions;   // ANDed
};

struct AnimStateMachine {
    std::string                 name;
    std::vector<AnimParameter>  parameters;
    std::vector<AnimState>      states;
    std::vector<AnimTransition> transitions;
    int                         entryState = 0;   // index into states

    int FindParameter(const std::string& n) const {
        for (int i = 0; i < (int)parameters.size(); ++i)
            if (parameters[i].name == n) return i;
        return -1;
    }
    int FindState(const std::string& n) const {
        for (int i = 0; i < (int)states.size(); ++i)
            if (states[i].name == n) return i;
        return -1;
    }
};

} // namespace Diamond
