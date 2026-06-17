#include "Animation/AnimationSystem.h"
#include "Animation/AnimationComponents.h"
#include "Animation/AnimationSampler.h"

#include <algorithm>
#include <cmath>

namespace Diamond {

// Advances a clip time by `delta`, wrapping (loop) or clamping (one-shot) to the
// clip duration. Handles negative speed so reversed playback wraps correctly.
static float AdvanceTime(float t, float delta, float duration, bool loop)
{
    t += delta;
    if (duration <= 0.0f) return t;
    if (loop) {
        t = std::fmod(t, duration);
        if (t < 0.0f) t += duration;
    } else {
        t = std::clamp(t, 0.0f, duration);
    }
    return t;
}

void UpdateAnimators(entt::registry& reg, float dt, bool advance)
{
    auto view = reg.view<SkinnedMeshComponent, AnimatorComponent>();
    for (auto entity : view) {
        auto& smc  = view.get<SkinnedMeshComponent>(entity);
        auto& anim = view.get<AnimatorComponent>(entity);
        if (smc.skeleton.bones.empty()) continue;

        auto clipValid = [&](int c) { return c >= 0 && c < (int)smc.clips.size(); };
        auto duration  = [&](int c) { return clipValid(c) ? smc.clips[c].duration : 0.0f; };

        if (advance) {
            if (anim.playing) {
                anim.time = AdvanceTime(anim.time, dt * anim.speed, duration(anim.clip), anim.loop);
                if (anim.prevClip >= 0) {
                    // Keep the outgoing clip animating during the fade so locomotion
                    // cross-fades read naturally rather than freezing mid-stride.
                    anim.prevTime    = AdvanceTime(anim.prevTime, dt * anim.speed, duration(anim.prevClip), anim.loop);
                    anim.fadeElapsed += dt;
                    if (anim.fadeDuration <= 0.0f || anim.fadeElapsed >= anim.fadeDuration)
                        anim.prevClip = -1;   // fade complete
                }
            }
        } else {
            // Editor / paused: don't progress a fade — snap to the target clip so a
            // clip change in the inspector previews instantly instead of stalling.
            anim.prevClip = -1;
        }

        // Sample the current (target) pose.
        Pose target;
        if (clipValid(anim.clip)) SampleLocalPose(smc.skeleton, smc.clips[anim.clip], anim.time, target);
        else                      BindPose(smc.skeleton, target);

        // Blend with the outgoing pose while a fade is in progress.
        if (anim.prevClip >= 0 && anim.fadeDuration > 0.0f) {
            Pose from;
            if (clipValid(anim.prevClip)) SampleLocalPose(smc.skeleton, smc.clips[anim.prevClip], anim.prevTime, from);
            else                          BindPose(smc.skeleton, from);

            float w = std::clamp(anim.fadeElapsed / anim.fadeDuration, 0.0f, 1.0f);
            Pose blended;
            BlendPoses(from, target, w, blended);
            ComputePalette(smc.skeleton, blended, anim.palette);
        } else {
            ComputePalette(smc.skeleton, target, anim.palette);
        }
    }
}

// --- state machine ---------------------------------------------------------

static bool EvalCondition(const AnimStateMachineComponent& sm, const AnimCondition& c)
{
    switch (c.op) {
        case AnimCondOp::Greater:      return sm.GetFloat(c.parameter) >  c.value;
        case AnimCondOp::Less:         return sm.GetFloat(c.parameter) <  c.value;
        case AnimCondOp::GreaterEqual: return sm.GetFloat(c.parameter) >= c.value;
        case AnimCondOp::LessEqual:    return sm.GetFloat(c.parameter) <= c.value;
        case AnimCondOp::Equal:        return sm.GetFloat(c.parameter) == c.value;
        case AnimCondOp::NotEqual:     return sm.GetFloat(c.parameter) != c.value;
        case AnimCondOp::IsTrue:       return sm.GetBool(c.parameter);
        case AnimCondOp::IsFalse:      return !sm.GetBool(c.parameter);
        case AnimCondOp::Triggered:    return sm.triggers.count(c.parameter) > 0;
    }
    return false;
}

void UpdateStateMachines(entt::registry& reg, float dt, bool advance)
{
    (void)dt;
    auto view = reg.view<AnimStateMachineComponent, AnimatorComponent, SkinnedMeshComponent>();
    for (auto entity : view) {
        auto& sm   = view.get<AnimStateMachineComponent>(entity);
        auto& anim = view.get<AnimatorComponent>(entity);
        auto& smc  = view.get<SkinnedMeshComponent>(entity);
        if (!sm.machine || sm.machine->states.empty()) continue;
        const AnimStateMachine& M = *sm.machine;

        auto clipIndex = [&](const std::string& name) -> int {
            for (int i = 0; i < (int)smc.clips.size(); ++i)
                if (smc.clips[i].name == name) return i;
            return -1;
        };

        // Enter the entry state on first tick (or if state went out of range, e.g.
        // after a scene reload reset currentState to -1).
        if (sm.currentState < 0 || sm.currentState >= (int)M.states.size()) {
            int entry = (M.entryState >= 0 && M.entryState < (int)M.states.size()) ? M.entryState : 0;
            const AnimState& st = M.states[entry];
            anim.clip     = clipIndex(st.clipName);
            anim.time     = 0.0f;
            anim.prevClip = -1;
            anim.loop     = st.loop;
            anim.speed    = st.speed;
            sm.currentState = entry;
        }

        if (!advance) continue;   // editor / paused: hold the current state

        // First transition out of the current state whose conditions all pass wins
        // (listed order = priority).
        for (const AnimTransition& t : M.transitions) {
            if (t.fromState != sm.currentState) continue;   // AnyState (fromState<0) deferred
            if (t.toState < 0 || t.toState >= (int)M.states.size()) continue;

            bool allPass = true;
            for (const AnimCondition& c : t.conditions)
                if (!EvalCondition(sm, c)) { allPass = false; break; }
            if (!allPass) continue;

            // Consume the triggers this transition tested.
            for (const AnimCondition& c : t.conditions)
                if (c.op == AnimCondOp::Triggered) sm.triggers.erase(c.parameter);

            const AnimState& dst = M.states[t.toState];
            anim.CrossFade(clipIndex(dst.clipName), t.blendDuration);
            anim.loop  = dst.loop;
            anim.speed = dst.speed;
            sm.currentState = t.toState;
            break;
        }
    }
}

} // namespace Diamond
