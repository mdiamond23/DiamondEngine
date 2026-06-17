#include "Animation/AnimationSystem.h"
#include "Animation/AnimationComponents.h"
#include "Animation/AnimationSampler.h"

#include <algorithm>
#include <cmath>

namespace Diamond {

void UpdateAnimators(entt::registry& reg, float dt)
{
    auto view = reg.view<SkinnedMeshComponent, AnimatorComponent>();
    for (auto entity : view) {
        auto& smc  = view.get<SkinnedMeshComponent>(entity);
        auto& anim = view.get<AnimatorComponent>(entity);
        if (smc.skeleton.bones.empty()) continue;

        const bool haveClip = anim.clip >= 0 && anim.clip < (int)smc.clips.size();
        if (!haveClip) {
            ComputeBindPose(smc.skeleton, anim.palette);
            continue;
        }

        const AnimationClip& clip = smc.clips[anim.clip];
        if (anim.playing) {
            anim.time += dt * anim.speed;
            if (clip.duration > 0.0f) {
                anim.time = anim.loop ? std::fmod(anim.time, clip.duration)
                                      : std::min(anim.time, clip.duration);
            }
        }
        SamplePose(smc.skeleton, clip, anim.time, anim.palette);
    }
}

} // namespace Diamond
