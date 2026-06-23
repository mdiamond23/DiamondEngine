#include "Animation/IKSystem.h"
#include "Animation/IKComponent.h"
#include "Animation/IKSolver.h"
#include "Animation/AnimationComponents.h"
#include "Animation/AnimationSampler.h"
#include "Scene/Scene.h"
#include "Scene/Physics/RagdollComponent.h"

#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <cmath>

namespace Diamond {

// Model-space orientation of a bone matrix, scale stripped.
static glm::quat OrientationOf(const glm::mat4& m)
{
    glm::vec3 x(m[0]), y(m[1]), z(m[2]);
    if (glm::dot(x, x) < 1e-12f || glm::dot(y, y) < 1e-12f || glm::dot(z, z) < 1e-12f)
        return glm::quat(1, 0, 0, 0);
    return glm::normalize(glm::quat_cast(
        glm::mat3(glm::normalize(x), glm::normalize(y), glm::normalize(z))));
}

// Shortest-arc slerp from `from` toward `to` by w.
static glm::quat BlendTo(glm::quat from, glm::quat to, float w)
{
    if (glm::dot(from, to) < 0.0f) to = -to;
    return glm::normalize(glm::slerp(from, to, w));
}

void UpdateIK(Scene& scene, float dt)
{
    auto& reg  = scene.GetRegistry();
    auto  view = reg.view<SkinnedMeshComponent, AnimatorComponent, IKComponent>();
    std::vector<glm::mat4> world;   // reused per entity

    for (auto entity : view) {
        auto& smc  = view.get<SkinnedMeshComponent>(entity);
        auto& anim = view.get<AnimatorComponent>(entity);
        auto& ik   = view.get<IKComponent>(entity);

        const Diamond::Skeleton& skel = smc.skeleton;
        const int n = (int)skel.bones.size();
        if (n == 0 || (int)anim.pose.size() != n) continue;   // pose not built this frame

        // A ragdoll in Limp/Powered owns the pose; don't fight it.
        if (auto* rag = reg.try_get<RagdollComponent>(entity); rag && rag->mode != RagdollMode::Animated)
            continue;

        const glm::mat4 modelFromWorld = glm::inverse(scene.GetTransformSystem().GetWorldMatrix(entity));

        // Model-space bone transforms for the current animated pose. Chains read joint
        // positions/orientations from this snapshot and write results back into pose.
        ComputeWorldTransforms(skel, anim.pose, world);

        // Exponential-smoothing factor for this frame (per chain easeTime is applied
        // below). easeTime <= 0 -> snap. Reused for both weight and target.
        const float dtClamped = dt > 0.0f ? dt : 0.0f;

        bool anySolved = false;
        for (IKChain& chain : ik.chains) {
            if (chain.solver != IKSolverType::TwoBone) continue;

            const float wTarget = glm::clamp(chain.weight, 0.0f, 1.0f);
            const float ease    = chain.easeTime > 1e-4f
                ? 1.0f - std::exp(-dtClamped / chain.easeTime)
                : 1.0f;

            // Ramp the smoothed weight toward the authored weight EVERY frame, so a
            // chain that was just disabled (weight -> 0) eases out instead of snapping.
            chain._curWeight += (wTarget - chain._curWeight) * ease;
            if (chain._curWeight <= 1e-3f && wTarget <= 0.0f) {
                chain._curWeight = 0.0f;
                chain._curTargetSet = false;   // re-seed (no swing) next time it re-enables
                continue;
            }

            // Resolve / re-validate the tip bone (cached; re-find if the name moved).
            if (chain._tipBone < 0 || chain._tipBone >= n ||
                skel.bones[chain._tipBone].name != chain.endEffectorBone)
                chain._tipBone = skel.Find(chain.endEffectorBone);
            const int tip = chain._tipBone;
            if (tip < 0) continue;

            // Two-bone chain by walking parents: tip <- mid <- root.
            const int mid = skel.bones[tip].parent;
            if (mid < 0) continue;
            const int root = skel.bones[mid].parent;
            if (root < 0) continue;

            const glm::vec3 a = glm::vec3(world[root][3]);   // shoulder / hip
            const glm::vec3 b = glm::vec3(world[mid][3]);    // elbow / knee
            const glm::vec3 c = glm::vec3(world[tip][3]);    // wrist / ankle (the tip)
            const glm::quat rootR = OrientationOf(world[root]);
            const glm::quat midR  = OrientationOf(world[mid]);

            // Target in MODEL space (entity transform divided out, matching the solver).
            glm::vec3 targetWorld = chain.targetOffset;
            if (chain.targetEntity != entt::null && reg.valid(chain.targetEntity))
                targetWorld += glm::vec3(scene.GetTransformSystem().GetWorldMatrix(chain.targetEntity)[3]);
            else
                targetWorld += chain.targetWorldPos;
            const glm::vec3 target = glm::vec3(modelFromWorld * glm::vec4(targetWorld, 1.0f));

            // Smooth the model-space target so a hard target jump (teleport / source
            // swap) reads as a quick ease rather than a pop. Seed on first use so the
            // hand doesn't swing in from the model origin.
            if (!chain._curTargetSet) {
                chain._curTarget    = target;
                chain._curTargetSet = true;
            } else {
                chain._curTarget += (target - chain._curTarget) * ease;
            }
            const glm::vec3 solveTarget = chain._curTarget;

            // Pole point: the current elbow keeps the animated bend plane; offset nudges it.
            const glm::vec3 pole = b + chain.poleOffset;

            glm::quat newRootR, newMidR;
            SolveTwoBone(a, b, c, rootR, midR, solveTarget, pole, newRootR, newMidR);

            // Solved MODEL-space rotations -> LOCAL (parent-relative), then blend toward
            // them by the weight. The mid bone's parent is the root, whose model rotation
            // we just solved, so reference newRootR (not the stale snapshot).
            const int rootParent = skel.bones[root].parent;
            const glm::quat rootParentR = rootParent >= 0 ? OrientationOf(world[rootParent])
                                                          : glm::quat(1, 0, 0, 0);
            const glm::quat rootLocal = glm::normalize(glm::conjugate(rootParentR) * newRootR);
            const glm::quat midLocal  = glm::normalize(glm::conjugate(newRootR)    * newMidR);

            const float bw = chain._curWeight;
            anim.pose[root].rotation = BlendTo(anim.pose[root].rotation, rootLocal, bw);
            anim.pose[mid].rotation  = BlendTo(anim.pose[mid].rotation,  midLocal,  bw);
            anySolved = true;
        }

        if (anySolved)
            ComputePalette(skel, anim.pose, anim.palette);   // recompose with the IK-adjusted pose
    }
}

} // namespace Diamond
