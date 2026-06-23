#include "Animation/IKSystem.h"
#include "Animation/IKComponent.h"
#include "Animation/IKSolver.h"
#include "Animation/AnimationComponents.h"
#include "Animation/AnimationSampler.h"
#include "Scene/Scene.h"
#include "Scene/Physics/RagdollComponent.h"
#include "Scene/Physics/PhysicsAPI.h"

#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

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
    std::vector<glm::mat4> world;
    std::vector<float>     targetWeights;   // per-chain target weight; -1 = use chain.weight

    for (auto entity : view) {
        auto& smc  = view.get<SkinnedMeshComponent>(entity);
        auto& anim = view.get<AnimatorComponent>(entity);
        auto& ik   = view.get<IKComponent>(entity);

        const Diamond::Skeleton& skel = smc.skeleton;
        const int n = (int)skel.bones.size();
        if (n == 0 || (int)anim.pose.size() != n) continue;

        // A ragdoll in Limp/Powered owns the pose; don't fight it.
        if (auto* rag = reg.try_get<RagdollComponent>(entity); rag && rag->mode != RagdollMode::Animated)
            continue;

        const glm::mat4 worldMat       = scene.GetTransformSystem().GetWorldMatrix(entity);
        const glm::mat4 modelFromWorld = glm::inverse(worldMat);
        const float     dtClamped      = dt > 0.0f ? dt : 0.0f;
        const glm::vec3 kWorldUp       { 0.0f, 1.0f, 0.0f };
        // World up expressed in the skeleton's model space. The skeleton may be
        // authored Z-up (e.g. glTF with a Z_UP node) and rotated to world Y-up by
        // the entity transform, so model "up" is not necessarily +Y. Deriving it
        // from the transform keeps swing detection, pelvis drop, and foot tilt
        // correct for any rig orientation.
        const glm::vec3 modelUp = glm::normalize(
            glm::vec3(modelFromWorld * glm::vec4(kWorldUp, 0.0f)));

        // Animated-pose snapshot used by Phase 1 raycasts and Phase 2 IK reads.
        ComputeWorldTransforms(skel, anim.pose, world);

        // ── Phase 1: foot raycasts + pelvis correction ──────────────────────
        //
        // For each foot chain: cast a ray straight down from above the animated
        // ankle to find the ground. Compute how far each foot needs to move down,
        // then lower the pelvis by the maximum so legs don't hyperextend. A
        // re-snapshot after the pelvis shift feeds the correct chain-root positions
        // into Phase 2's IK solve.

        const int numChains = (int)ik.chains.size();
        targetWeights.assign(numChains, -1.0f);   // -1 sentinel = "use chain.weight"

        // Resolve pelvis bone index (re-validates name each frame like _tipBone).
        if (!ik.pelvisBone.empty()) {
            if (ik._pelvisBoneIdx < 0 || ik._pelvisBoneIdx >= n ||
                skel.bones[ik._pelvisBoneIdx].name != ik.pelvisBone)
                ik._pelvisBoneIdx = skel.Find(ik.pelvisBone);
        }

        bool  anyFootChain   = false;
        float maxDrop        = 0.0f;
        bool  pelvisModified = false;

        for (int ci = 0; ci < numChains; ++ci) {
            IKChain& chain = ik.chains[ci];
            if (!chain.isFootChain) continue;

            // Resolve tip bone.
            if (chain._tipBone < 0 || chain._tipBone >= n ||
                skel.bones[chain._tipBone].name != chain.endEffectorBone)
                chain._tipBone = skel.Find(chain.endEffectorBone);
            const int tip = chain._tipBone;
            if (tip < 0) continue;

            // Cache the bind-pose ankle model-space position once (swing detection).
            if (!chain._bindResolved) {
                chain._bindFootPos  = glm::vec3(glm::inverse(skel.bones[tip].inverseBind)[3]);
                chain._bindResolved = true;
            }

            anyFootChain = true;

            const float ease = chain.easeTime > 1e-4f
                ? 1.0f - std::exp(-dtClamped / chain.easeTime)
                : 1.0f;

            // Animated ankle in world space.
            const glm::vec3 animAnkleModel = glm::vec3(world[tip][3]);
            const glm::vec3 animAnkleWorld = glm::vec3(worldMat * glm::vec4(animAnkleModel, 1.0f));

            // Swing phase: if the animated ankle is above bind-pose height + threshold
            // the foot is in the air — release IK so the animation plays unmodified.
            // Heights are measured along modelUp (not raw .y) for rig-orientation safety.
            const float ankleUp    = glm::dot(animAnkleModel,      modelUp);
            const float bindUp     = glm::dot(chain._bindFootPos,  modelUp);
            const bool  inSwing    = ankleUp > bindUp + chain.swingThreshold;
            const float swingBlend = inSwing ? 0.0f : 1.0f;

            // Downward ray from castOffset above the animated ankle.
            const glm::vec3 rayOrigin = animAnkleWorld + kWorldUp * chain.castOffset;
            const float     rayLen    = chain.castOffset + chain.maxStepHeight;
            const HitResult hit       = Physics::Raycast(rayOrigin, -kWorldUp, rayLen, entity);

            if (hit.hit) {
                // Target = hit point raised by ankle height (ankle center isn't at the sole).
                chain.targetWorldPos = hit.point + kWorldUp * chain.ankleHeight;
                // Smooth the surface normal to avoid frame-to-frame jitter on rough terrain.
                chain._groundNormal = glm::normalize(
                    chain._groundNormal + (hit.normal - chain._groundNormal) * ease);

                // Positive drop = target is below the animated ankle (leg needs to reach down).
                const float drop = animAnkleWorld.y - chain.targetWorldPos.y;
                chain._rawDrop   = drop;
                maxDrop          = std::max(maxDrop, drop);
                targetWeights[ci] = chain.weight * swingBlend;
            } else {
                // No ground hit: keep the foot at the animated position and release IK.
                chain.targetWorldPos = animAnkleWorld;
                chain._groundNormal  = kWorldUp;
                chain._rawDrop       = 0.0f;
                targetWeights[ci]    = 0.0f;
            }
        }

        // Pelvis correction: lower the pelvis by the max drop so the legs can reach
        // without fully extending, then re-snapshot the world transforms.
        if (anyFootChain && ik._pelvisBoneIdx >= 0 && ik._pelvisBoneIdx < n) {
            const float clampedDrop = glm::clamp(maxDrop, 0.0f, ik.maxPelvisDrop);
            const float pelvisEase  = ik.pelvisEaseTime > 1e-4f
                ? 1.0f - std::exp(-dtClamped / ik.pelvisEaseTime)
                : 1.0f;
            ik._curPelvisDrop += (clampedDrop - ik._curPelvisDrop) * pelvisEase;

            if (ik._curPelvisDrop > 1e-4f) {
                // Move the pelvis straight down in WORLD space by the drop amount,
                // expressed in model space (handles rig orientation and scale).
                anim.pose[ik._pelvisBoneIdx].translation -=
                    glm::vec3(modelFromWorld * glm::vec4(kWorldUp * ik._curPelvisDrop, 0.0f));
                ComputeWorldTransforms(skel, anim.pose, world);
                pelvisModified = true;
            }
        }

        // ── Phase 2: IK solve for all chains ────────────────────────────────
        //
        // Foot chains: targetWorldPos was set by Phase 1 raycasts; weight is the
        // swing-adjusted value from targetWeights[]. Non-foot chains: use the
        // authored targetEntity/targetWorldPos and weight unchanged.

        bool anySolved = false;
        for (int ci = 0; ci < numChains; ++ci) {
            IKChain& chain = ik.chains[ci];
            if (chain.solver != IKSolverType::TwoBone) continue;

            // -1 sentinel means non-foot chain; fall back to authored weight.
            const float wTarget = glm::clamp(
                targetWeights[ci] < 0.0f ? chain.weight : targetWeights[ci],
                0.0f, 1.0f);
            const float ease = chain.easeTime > 1e-4f
                ? 1.0f - std::exp(-dtClamped / chain.easeTime)
                : 1.0f;

            chain._curWeight += (wTarget - chain._curWeight) * ease;
            if (chain._curWeight <= 1e-3f && wTarget <= 0.0f) {
                chain._curWeight    = 0.0f;
                chain._curTargetSet = false;
                continue;
            }

            // Resolve tip bone (foot chains already done in Phase 1).
            if (chain._tipBone < 0 || chain._tipBone >= n ||
                skel.bones[chain._tipBone].name != chain.endEffectorBone)
                chain._tipBone = skel.Find(chain.endEffectorBone);
            const int tip = chain._tipBone;
            if (tip < 0) continue;

            const int mid  = skel.bones[tip].parent;
            if (mid < 0) continue;
            const int root = skel.bones[mid].parent;
            if (root < 0) continue;

            const glm::vec3 a     = glm::vec3(world[root][3]);
            const glm::vec3 b     = glm::vec3(world[mid][3]);
            const glm::vec3 c     = glm::vec3(world[tip][3]);
            const glm::quat rootR = OrientationOf(world[root]);
            const glm::quat midR  = OrientationOf(world[mid]);

            // World-space target. Foot chains have targetWorldPos pre-filled by Phase 1;
            // non-foot chains resolve entity-follow or use the authored world point.
            glm::vec3 targetWorld = chain.targetOffset;
            if (!chain.isFootChain && chain.targetEntity != entt::null && reg.valid(chain.targetEntity))
                targetWorld += glm::vec3(scene.GetTransformSystem().GetWorldMatrix(chain.targetEntity)[3]);
            else
                targetWorld += chain.targetWorldPos;
            const glm::vec3 target = glm::vec3(modelFromWorld * glm::vec4(targetWorld, 1.0f));

            if (!chain._curTargetSet) {
                chain._curTarget    = target;
                chain._curTargetSet = true;
            } else {
                chain._curTarget += (target - chain._curTarget) * ease;
            }

            const glm::vec3 pole = b + chain.poleOffset;
            glm::quat newRootR, newMidR;
            SolveTwoBone(a, b, c, rootR, midR, chain._curTarget, pole, newRootR, newMidR);

            // Solved MODEL-space rotations -> LOCAL (parent-relative), then blend.
            const int       rootParent  = skel.bones[root].parent;
            const glm::quat rootParentR = rootParent >= 0 ? OrientationOf(world[rootParent])
                                                          : glm::quat(1, 0, 0, 0);
            const glm::quat rootLocal = glm::normalize(glm::conjugate(rootParentR) * newRootR);
            const glm::quat midLocal  = glm::normalize(glm::conjugate(newRootR)    * newMidR);

            anim.pose[root].rotation = BlendTo(anim.pose[root].rotation, rootLocal, chain._curWeight);
            anim.pose[mid].rotation  = BlendTo(anim.pose[mid].rotation,  midLocal,  chain._curWeight);
            anySolved = true;
        }

        // ── Phase 3: foot tilt ───────────────────────────────────────────────
        //
        // After the leg IK has set the ankle position, rotate the tip (foot) bone
        // so the sole aligns with the surface normal. The rotation applied is the
        // delta from flat ground (model Y-up) to the surface normal — this is
        // independent of the foot bone's local axis orientation in the rig.

        bool worldRefreshed = false;
        for (int ci = 0; ci < numChains; ++ci) {
            IKChain& chain = ik.chains[ci];
            if (!chain.isFootChain || !chain.tiltToNormal || chain._curWeight < 1e-3f) continue;

            const int tip = chain._tipBone;
            if (tip < 0) continue;
            const int tipParent = skel.bones[tip].parent;
            if (tipParent < 0) continue;

            // Refresh world transforms once (after Phase 2 wrote new root/mid rotations).
            if (!worldRefreshed) {
                ComputeWorldTransforms(skel, anim.pose, world);
                worldRefreshed = true;
            }

            // Rotation from flat ground (modelUp) to the surface normal in model space.
            const glm::vec3 normalModel = glm::normalize(
                glm::vec3(modelFromWorld * glm::vec4(chain._groundNormal, 0.0f)));
            const glm::vec3 axis = glm::cross(modelUp, normalModel);
            const float     sinA = glm::length(axis);
            const float     cosA = glm::dot(modelUp, normalModel);
            if (sinA < 1e-6f) continue;   // flat ground, no tilt needed

            const glm::quat tiltDelta    = glm::angleAxis(std::atan2(sinA, cosA), glm::normalize(axis));
            const glm::quat tipModelR    = OrientationOf(world[tip]);
            const glm::quat newTipModelR = tiltDelta * tipModelR;

            const glm::quat parentModelR = OrientationOf(world[tipParent]);
            const glm::quat newTipLocalR = glm::normalize(glm::conjugate(parentModelR) * newTipModelR);

            const float bw = chain.tiltWeight * chain._curWeight;
            anim.pose[tip].rotation = BlendTo(anim.pose[tip].rotation, newTipLocalR, bw);
            anySolved = true;
        }

        if (anySolved || pelvisModified)
            ComputePalette(skel, anim.pose, anim.palette);
    }
}

} // namespace Diamond
