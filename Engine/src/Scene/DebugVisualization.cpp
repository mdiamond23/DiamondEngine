#include "Scene/DebugVisualization.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"
#include "Animation/AnimationComponents.h"
#include "Animation/AnimationSampler.h"
#include "Animation/IKComponent.h"
#include "DebugDraw.h"

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include <algorithm>
#include <cmath>

namespace Diamond {

namespace {

// Draws a wireframe cone outline: edge lines from the apex plus a ring at the far
// end, opening along `dir` (normalized) with the given half-angle. Used to picture
// an audio source's directional cone.
void DrawConeOutline(glm::vec3 apex, glm::vec3 dir, float halfAngleRad,
                     float length, glm::vec3 color)
{
    // Orthonormal basis around the cone axis.
    glm::vec3 up = std::abs(dir.y) > 0.99f ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
    glm::vec3 right = glm::normalize(glm::cross(dir, up));
    up = glm::normalize(glm::cross(right, dir));

    const float ringRadius = std::tan(halfAngleRad) * length;
    const glm::vec3 ringCenter = apex + dir * length;

    constexpr int kSeg = 24;
    glm::vec3 prev{};
    for (int i = 0; i <= kSeg; ++i) {
        float a = (float)i / kSeg * glm::two_pi<float>();
        glm::vec3 p = ringCenter + (right * std::cos(a) + up * std::sin(a)) * ringRadius;
        if (i > 0) DebugDraw::Line(prev, p, color);
        // Four edge spokes from the apex.
        if (i % (kSeg / 4) == 0) DebugDraw::Line(apex, p, color);
        prev = p;
    }
}

} // namespace

void DrawIKDebug(Scene& scene, entt::entity sel, int activeChain)
{
    auto& reg = scene.GetRegistry();
    if (sel == entt::null || !reg.valid(sel)) return;
    if (!reg.all_of<SkinnedMeshComponent, AnimatorComponent, IKComponent>(sel)) return;

    auto& smc  = reg.get<SkinnedMeshComponent>(sel);
    auto& anim = reg.get<AnimatorComponent>(sel);
    auto& ik   = reg.get<IKComponent>(sel);

    const Skeleton& skel = smc.skeleton;
    const int n = (int)skel.bones.size();
    if (n == 0 || (int)anim.pose.size() != n) return;

    const glm::mat4 worldMat = scene.GetTransformSystem().GetWorldMatrix(sel);
    std::vector<glm::mat4> world;
    ComputeWorldTransforms(skel, anim.pose, world);

    auto boneWorld = [&](int i) { return glm::vec3(worldMat * world[i] * glm::vec4(0, 0, 0, 1)); };

    for (int ci = 0; ci < (int)ik.chains.size(); ++ci) {
        const IKChain& chain = ik.chains[ci];
        const bool active = (ci == activeChain);

        int tip = skel.Find(chain.endEffectorBone);
        if (tip < 0) continue;
        const int mid  = skel.bones[tip].parent;
        if (mid < 0) continue;
        const int root = skel.bones[mid].parent;
        if (root < 0) continue;

        const glm::vec3 cBone = boneWorld(tip);
        const glm::vec3 cMid  = boneWorld(mid);
        const glm::vec3 cRoot = boneWorld(root);

        // Bone segments: bright orange for the active chain, dim steel for the rest.
        const glm::vec3 boneCol = active ? glm::vec3(1.0f, 0.55f, 0.1f)
                                         : glm::vec3(0.35f, 0.45f, 0.6f);
        DebugDraw::Line(cRoot, cMid, boneCol);
        DebugDraw::Line(cMid,  cBone, boneCol);
        DebugDraw::Sphere(cRoot, 0.025f, boneCol);
        DebugDraw::Sphere(cMid,  0.025f, boneCol);
        DebugDraw::Sphere(cBone, 0.03f,  boneCol);

        // Resolved world target (mirrors UpdateIK's resolution order).
        glm::vec3 targetWorld = chain.targetOffset;
        if (!chain.isFootChain && chain.targetEntity != entt::null && reg.valid(chain.targetEntity))
            targetWorld += glm::vec3(scene.GetTransformSystem().GetWorldMatrix(chain.targetEntity)[3]);
        else
            targetWorld += chain.targetWorldPos;

        const glm::vec3 tgtCol = active ? glm::vec3(0.2f, 1.0f, 0.3f)
                                        : glm::vec3(0.2f, 0.55f, 0.3f);
        DebugDraw::Sphere(targetWorld, 0.04f, tgtCol);
        DebugDraw::Line(cBone, targetWorld, tgtCol);

        // Pole hint: model-space (mid + poleOffset) into world. Only worth showing
        // when the user has nudged it off the animated bend.
        if (active && glm::dot(chain.poleOffset, chain.poleOffset) > 1e-8f) {
            const glm::vec3 poleModel = glm::vec3(world[mid] * glm::vec4(0, 0, 0, 1)) + chain.poleOffset;
            const glm::vec3 poleWorld = glm::vec3(worldMat * glm::vec4(poleModel, 1.0f));
            const glm::vec3 poleCol(0.85f, 0.2f, 0.9f);
            DebugDraw::Sphere(poleWorld, 0.03f, poleCol);
            DebugDraw::Line(cMid, poleWorld, poleCol);
        }
    }
}

void DrawAudioDebug(Scene& scene, entt::entity sel)
{
    auto& reg = scene.GetRegistry();
    auto& ts  = scene.GetTransformSystem();

    for (auto [e, src] : reg.view<AudioSourceComponent>().each()) {
        if (!src.is3D) continue;
        const glm::mat4 w = ts.GetWorldMatrix(e);
        const glm::vec3 pos = glm::vec3(w[3]);
        const bool selected = (e == sel);

        // Min distance (full volume) and max distance (falloff end).
        const glm::vec3 minCol = selected ? glm::vec3(0.25f, 1.0f, 0.4f) : glm::vec3(0.15f, 0.6f, 0.25f);
        const glm::vec3 maxCol = selected ? glm::vec3(0.3f, 0.6f, 1.0f)  : glm::vec3(0.18f, 0.35f, 0.6f);
        DebugDraw::Sphere(pos, src.minDistance, minCol);
        DebugDraw::Sphere(pos, src.maxDistance, maxCol);

        // Directional cone (only when it actually narrows the field).
        if (src.coneOuterAngle < 359.9f) {
            const glm::vec3 fwd = glm::normalize(-glm::vec3(w[2]));
            const float len = std::min(src.maxDistance, src.minDistance * 4.0f + 1.0f);
            const glm::vec3 coneCol = selected ? glm::vec3(1.0f, 0.8f, 0.2f) : glm::vec3(0.6f, 0.5f, 0.15f);
            DrawConeOutline(pos, fwd, glm::radians(src.coneOuterAngle * 0.5f), len, coneCol);
            if (src.coneInnerAngle < src.coneOuterAngle - 0.1f)
                DrawConeOutline(pos, fwd, glm::radians(src.coneInnerAngle * 0.5f), len,
                                coneCol * 0.7f + glm::vec3(0.0f, 0.0f, 0.1f));
        }
    }

    // Listeners: a small marker and a forward line (-Z), like the AudioSystem basis.
    for (auto [e, l] : reg.view<AudioListenerComponent>().each()) {
        if (!l.enabled) continue;
        const glm::mat4 w = ts.GetWorldMatrix(e);
        const glm::vec3 pos = glm::vec3(w[3]);
        const glm::vec3 fwd = glm::normalize(-glm::vec3(w[2]));
        const glm::vec3 col(1.0f, 0.55f, 0.15f);
        DebugDraw::Sphere(pos, 0.15f, col);
        DebugDraw::Line(pos, pos + fwd * 0.6f, col);
    }
}

} // namespace Diamond
