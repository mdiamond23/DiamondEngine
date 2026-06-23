#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <vector>
#include <entt/entt.hpp>

// Procedural inverse-kinematics on a skinned character: a list of chains, each
// pulling one end-effector bone (a hand or a foot) toward a world-space target and
// blended into the animated pose by a weight. Kept in the global namespace to match
// the other ECS components (SkinnedMeshComponent, AnimatorComponent). Solved by
// UpdateIK after UpdateAnimators each frame. See Docs/ik-design.md.

enum class IKSolverType { TwoBone };   // FABRIK / CCD later, for longer chains

// One IK chain. The chain is the end-effector bone plus its two ancestors, found by
// walking Skeleton::Bone::parent (a 2-bone chain: e.g. hand <- forearm <- upper arm),
// so only the bone name and the moving-link count are authored, not the whole chain.
struct IKChain {
    std::string  endEffectorBone;               // tip whose ORIGIN is driven to the target (e.g. the hand/wrist bone)
    int          boneCount = 2;                  // moving links above the tip (2 = forearm + upper arm)
    IKSolverType solver    = IKSolverType::TwoBone;

    // --- target (one source) ---
    entt::entity targetEntity   = entt::null;    // follow this entity's world position...
    glm::vec3    targetWorldPos  { 0.0f };        // ...or an explicit world point (when targetEntity == null)
    glm::vec3    targetOffset    { 0.0f };        // extra offset in world space

    // --- shaping ---
    glm::vec3    poleOffset { 0.0f };             // model-space nudge to the elbow/knee hint; 0 keeps the animated bend plane
    float        weight     = 1.0f;               // 0..1 blend of the solved chain against the animated pose
    float        easeTime   = 0.15f;              // smoothing time-constant (s) for weight + target; 0 = snap

    // --- foot placement (isFootChain = true) ---
    // When enabled, UpdateIK casts a ray down from the animated ankle each frame
    // and drives targetWorldPos from the hit point instead of the authored value.
    bool      isFootChain    = false;
    float     castOffset     = 0.5f;    // ray origin = animated ankle + worldUp * castOffset
    float     maxStepHeight  = 0.5f;    // total ray length = castOffset + maxStepHeight
    float     ankleHeight    = 0.08f;   // ankle center above the floor (added to hit.point)
    float     swingThreshold = 0.1f;    // model-space ankle Y rise above bind pose that triggers swing (IK off)
    bool      tiltToNormal   = true;    // rotate the tip bone to match the surface normal
    float     tiltWeight     = 0.8f;    // blend weight for the foot tilt (0..1)

    // --- runtime cache (resolved/smoothed by UpdateIK, not authored/serialized) ---
    int          _tipBone      = -1;              // resolved from endEffectorBone (-1 = unresolved/invalid)
    float        _curWeight    = 0.0f;            // smoothed weight (eases toward `weight`)
    glm::vec3    _curTarget    { 0.0f };          // smoothed model-space target
    bool         _curTargetSet = false;           // false until _curTarget has been seeded (avoids a swing from the origin)
    glm::vec3    _groundNormal { 0.0f, 1.0f, 0.0f }; // smoothed world-space surface normal (foot chains)
    float        _rawDrop      = 0.0f;            // corrective drop this frame, fed to pelvis correction
    glm::vec3    _bindFootPos  { 0.0f };          // model-space ankle position in bind pose (swing detection)
    bool         _bindResolved = false;           // false until _bindFootPos has been cached
};

struct IKComponent {
    std::vector<IKChain> chains;

    // --- pelvis correction (foot placement) ---
    // Shifts the pelvis bone down by the max foot correction needed so the legs
    // can reach without hyperextending. Ignored when pelvisBone is empty.
    std::string pelvisBone     = "";
    float       maxPelvisDrop  = 0.5f;
    float       pelvisEaseTime = 0.1f;
    float       _curPelvisDrop = 0.0f;  // smoothed current drop (runtime)
    int         _pelvisBoneIdx = -1;    // cached bone index
};
