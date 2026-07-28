#include "Scene/Physics/RagdollConfig.h"
#include "Animation/Skeleton.h"

#include <glm/gtc/matrix_access.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>

// Best-effort ragdoll auto-generation. Classifies bones by coarse role hints +
// chain topology rather than an exact name table, because real rigs name bones
// wildly differently (CesiumMan: "Skeleton_arm_joint_L__4_", "leg_joint_R_5",
// with inconsistent L/R numbering). The output seeds an editable .ragdoll asset;
// it is expected to be hand-tuned, not trusted verbatim. See Docs/ragdoll-design.md.

namespace {

enum class Role { None, Spine, Neck, Head, Arm, Leg };

std::string Lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return (char)std::tolower(c); });
    return s;
}
bool Has(const std::string& hay, const char* needle) {
    return hay.find(needle) != std::string::npos;
}

Role ClassifyBone(const std::string& rawName) {
    const std::string n = Lower(rawName);
    if (Has(n, "head"))                                           return Role::Head;
    if (Has(n, "neck"))                                           return Role::Neck;
    if (Has(n, "torso") || Has(n, "spine") || Has(n, "chest") ||
        Has(n, "hip")   || Has(n, "pelvis"))                     return Role::Spine;
    if (Has(n, "arm")   || Has(n, "shoulder") || Has(n, "hand") ||
        Has(n, "forearm") || Has(n, "clavicle"))                 return Role::Arm;
    if (Has(n, "leg")   || Has(n, "thigh") || Has(n, "shin") ||
        Has(n, "calf")  || Has(n, "foot")  || Has(n, "knee") ||
        Has(n, "ankle"))                                          return Role::Leg;
    return Role::None;
}

// Hinge if the name clearly names a 1-DOF joint, OR (fallback) it's the second
// bone down a limb chain (the elbow / knee position).
bool LooksLikeHinge(const std::string& rawName, Role role, int limbDepth) {
    const std::string n = Lower(rawName);
    if (Has(n, "elbow") || Has(n, "knee") || Has(n, "forearm") ||
        Has(n, "shin")  || Has(n, "calf") || Has(n, "lowerarm") ||
        Has(n, "lowerleg"))
        return true;
    return (role == Role::Arm || role == Role::Leg) && limbDepth == 1;
}

} // namespace

RagdollConfig BuildDefaultRagdoll(const Diamond::Skeleton& skel)
{
    RagdollConfig cfg;
    const auto& bones = skel.bones;
    const int   n     = (int)bones.size();
    if (n == 0) return cfg;

    // --- per-bone precompute: role, bind-pose world transform, children ---------
    std::vector<Role>      role(n);
    std::vector<glm::mat4> bindWorld(n);   // bone -> mesh space at bind
    std::vector<glm::vec3> bindPos(n);
    std::vector<std::vector<int>> children(n);

    for (int i = 0; i < n; ++i) {
        role[i]      = ClassifyBone(bones[i].name);
        bindWorld[i] = glm::inverse(bones[i].inverseBind);
        bindPos[i]   = glm::vec3(bindWorld[i][3]);
        if (bones[i].parent >= 0 && bones[i].parent < n)
            children[bones[i].parent].push_back(i);
    }

    auto included = [&](int i) { return i >= 0 && i < n && role[i] != Role::None; };

    // Nearest ancestor bone that is itself included (becomes the body's parent).
    auto nearestIncludedAncestor = [&](int i) {
        for (int p = bones[i].parent; p >= 0; p = bones[p].parent)
            if (included(p)) return p;
        return -1;
    };

    // Depth within a limb: 0 at the limb root (first bone whose included ancestor
    // has a different role), incrementing down same-role included ancestors.
    auto limbDepth = [&](int i) {
        int depth = 0;
        for (int a = nearestIncludedAncestor(i); a >= 0 && role[a] == role[i];
             a = nearestIncludedAncestor(a))
            ++depth;
        return depth;
    };

    // Pick a child bone to size the capsule toward — prefer an included child so
    // the capsule spans to the next body; else any child.
    auto sizingChild = [&](int i) {
        int fallback = -1;
        for (int c : children[i]) {
            if (fallback < 0) fallback = c;
            if (included(c))  return c;
        }
        return fallback;
    };

    for (int i = 0; i < n; ++i) {
        if (!included(i)) continue;

        RagdollBodyDef def;
        def.boneName = bones[i].name;
        int parent   = nearestIncludedAncestor(i);
        def.parentBoneName = (parent >= 0) ? bones[parent].name : std::string{};

        const int   depth = limbDepth(i);
        const Role  r     = role[i];
        const std::string ln = Lower(bones[i].name);

        // --- size from the bind pose -------------------------------------------
        glm::vec3 dirLocal { 0.0f, 1.0f, 0.0f };
        float     length = 0.12f;   // leaf fallback
        int       child  = sizingChild(i);
        if (child >= 0) {
            glm::vec3 dirWorld = bindPos[child] - bindPos[i];
            float     len      = glm::length(dirWorld);
            if (len > 1e-5f) {
                length = len;
                // direction to the child expressed in this bone's local frame
                glm::vec3 dl = glm::mat3(bones[i].inverseBind) * dirWorld;
                if (glm::length(dl) > 1e-6f) dirLocal = glm::normalize(dl);
            }
        }
        def.twistAxisLocal = dirLocal;

        float radius = std::clamp(length * 0.18f, 0.02f, 0.2f);
        def.radius     = radius;
        def.halfHeight = std::max(length * 0.5f - radius, 0.01f);
        def.halfExtents = glm::vec3(std::clamp(length * 0.30f, 0.03f, 0.25f),
                                    std::max(length * 0.5f, 0.03f),
                                    std::clamp(length * 0.30f, 0.03f, 0.25f));

        // --- shape -------------------------------------------------------------
        if (r == Role::Head || Has(ln, "head"))
            def.shape = RagdollBodyDef::Shape::Sphere;
        else if (Has(ln, "hip") || Has(ln, "pelvis") || (r == Role::Spine && parent < 0))
            def.shape = RagdollBodyDef::Shape::Box;     // the torso root
        else
            def.shape = RagdollBodyDef::Shape::Capsule;

        // --- joint type + limits ----------------------------------------------
        if (LooksLikeHinge(bones[i].name, r, depth)) {
            def.jointType   = ConstraintType::Hinge;
            def.hingeMinDeg  = 0.0f;
            def.hingeMaxDeg  = 150.0f;

            // The capsule axis follows the limb; the hinge axis must be the normal of
            // its bend plane. Infer that plane from the incoming/outgoing bind-pose
            // segments. Near-perfectly straight chains fall back to any stable axis
            // perpendicular to the capsule instead of the old axial-twist mistake.
            glm::vec3 hingeWorld(0.0f);
            if (parent >= 0 && child >= 0)
                hingeWorld = glm::cross(bindPos[i] - bindPos[parent],
                                        bindPos[child] - bindPos[i]);
            if (glm::dot(hingeWorld, hingeWorld) > 1e-10f) {
                const glm::vec3 hingeLocal = glm::mat3(bones[i].inverseBind)
                                           * glm::normalize(hingeWorld);
                if (glm::dot(hingeLocal, hingeLocal) > 1e-10f)
                    def.hingeAxisLocal = glm::normalize(hingeLocal);
            } else {
                const glm::vec3 reference = std::abs(dirLocal.y) < 0.9f
                    ? glm::vec3(0.0f, 1.0f, 0.0f)
                    : glm::vec3(1.0f, 0.0f, 0.0f);
                def.hingeAxisLocal = glm::normalize(glm::cross(dirLocal, reference));
            }
        } else {
            def.jointType    = ConstraintType::SwingTwist;
            // Tighter cones for the spine/neck, wider for shoulders/hips.
            const bool ballJoint = (r == Role::Arm || r == Role::Leg);
            def.swingNormalDeg = ballJoint ? 60.0f : 25.0f;
            def.swingPlaneDeg  = ballJoint ? 45.0f : 20.0f;
            def.twistMinDeg    = ballJoint ? -45.0f : -15.0f;
            def.twistMaxDeg    = ballJoint ?  45.0f :  15.0f;
        }

        // --- mass (rough, by role + chain depth, kg) ---------------------------
        switch (r) {
            case Role::Spine: def.mass = (parent < 0) ? 12.0f : 8.0f; break;
            case Role::Neck:  def.mass = 2.0f;  break;
            case Role::Head:  def.mass = 4.0f;  break;
            case Role::Arm:   def.mass = (depth == 0) ? 2.5f : (depth == 1 ? 1.5f : 0.5f); break;
            case Role::Leg:   def.mass = (depth == 0) ? 7.0f : (depth == 1 ? 4.0f : 1.0f); break;
            default:          def.mass = 1.0f;  break;
        }

        cfg.bodies.push_back(std::move(def));
    }

    spdlog::info("BuildDefaultRagdoll: {} bodies from {} skeleton bones",
                 cfg.bodies.size(), n);
    return cfg;
}
