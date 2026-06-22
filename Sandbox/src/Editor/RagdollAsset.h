#pragma once
#include <Scene/Physics/RagdollConfig.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <filesystem>

// Load/save for the .ragdoll asset (the ragdoll body+joint configuration).
// Editor-side because JSON lives in Sandbox, not the Engine — same split as
// AnimStateMachineAsset / PhysicsMaterialAsset. The data model itself is
// RagdollConfig (Engine/include/Scene/Physics/RagdollConfig.h).

// Normalize mixed forward/back slashes to native (same reason as NormalizeAnimSmPath).
inline std::string NormalizeRagdollPath(const std::string& path)
{
    return std::filesystem::path(path).make_preferred().string();
}

// ---- enum <-> string (stable on-disk names, robust to enum reordering) -------

inline const char* RagdollShapeToStr(RagdollBodyDef::Shape s)
{
    switch (s) {
        case RagdollBodyDef::Shape::Box:    return "box";
        case RagdollBodyDef::Shape::Sphere: return "sphere";
        default:                            return "capsule";
    }
}
inline RagdollBodyDef::Shape RagdollShapeFromStr(const std::string& s)
{
    if (s == "box")    return RagdollBodyDef::Shape::Box;
    if (s == "sphere") return RagdollBodyDef::Shape::Sphere;
    return RagdollBodyDef::Shape::Capsule;
}

// Only Hinge / SwingTwist are meaningful for ragdoll joints, but round-trip the
// full ConstraintType so the format doesn't silently drop a hand-authored value.
inline const char* RagdollJointToStr(ConstraintType t)
{
    switch (t) {
        case ConstraintType::Hinge: return "hinge";
        case ConstraintType::Fixed: return "fixed";
        case ConstraintType::Point: return "point";
        default:                    return "swingtwist";
    }
}
inline ConstraintType RagdollJointFromStr(const std::string& s)
{
    if (s == "hinge") return ConstraintType::Hinge;
    if (s == "fixed") return ConstraintType::Fixed;
    if (s == "point") return ConstraintType::Point;
    return ConstraintType::SwingTwist;
}

// ---- small glm <-> json helpers ---------------------------------------------

inline nlohmann::json Vec3ToJson(const glm::vec3& v) { return { v.x, v.y, v.z }; }
inline glm::vec3 Vec3FromJson(const nlohmann::json& j, const glm::vec3& fallback)
{
    if (!j.is_array() || j.size() != 3) return fallback;
    return { j[0].get<float>(), j[1].get<float>(), j[2].get<float>() };
}

// ---- load --------------------------------------------------------------------

inline bool LoadRagdoll(const std::string& path, RagdollConfig& out)
{
    std::ifstream f{std::filesystem::path(path)};
    if (!f.is_open()) return false;
    auto j = nlohmann::json::parse(f, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return false;

    out = RagdollConfig{};
    out.impactThreshold = j.value("impactThreshold", 0.0f);

    for (const auto& bj : j.value("bodies", nlohmann::json::array())) {
        RagdollBodyDef d;
        d.boneName       = bj.value("bone", std::string{});
        d.parentBoneName = bj.value("parent", std::string{});
        d.mass           = bj.value("mass", 1.0f);

        if (bj.contains("shape")) {
            const auto& sj = bj["shape"];
            d.shape       = RagdollShapeFromStr(sj.value("type", "capsule"));
            d.radius      = sj.value("radius", 0.05f);
            d.halfHeight  = sj.value("halfHeight", 0.10f);
            d.halfExtents = Vec3FromJson(sj.value("halfExtents", nlohmann::json{}), d.halfExtents);
        }

        if (bj.contains("joint") && bj["joint"].is_object()) {
            const auto& jj = bj["joint"];
            d.jointType      = RagdollJointFromStr(jj.value("type", "swingtwist"));
            d.twistAxisLocal = Vec3FromJson(jj.value("twistAxis", nlohmann::json{}), d.twistAxisLocal);
            d.swingNormalDeg = jj.value("swingNormalDeg", d.swingNormalDeg);
            d.swingPlaneDeg  = jj.value("swingPlaneDeg",  d.swingPlaneDeg);
            d.twistMinDeg    = jj.value("twistMinDeg",    d.twistMinDeg);
            d.twistMaxDeg    = jj.value("twistMaxDeg",    d.twistMaxDeg);
            d.hingeMinDeg    = jj.value("hingeMinDeg",    d.hingeMinDeg);
            d.hingeMaxDeg    = jj.value("hingeMaxDeg",    d.hingeMaxDeg);
            d.motorMaxTorque = jj.value("motorMaxTorque", d.motorMaxTorque);
            d.motorFrequency = jj.value("motorFrequency", d.motorFrequency);
            d.motorDamping   = jj.value("motorDamping",   d.motorDamping);
        }

        out.bodies.push_back(std::move(d));
    }
    return true;
}

// ---- save --------------------------------------------------------------------

inline bool SaveRagdoll(const std::string& path, const RagdollConfig& cfg)
{
    std::ofstream f{std::filesystem::path(path)};
    if (!f.is_open()) return false;

    nlohmann::json j;
    j["version"]         = 1;
    j["impactThreshold"] = cfg.impactThreshold;
    j["bodies"]          = nlohmann::json::array();

    for (const auto& d : cfg.bodies) {
        nlohmann::json bj;
        bj["bone"]   = d.boneName;
        bj["parent"] = d.parentBoneName;
        bj["mass"]   = d.mass;
        bj["shape"]  = {
            { "type",        RagdollShapeToStr(d.shape) },
            { "radius",      d.radius },
            { "halfHeight",  d.halfHeight },
            { "halfExtents", Vec3ToJson(d.halfExtents) }
        };
        // Root bodies have no joint to a parent.
        if (!d.parentBoneName.empty()) {
            bj["joint"] = {
                { "type",          RagdollJointToStr(d.jointType) },
                { "twistAxis",     Vec3ToJson(d.twistAxisLocal) },
                { "swingNormalDeg", d.swingNormalDeg },
                { "swingPlaneDeg",  d.swingPlaneDeg },
                { "twistMinDeg",    d.twistMinDeg },
                { "twistMaxDeg",    d.twistMaxDeg },
                { "hingeMinDeg",    d.hingeMinDeg },
                { "hingeMaxDeg",    d.hingeMaxDeg },
                { "motorMaxTorque", d.motorMaxTorque },
                { "motorFrequency", d.motorFrequency },
                { "motorDamping",   d.motorDamping }
            };
        }
        j["bodies"].push_back(std::move(bj));
    }

    f << j.dump(4);
    return true;
}
