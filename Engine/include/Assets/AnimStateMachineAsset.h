#pragma once
#include <Animation/AnimationStateMachine.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <memory>
#include <filesystem>

// Load/save for the .animsm asset (the reusable animation state machine graph).
// Editor-side because JSON lives in Sandbox, not the Engine — same split as
// PhysicsMaterialAsset. The data model itself is Diamond::AnimStateMachine.

// Normalize mixed forward/back slashes to native (see NormalizePhysMatPath for why).
inline std::string NormalizeAnimSmPath(const std::string& path)
{
    return std::filesystem::path(path).make_preferred().string();
}

// ---- enum <-> string (stable on-disk names, robust to enum reordering) -------

inline const char* AnimParamTypeToStr(Diamond::AnimParamType t)
{
    switch (t) {
        case Diamond::AnimParamType::Bool:    return "bool";
        case Diamond::AnimParamType::Trigger: return "trigger";
        default:                              return "float";
    }
}
inline Diamond::AnimParamType AnimParamTypeFromStr(const std::string& s)
{
    if (s == "bool")    return Diamond::AnimParamType::Bool;
    if (s == "trigger") return Diamond::AnimParamType::Trigger;
    return Diamond::AnimParamType::Float;
}

inline const char* AnimCondOpToStr(Diamond::AnimCondOp op)
{
    using O = Diamond::AnimCondOp;
    switch (op) {
        case O::Greater:      return "gt";
        case O::Less:         return "lt";
        case O::GreaterEqual: return "ge";
        case O::LessEqual:    return "le";
        case O::Equal:        return "eq";
        case O::NotEqual:     return "ne";
        case O::IsTrue:       return "true";
        case O::IsFalse:      return "false";
        case O::Triggered:    return "triggered";
    }
    return "gt";
}
inline Diamond::AnimCondOp AnimCondOpFromStr(const std::string& s)
{
    using O = Diamond::AnimCondOp;
    if (s == "lt")        return O::Less;
    if (s == "ge")        return O::GreaterEqual;
    if (s == "le")        return O::LessEqual;
    if (s == "eq")        return O::Equal;
    if (s == "ne")        return O::NotEqual;
    if (s == "true")      return O::IsTrue;
    if (s == "false")     return O::IsFalse;
    if (s == "triggered") return O::Triggered;
    return O::Greater;
}

// ---- load --------------------------------------------------------------------

inline std::shared_ptr<Diamond::AnimStateMachine> LoadAnimStateMachine(const std::string& path)
{
    std::ifstream f{std::filesystem::path(path)};
    if (!f.is_open()) return nullptr;
    auto j = nlohmann::json::parse(f, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return nullptr;

    auto sm = std::make_shared<Diamond::AnimStateMachine>();
    sm->name       = j.value("name", std::string{});
    sm->entryState = j.value("entryState", 0);

    for (const auto& pj : j.value("parameters", nlohmann::json::array())) {
        Diamond::AnimParameter p;
        p.name         = pj.value("name", std::string{});
        p.type         = AnimParamTypeFromStr(pj.value("type", "float"));
        p.defaultFloat = pj.value("defaultFloat", 0.0f);
        p.defaultBool  = pj.value("defaultBool",  false);
        sm->parameters.push_back(std::move(p));
    }

    for (const auto& sj : j.value("states", nlohmann::json::array())) {
        Diamond::AnimState s;
        s.name     = sj.value("name", std::string{});
        s.clipName = sj.value("clipName", std::string{});
        s.speed    = sj.value("speed", 1.0f);
        s.loop     = sj.value("loop", true);
        auto pos   = sj.value("nodePos", std::vector<float>{0.0f, 0.0f});
        if (pos.size() == 2) s.nodePos = { pos[0], pos[1] };
        sm->states.push_back(std::move(s));
    }

    for (const auto& tj : j.value("transitions", nlohmann::json::array())) {
        Diamond::AnimTransition t;
        t.fromState     = tj.value("fromState", -1);
        t.toState       = tj.value("toState", -1);
        t.blendDuration = tj.value("blendDuration", 0.2f);
        for (const auto& cj : tj.value("conditions", nlohmann::json::array())) {
            Diamond::AnimCondition c;
            c.parameter = cj.value("parameter", std::string{});
            c.op        = AnimCondOpFromStr(cj.value("op", "gt"));
            c.value     = cj.value("value", 0.0f);
            t.conditions.push_back(std::move(c));
        }
        sm->transitions.push_back(std::move(t));
    }

    return sm;
}

// ---- save --------------------------------------------------------------------

inline bool SaveAnimStateMachine(const std::string& path, const Diamond::AnimStateMachine& sm)
{
    std::ofstream f{std::filesystem::path(path)};
    if (!f.is_open()) return false;

    nlohmann::json j;
    j["name"]       = sm.name;
    j["entryState"] = sm.entryState;

    j["parameters"] = nlohmann::json::array();
    for (const auto& p : sm.parameters) {
        j["parameters"].push_back({
            { "name",         p.name },
            { "type",         AnimParamTypeToStr(p.type) },
            { "defaultFloat", p.defaultFloat },
            { "defaultBool",  p.defaultBool }
        });
    }

    j["states"] = nlohmann::json::array();
    for (const auto& s : sm.states) {
        j["states"].push_back({
            { "name",     s.name },
            { "clipName", s.clipName },
            { "speed",    s.speed },
            { "loop",     s.loop },
            { "nodePos",  { s.nodePos.x, s.nodePos.y } }
        });
    }

    j["transitions"] = nlohmann::json::array();
    for (const auto& t : sm.transitions) {
        nlohmann::json cj = nlohmann::json::array();
        for (const auto& c : t.conditions) {
            cj.push_back({
                { "parameter", c.parameter },
                { "op",        AnimCondOpToStr(c.op) },
                { "value",     c.value }
            });
        }
        j["transitions"].push_back({
            { "fromState",     t.fromState },
            { "toState",       t.toState },
            { "blendDuration", t.blendDuration },
            { "conditions",    cj }
        });
    }

    f << j.dump(4);
    return true;
}
