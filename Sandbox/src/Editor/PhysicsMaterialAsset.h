#pragma once
#include <Scene/Physics/Collision.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <memory>
#include <filesystem>

// Normalize any mix of forward / back slashes to the platform-native separator.
// Mixed separators (e.g. "C:/Dev\Assets\foo.physmat") are produced when CMake's
// forward-slash ASSETS_DIR is concatenated with Windows backslash directory
// entries; std::ifstream(std::string) on MSVC cannot open them reliably.
inline std::string NormalizePhysMatPath(const std::string& path)
{
    return std::filesystem::path(path).make_preferred().string();
}

inline std::shared_ptr<PhysicsMaterial> LoadPhysicsMaterial(const std::string& path)
{
    std::filesystem::path fp{path};
    std::ifstream f{fp};
    if (!f.is_open()) return nullptr;
    auto j = nlohmann::json::parse(f, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return nullptr;
    auto m = std::make_shared<PhysicsMaterial>();
    m->staticFriction  = j.value("staticFriction",  0.5f);
    m->dynamicFriction = j.value("dynamicFriction", 0.5f);
    m->restitution     = j.value("restitution",     0.3f);
    return m;
}

inline bool SavePhysicsMaterial(const std::string& path, const PhysicsMaterial& m)
{
    std::filesystem::path fp{path};
    std::ofstream f{fp};
    if (!f.is_open()) return false;
    nlohmann::json j = {
        { "staticFriction",  m.staticFriction  },
        { "dynamicFriction", m.dynamicFriction },
        { "restitution",     m.restitution     }
    };
    f << j.dump(4);
    return true;
}
