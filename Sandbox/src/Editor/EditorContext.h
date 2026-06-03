#pragma once
#include <entt/entt.hpp>
#include <string>
#include "Scene/Scene.h"

struct EditorContext {
    Scene*       ActiveScene    = nullptr;
    entt::entity selectedEntity = entt::null;
    std::string  currentScenePath;          // empty = unsaved new scene

    bool HasSelection() const { return selectedEntity != entt::null; }
};