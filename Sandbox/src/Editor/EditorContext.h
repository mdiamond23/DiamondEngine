#pragma once
#include <entt/entt.hpp>
#include "Scene/Scene.h"

struct EditorContext {
    Scene* ActiveScene = nullptr;
    entt::entity selectedEntity = entt::null;

    bool HasSelection() const { return selectedEntity != entt::null; }
};