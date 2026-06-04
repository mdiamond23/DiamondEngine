#pragma once
#include <entt/entt.hpp>
#include <string>
#include <glm/glm.hpp>
#include "Scene/Scene.h"

struct EditorContext {
    Scene*       ActiveScene    = nullptr;
    entt::entity selectedEntity = entt::null;
    std::string  currentScenePath;          // empty = unsaved new scene

    // Updated each frame by EditorLayer::UpdateCamera before ImGui renders.
    glm::mat4 viewMatrix  = glm::mat4(1.0f);
    glm::mat4 projMatrix  = glm::mat4(1.0f);
    glm::vec3 cameraPos   = {};

    bool HasSelection() const { return selectedEntity != entt::null; }
};