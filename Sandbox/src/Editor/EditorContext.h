#pragma once
#include <entt/entt.hpp>
#include <string>
#include <unordered_set>
#include <glm/glm.hpp>
#include "Scene/Scene.h"
#include "Command.h"

struct EditorContext {
    Scene*          ActiveScene    = nullptr;
    CommandManager  Commands;
    std::unordered_set<entt::entity> selectedEntities;
    std::string  currentScenePath;

    glm::mat4 viewMatrix  = glm::mat4(1.0f);
    glm::mat4 projMatrix  = glm::mat4(1.0f);
    glm::vec3 cameraPos   = {};

    bool HasSelection() const { return !selectedEntities.empty(); }

    entt::entity PrimarySelection() const {
        if (selectedEntities.empty()) return entt::null;
        return *selectedEntities.begin();
    }

    bool IsSelected(entt::entity e) const {
        return selectedEntities.count(e) > 0;
    }

    void SelectOnly(entt::entity e) {
        selectedEntities.clear();
        if (e != entt::null) selectedEntities.insert(e);
    }

    void ToggleSelect(entt::entity e) {
        if (e == entt::null) return;
        if (selectedEntities.count(e)) selectedEntities.erase(e);
        else selectedEntities.insert(e);
    }

    void ClearSelection() { selectedEntities.clear(); }
};
