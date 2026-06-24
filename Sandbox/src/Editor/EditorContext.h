#pragma once
#include <entt/entt.hpp>
#include <string>
#include <unordered_set>
#include <glm/glm.hpp>
#include <imgui.h>
#include "Scene/Scene.h"
#include "Core/Camera.h"
#include "Command.h"

struct EditorContext {
    Scene*            ActiveScene   = nullptr;
    ImFont*           IconFont      = nullptr;
    Diamond::Camera*  EditorCamera  = nullptr;
    CommandManager    Commands;
    std::unordered_set<entt::entity> selectedEntities;
    std::string  currentScenePath;

    glm::mat4 viewMatrix  = glm::mat4(1.0f);
    glm::mat4 projMatrix  = glm::mat4(1.0f);
    glm::vec3 cameraPos   = {};

    // Mouse position within a viewport, normalized to 0..1 (top-left origin).
    // Written each frame by the owning panel; components < 0 or >= 1 mean the
    // cursor is outside that viewport. Used to map the OS mouse into the
    // full-window UI framebuffer for UI hit-testing. The Scene one feeds the
    // editor; the Game one feeds the in-game HUD during play.
    glm::vec2 viewportMouseNorm     { -1.0f, -1.0f };
    glm::vec2 gameViewportMouseNorm { -1.0f, -1.0f };

    // Viewport debug visualization — one master toggle gating collider wireframes,
    // ragdoll bodies, and IK chain/target draw (+ the IK target gizmo).
    bool showDebugDraw = false;
    // Which chain of the selected entity's IKComponent gets the viewport target
    // gizmo and the highlighted draw. Clamped to the chain list by the inspector.
    int  activeIKChain = 0;

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
