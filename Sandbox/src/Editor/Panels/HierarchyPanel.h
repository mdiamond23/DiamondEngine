#pragma once
#include "Panels.h"
#include "../EditorContext.h"
#include <entt/entt.hpp>
#include <functional>
#include <string>

class HierarchyPanel : public Panel {
public:
    void OnImGuiRender() override;
    const char* GetName() const override { return "Hierarchy"; }
    void SetContext(EditorContext* context) { m_Context = context; }
    // Fired by "Edit Prefab" on an instance root — the owner switches the
    // editor into prefab edit mode for that .prefab.
    void SetOnEditPrefab(std::function<void(const std::string&)> cb) { m_OnEditPrefab = std::move(cb); }
private:
    EditorContext* m_Context           = nullptr;
    std::function<void(const std::string&)> m_OnEditPrefab;
    entt::entity   m_RenamingEntity    = entt::null;
    char           m_RenameBuffer[256] = {};
    bool           m_RenameFocusSet    = false;
    entt::entity   m_SelectionPivot    = entt::null;
};
