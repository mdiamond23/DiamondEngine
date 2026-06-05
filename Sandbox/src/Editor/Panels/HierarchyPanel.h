#pragma once
#include "Panels.h"
#include "../EditorContext.h"
#include <entt/entt.hpp>

class HierarchyPanel : public Panel {
public:
    void OnImGuiRender() override;
    void SetContext(EditorContext* context) { m_Context = context; }
private:
    EditorContext* m_Context           = nullptr;
    entt::entity   m_RenamingEntity    = entt::null;
    char           m_RenameBuffer[256] = {};
    bool           m_RenameFocusSet    = false;
    entt::entity   m_SelectionPivot    = entt::null;
};
