#pragma once
#include "Panels.h"
#include "../EditorContext.h"

class HierarchyPanel : public Panel {
public:
    void OnImGuiRender() override;
    void SetContext(EditorContext* context) { m_Context = context; }
private:
    EditorContext* m_Context = nullptr;
};
