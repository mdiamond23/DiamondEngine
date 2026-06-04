#pragma once
#include "Panels.h"
#include "../EditorContext.h"

class ContentPanel;

class InspectorPanel : public Panel {
public:
    void OnImGuiRender() override;
    void SetContext(EditorContext* context) { m_Context = context; }
    void SetContentPanel(ContentPanel* cp)  { m_ContentPanel = cp; }
private:
    EditorContext* m_Context      = nullptr;
    ContentPanel*  m_ContentPanel = nullptr;
};
