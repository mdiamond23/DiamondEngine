#pragma once
#include "Panels.h"
#include "../EditorContext.h"
#include <glm/glm.hpp>
#include <optional>

class ContentPanel;

class InspectorPanel : public Panel {
public:
    void OnImGuiRender() override;
    void SetContext(EditorContext* context) { m_Context = context; }
    void SetContentPanel(ContentPanel* cp)  { m_ContentPanel = cp; }
private:
    EditorContext* m_Context      = nullptr;
    ContentPanel*  m_ContentPanel = nullptr;

    bool m_Renaming           = false;
    char m_RenameBuffer[256]  = {};
    bool m_RenameFocusSet     = false;

    // Transform drag snapshots
    glm::vec3 m_OldPosition     = {};
    glm::vec3 m_OldEulerDegrees = {};
    glm::vec3 m_OldScale        = { 1, 1, 1 };

    // Light field drag snapshots
    glm::vec3 m_OldLightColor  = { 1, 1, 1 };
    float     m_OldIntensity   = 1.0f;
    float     m_OldRadius      = 10.0f;
    float     m_OldInnerCone   = 30.0f;
    float     m_OldOuterCone   = 45.0f;

    // Mesh material drag snapshots
    float m_OldUVScale          = 1.0f;
    float m_OldEmissiveStrength = 0.0f;

    // Saved component state for deferred remove commands
    std::optional<MeshComponent>  m_PendingRemovedMesh;
    std::optional<LightComponent> m_PendingRemovedLight;
};
