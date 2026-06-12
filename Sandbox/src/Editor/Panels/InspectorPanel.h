#pragma once
#include "Panels.h"
#include "../EditorContext.h"
#include <glm/glm.hpp>
#include <optional>
#include <vector>
#include "Scene/Physics/Collision.h"
#include "Scene/Physics/Rigidbody.h"

class ContentPanel;

class InspectorPanel : public Panel {
public:
    void OnImGuiRender() override;
    void SetContext(EditorContext* context) { m_Context = context; }
    void SetContentPanel(ContentPanel* cp)  { m_ContentPanel = cp; }
private:
    // Unity-style multi-object editing — common components, mixed-value fields.
    void DrawMultiInspector(const std::vector<entt::entity>& entities);

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

    // Camera field drag snapshots
    float m_OldFov     = 60.0f;
    float m_OldNear    = 0.1f;
    float m_OldFar     = 1000.0f;

    // Collider drag snapshots
    glm::vec3 m_OldColHalfExtents    = { 0.5f, 0.5f, 0.5f };
    float     m_OldColRadius         = 0.5f;
    float     m_OldColHalfHeight     = 0.5f;
    glm::vec3 m_OldColOffset         = {};
    float     m_OldStaticFriction    = 0.5f;
    float     m_OldDynamicFriction   = 0.5f;
    float     m_OldRestitution       = 0.3f;

    // Rigidbody drag snapshots
    float m_OldRbMass            = 1.0f;
    float m_OldRbLinearDamping   = 0.05f;
    float m_OldRbAngularDamping  = 0.05f;
    float m_OldRbGravityScale    = 1.0f;

    // Saved component state for deferred remove commands
    std::optional<MeshComponent>       m_PendingRemovedMesh;
    std::optional<LightComponent>      m_PendingRemovedLight;
    std::optional<CameraComponent>     m_PendingRemovedCamera;
    std::optional<ColliderComponent>   m_PendingRemovedCollider;
    std::optional<RigidBodyComponent>  m_PendingRemovedRigidbody;
};
