#pragma once
#include "Panels.h"
#include "../EditorContext.h"
#include <glm/glm.hpp>
#include <functional>
#include <optional>
#include <vector>
#include "Scene/Physics/Collision.h"
#include "Scene/Physics/Rigidbody.h"
#include "Scene/Physics/Constraint.h"
#include "Animation/AnimationComponents.h"

class ContentPanel;
namespace Diamond { struct PBRMaterial; }

class InspectorPanel : public Panel {
public:
    void OnImGuiRender() override;
    const char* GetName() const override { return "Inspector"; }
    void SetContext(EditorContext* context) { m_Context = context; }
    void SetContentPanel(ContentPanel* cp)  { m_ContentPanel = cp; }
    // Vulkan backend hook (no-op under GL): called on material edit-release so
    // the renderer can drop its baked descriptor set for the material. See
    // Diamond::EditorBackend::InvalidateMaterial.
    void SetMaterialInvalidator(std::function<void(const Diamond::PBRMaterial*)> fn) {
        m_MaterialInvalidator = std::move(fn);
    }
private:
    // Unity-style multi-object editing — common components, mixed-value fields.
    void DrawMultiInspector(const std::vector<entt::entity>& entities);

    EditorContext* m_Context      = nullptr;
    ContentPanel*  m_ContentPanel = nullptr;
    std::function<void(const Diamond::PBRMaterial*)> m_MaterialInvalidator;

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
    float     m_OldSkyIntensity = 1.0f;
    float     m_OldRadius      = 10.0f;
    float     m_OldInnerCone   = 30.0f;
    float     m_OldOuterCone   = 45.0f;

    // Mesh material drag snapshots
    float m_OldUVScale          = 1.0f;
    float m_OldEmissiveStrength = 0.0f;
    float m_OldAlphaCutoff      = 0.5f;

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

    // Animator drag snapshots
    float m_OldAnimSpeed = 1.0f;
    // Cross-fade duration applied when switching clips in the inspector (editor-only).
    float m_AnimBlendTime = 0.2f;

    // Saved component state for deferred remove commands
    std::optional<MeshComponent>         m_PendingRemovedMesh;
    std::optional<LightComponent>        m_PendingRemovedLight;
    std::optional<SkyLightComponent>     m_PendingRemovedSkyLight;
    std::optional<CameraComponent>       m_PendingRemovedCamera;
    std::optional<ColliderComponent>     m_PendingRemovedCollider;
    std::optional<RigidBodyComponent>    m_PendingRemovedRigidbody;
    std::optional<ConstraintComponent>   m_PendingRemovedConstraint;
    std::optional<SkinnedMeshComponent>  m_PendingRemovedSkinnedMesh;
    std::optional<AnimatorComponent>     m_PendingRemovedAnimator;
    std::optional<AnimStateMachineComponent> m_PendingRemovedAnimSM;
};
