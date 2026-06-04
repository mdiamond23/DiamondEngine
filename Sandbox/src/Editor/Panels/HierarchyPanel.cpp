#include "HierarchyPanel.h"
#include <imgui.h>
#include <cstring>
#include "Scene/Components.h"
#include "Scene/Scene.h"

// ---- tree node --------------------------------------------------------------

struct HierarchyDrawCtx {
    Scene*         scene;
    EditorContext* edCtx;
    entt::entity&  toDelete;
    entt::entity&  renamingEntity;
    char*          renameBuffer;
    bool&          renameFocusSet;
};

static void DrawEntityNode(entt::entity entity, const HierarchyDrawCtx& ctx)
{
    auto& reg = ctx.scene->GetRegistry();
    const std::string& name = ctx.scene->GetEntityName(entity);

    bool hasChildren = reg.all_of<HierarchyComponent>(entity)
                    && !reg.get<HierarchyComponent>(entity).children.empty();
    bool selected    = (ctx.edCtx->selectedEntity == entity);
    bool renaming    = (ctx.renamingEntity == entity);

    // --- Rename mode: show InputText instead of the tree node ----------------
    if (renaming) {
        if (!ctx.renameFocusSet) {
            ImGui::SetKeyboardFocusHere();
            ctx.renameFocusSet = true;
        }
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::InputText("##rename", ctx.renameBuffer, 256,
                             ImGuiInputTextFlags_EnterReturnsTrue))
        {
            if (ctx.renameBuffer[0] != '\0')
                ctx.scene->SetEntityName(entity, ctx.renameBuffer);
            ctx.renamingEntity = entt::null;
        }
        else if (!ImGui::IsItemActive() && ImGui::IsItemDeactivated())
        {
            if (ctx.renameBuffer[0] != '\0')
                ctx.scene->SetEntityName(entity, ctx.renameBuffer);
            ctx.renamingEntity = entt::null;
        }
        else if (ImGui::IsKeyPressed(ImGuiKey_Escape))
        {
            ctx.renamingEntity = entt::null;
        }
        return;
    }

    // --- Normal tree node ----------------------------------------------------
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow
                             | ImGuiTreeNodeFlags_SpanAvailWidth
                             | ImGuiTreeNodeFlags_OpenOnDoubleClick;
    if (selected)     flags |= ImGuiTreeNodeFlags_Selected;
    if (!hasChildren) flags |= ImGuiTreeNodeFlags_Leaf;

    bool opened = ImGui::TreeNodeEx((void*)(uintptr_t)(uint32_t)entity, flags, "%s", name.c_str());

    if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
        ctx.edCtx->selectedEntity = entity;

    // --- Context menu --------------------------------------------------------
    if (ImGui::BeginPopupContextItem()) {
        if (ImGui::MenuItem("Create Child Entity")) {
            entt::entity child = ctx.scene->CreateEntity("Empty Entity");
            ctx.scene->SetParent(child, entity);
            ctx.edCtx->selectedEntity = child;
        }
        if (ImGui::MenuItem("Rename")) {
            ctx.renamingEntity  = entity;
            ctx.renameFocusSet  = false;
            std::strncpy(ctx.renameBuffer, name.c_str(), 255);
            ctx.renameBuffer[255] = '\0';
        }
        if (ImGui::MenuItem("Unparent") && reg.all_of<HierarchyComponent>(entity)
            && reg.get<HierarchyComponent>(entity).parent != entt::null)
        {
            ctx.scene->UnsetParent(entity);
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Delete"))
            ctx.toDelete = entity;
        ImGui::EndPopup();
    }

    // --- Drag source ---------------------------------------------------------
    if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
        ImGui::SetDragDropPayload("HIERARCHY_ENTITY", &entity, sizeof(entt::entity));
        ImGui::Text("Move: %s", name.c_str());
        ImGui::EndDragDropSource();
    }

    // --- Drop target (reparent onto this entity) -----------------------------
    if (ImGui::BeginDragDropTarget()) {
        if (auto* payload = ImGui::AcceptDragDropPayload("HIERARCHY_ENTITY")) {
            entt::entity dragged = *(const entt::entity*)payload->Data;
            if (dragged != entity)
                ctx.scene->SetParent(dragged, entity);   // cycle guard is inside SetParent
        }
        ImGui::EndDragDropTarget();
    }

    // --- Recurse -------------------------------------------------------------
    if (opened) {
        if (hasChildren)
            for (entt::entity child : reg.get<HierarchyComponent>(entity).children)
                DrawEntityNode(child, ctx);
        ImGui::TreePop();
    }
}

// ---- panel ------------------------------------------------------------------

void HierarchyPanel::OnImGuiRender() {
    ImGui::Begin("Hierarchy");

    if (!m_Context || !m_Context->ActiveScene) {
        ImGui::End();
        return;
    }

    Scene* scene = m_Context->ActiveScene;
    auto&  reg   = scene->GetRegistry();

    if (ImGui::Button("+ Add Entity"))
        scene->CreateEntity("Empty Entity");

    ImGui::Separator();

    entt::entity toDelete = entt::null;

    HierarchyDrawCtx ctx {
        scene,
        m_Context,
        toDelete,
        m_RenamingEntity,
        m_RenameBuffer,
        m_RenameFocusSet
    };

    // Render roots only — they recurse into children.
    for (auto& [entity, name] : scene->GetEntityNames()) {
        bool isRoot = !reg.all_of<HierarchyComponent>(entity)
                   || reg.get<HierarchyComponent>(entity).parent == entt::null;
        if (isRoot)
            DrawEntityNode(entity, ctx);
    }

    // Drop onto the blank area below all entities → make dragged entity a root.
    ImVec2 remaining = ImGui::GetContentRegionAvail();
    if (remaining.y > 0.0f) {
        ImGui::InvisibleButton("##hierRoot", {-1.0f, remaining.y});
        if (ImGui::BeginDragDropTarget()) {
            if (auto* payload = ImGui::AcceptDragDropPayload("HIERARCHY_ENTITY")) {
                entt::entity dragged = *(const entt::entity*)payload->Data;
                scene->UnsetParent(dragged);
            }
            ImGui::EndDragDropTarget();
        }
    }

    if (toDelete != entt::null) {
        // Clear selection if the selected entity is the one being deleted or
        // lives anywhere inside its subtree (cascade-destroyed children).
        if (m_Context->selectedEntity == toDelete ||
            scene->IsAncestorOf(toDelete, m_Context->selectedEntity))
            m_Context->selectedEntity = entt::null;
        scene->DestroyEntity(toDelete);
    }

    ImGui::End();
}
