#include "HierarchyPanel.h"
#include <imgui.h>
#include <cstring>
#include <vector>
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
    entt::entity&  selectionPivot;
};

static void DrawEntityNode(entt::entity entity, const HierarchyDrawCtx& ctx)
{
    auto& reg = ctx.scene->GetRegistry();
    const std::string& name = ctx.scene->GetEntityName(entity);

    bool hasChildren = reg.all_of<HierarchyComponent>(entity)
                    && !reg.get<HierarchyComponent>(entity).children.empty();
    bool selected    = ctx.edCtx->IsSelected(entity);
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

    if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
        if (ImGui::GetIO().KeyCtrl) {
            ctx.edCtx->ToggleSelect(entity);
            ctx.selectionPivot = entity;
        } else if (!ctx.edCtx->IsSelected(entity)) {
            // Only clear selection when clicking an unselected entity.
            // Clicking an already-selected entity keeps the multi-selection
            // intact so it can be dragged as a group.
            ctx.edCtx->SelectOnly(entity);
            ctx.selectionPivot = entity;
        }
    }

    // --- Context menu --------------------------------------------------------
    if (ImGui::BeginPopupContextItem()) {
        if (ImGui::MenuItem("Create Child Entity")) {
            entt::entity child = ctx.scene->CreateEntity("Empty Entity");
            ctx.scene->SetParent(child, entity);
            ctx.edCtx->SelectOnly(child);
            ctx.selectionPivot = child;
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
        bool multiDrag = ctx.edCtx->IsSelected(entity) && ctx.edCtx->selectedEntities.size() > 1;
        if (multiDrag)
            ImGui::Text("Moving %zu entities", ctx.edCtx->selectedEntities.size());
        else
            ImGui::Text("Move: %s", name.c_str());
        ImGui::EndDragDropSource();
    }

    // --- Drop target (reparent onto this entity) -----------------------------
    if (ImGui::BeginDragDropTarget()) {
        if (auto* payload = ImGui::AcceptDragDropPayload("HIERARCHY_ENTITY")) {
            entt::entity dragged = *(const entt::entity*)payload->Data;
            if (dragged != entity) {
                if (ctx.edCtx->IsSelected(dragged) && ctx.edCtx->selectedEntities.size() > 1) {
                    for (auto e : ctx.edCtx->selectedEntities)
                        if (e != entity && !ctx.scene->IsAncestorOf(e, entity))
                            ctx.scene->SetParent(e, entity);
                } else {
                    ctx.scene->SetParent(dragged, entity);
                }
            }
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
        m_RenameFocusSet,
        m_SelectionPivot
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
                if (m_Context->IsSelected(dragged) && m_Context->selectedEntities.size() > 1) {
                    for (auto e : m_Context->selectedEntities)
                        scene->UnsetParent(e);
                } else {
                    scene->UnsetParent(dragged);
                }
            }
            ImGui::EndDragDropTarget();
        }
    }

    // Ctrl+A — select all entities
    if (ImGui::IsWindowFocused() && ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_A)) {
        for (auto& [entity, name] : scene->GetEntityNames())
            m_Context->selectedEntities.insert(entity);
    }

    // Delete key — delete all selected entities (guard: no rename active)
    if (ImGui::IsWindowFocused() && (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace))
        && !m_Context->selectedEntities.empty()
        && m_RenamingEntity == entt::null)
    {
        std::vector<entt::entity> toDeleteAll(
            m_Context->selectedEntities.begin(),
            m_Context->selectedEntities.end());
        m_Context->ClearSelection();
        m_SelectionPivot = entt::null;
        for (auto e : toDeleteAll)
            if (reg.valid(e))
                scene->DestroyEntity(e);
    }

    // Context menu single-entity delete
    if (toDelete != entt::null) {
        std::vector<entt::entity> toRemove;
        for (auto e : m_Context->selectedEntities)
            if (e == toDelete || scene->IsAncestorOf(toDelete, e))
                toRemove.push_back(e);
        for (auto e : toRemove)
            m_Context->selectedEntities.erase(e);
        if (m_SelectionPivot == toDelete || scene->IsAncestorOf(toDelete, m_SelectionPivot))
            m_SelectionPivot = entt::null;
        scene->DestroyEntity(toDelete);
    }

    ImGui::End();
}
