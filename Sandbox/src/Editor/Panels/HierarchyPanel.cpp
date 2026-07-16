#include "HierarchyPanel.h"
#include "../Command.h"
#include "Scene/SceneSerializer.h"
#include <imgui.h>
#include <cctype>
#include <cstring>
#include <string>
#include <vector>
#include <optional>
#include "Scene/Components.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include "Scene/Physics/Rigidbody.h"
#include "Scene/Physics/Collision.h"
#include "Scene/Physics/Constraint.h"

static bool IsPrefabPath(const std::string& path)
{
    constexpr const char* kExt = ".prefab";
    constexpr size_t      kLen = 7;
    if (path.size() < kLen) return false;
    for (size_t i = 0; i < kLen; ++i)
        if (std::tolower((unsigned char)path[path.size() - kLen + i]) != kExt[i])
            return false;
    return true;
}

// ---- entity snapshot --------------------------------------------------------
// Captures enough state to fully recreate an entity (and its subtree) after
// deletion or for duplication. Stored in delete commands so undo can restore
// the entity, and in duplicate commands so redo can recreate the copy.

struct EntitySnapshot {
    std::string                         name;
    entt::entity                        parent = entt::null;
    TransformComponent                  transform;
    std::optional<MeshComponent>        mesh;
    std::optional<LightComponent>       light;
    std::optional<CameraComponent>      camera;
    std::optional<RigidBodyComponent>   rigidBody;
    std::optional<ColliderComponent>    collider;
    std::optional<ConstraintComponent>  constraint;
    std::optional<PrefabInstanceComponent> prefabInstance;
    std::optional<PrefabChildComponent>    prefabChild;
    // Script components captured generically via the ComponentRegistry —
    // (registry display name, serialized state). Any DECLARE_COMPONENT type
    // is included automatically; fidelity matches its SerializeComponent impl.
    std::vector<std::pair<std::string, std::string>> scriptComponents;
    std::vector<EntitySnapshot>         children;
};

static EntitySnapshot SnapshotEntity(Scene* scene, entt::entity e)
{
    auto& reg = scene->GetRegistry();
    EntitySnapshot s;
    s.name      = scene->GetEntityName(e);
    s.transform = reg.get<TransformComponent>(e);
    if (reg.all_of<MeshComponent>(e))   s.mesh   = reg.get<MeshComponent>(e);
    if (reg.all_of<LightComponent>(e))  s.light  = reg.get<LightComponent>(e);
    if (reg.all_of<CameraComponent>(e)) s.camera = reg.get<CameraComponent>(e);
    if (reg.all_of<RigidBodyComponent>(e)) {
        s.rigidBody = reg.get<RigidBodyComponent>(e);
        s.rigidBody->_bodyId = 0xFFFFFFFFu;  // restored entity gets its own physics body
    }
    if (reg.all_of<ColliderComponent>(e)) {
        s.collider = reg.get<ColliderComponent>(e);
        s.collider->_bodyId = 0xFFFFFFFFu;
    }
    if (reg.all_of<ConstraintComponent>(e)) {
        s.constraint = reg.get<ConstraintComponent>(e);
        s.constraint->_constraintId = 0xFFFFFFFFu;  // restored entity builds its own joint
    }
    if (reg.all_of<PrefabInstanceComponent>(e))
        s.prefabInstance = reg.get<PrefabInstanceComponent>(e);
    if (reg.all_of<PrefabChildComponent>(e))
        s.prefabChild = reg.get<PrefabChildComponent>(e);
    for (auto& desc : ComponentRegistry::Get().GetAll()) {
        if (desc.serialize && desc.has(*scene, e)) {
            try { s.scriptComponents.emplace_back(desc.name, desc.serialize(*scene, e)); }
            catch (...) {}
        }
    }
    if (reg.all_of<HierarchyComponent>(e)) {
        s.parent = reg.get<HierarchyComponent>(e).parent;
        for (auto child : reg.get<HierarchyComponent>(e).children)
            s.children.push_back(SnapshotEntity(scene, child));
    }
    return s;
}

// Recreates an entity from a snapshot. parentOverride is used for child
// entities so they are parented to the newly-created parent, not the old handle.
static entt::entity RestoreEntity(Scene* scene, const EntitySnapshot& s,
                                  entt::entity parentOverride = entt::null)
{
    auto& reg   = scene->GetRegistry();
    entt::entity e = scene->CreateEntity(s.name);
    reg.get<TransformComponent>(e) = s.transform;
    if (s.mesh)      reg.emplace_or_replace<MeshComponent>(e,      *s.mesh);
    if (s.light)     reg.emplace_or_replace<LightComponent>(e,     *s.light);
    if (s.camera)    reg.emplace_or_replace<CameraComponent>(e,    *s.camera);
    if (s.rigidBody)  reg.emplace_or_replace<RigidBodyComponent>(e, *s.rigidBody);
    if (s.collider)   reg.emplace_or_replace<ColliderComponent>(e,  *s.collider);
    if (s.constraint) reg.emplace_or_replace<ConstraintComponent>(e, *s.constraint);
    if (s.prefabInstance) reg.emplace_or_replace<PrefabInstanceComponent>(e, *s.prefabInstance);
    if (s.prefabChild)    reg.emplace_or_replace<PrefabChildComponent>(e, *s.prefabChild);

    for (const auto& [name, data] : s.scriptComponents) {
        for (auto& desc : ComponentRegistry::Get().GetAll()) {
            if (desc.name != name) continue;
            if (desc.add && !desc.has(*scene, e))
                desc.add(*scene, e);
            if (desc.deserialize) {
                try { desc.deserialize(*scene, e, data); }
                catch (...) {}
            }
            break;
        }
    }

    entt::entity parent = (parentOverride != entt::null) ? parentOverride : s.parent;
    if (parent != entt::null && reg.valid(parent))
        scene->SetParent(e, parent);

    for (const auto& child : s.children)
        RestoreEntity(scene, child, e);
    return e;
}

// Deep-copies per-entity mutable state so a duplicate doesn't alias the source
// entity. Asset-backed resources (mesh data, textures) stay shared; only the
// material parameter blocks editable in the inspector are cloned.
static void MakeSnapshotIndependent(EntitySnapshot& s)
{
    if (s.mesh && s.mesh->material)
        s.mesh->material = std::make_shared<Diamond::PBRMaterial>(*s.mesh->material);
    if (s.collider && s.collider->material)
        s.collider->material = std::make_shared<PhysicsMaterial>(*s.collider->material);
    for (auto& child : s.children)
        MakeSnapshotIndependent(child);
}

// ---- tree node --------------------------------------------------------------

struct HierarchyDrawCtx {
    Scene*          scene;
    EditorContext*  edCtx;
    entt::entity&   toDelete;
    EntitySnapshot* toDeleteSnap;
    entt::entity&   toDuplicate;
    entt::entity&   renamingEntity;
    char*           renameBuffer;
    bool&           renameFocusSet;
    entt::entity&   selectionPivot;
    // Prefab drop, deferred: instantiation creates entities, which would
    // invalidate the entity-map iteration driving the tree draw.
    std::string&    prefabToSpawn;
    entt::entity&   prefabSpawnParent;
    // Live-link after "Save to Prefab", deferred for the same reason:
    // propagation destroys/recreates the other instances of the file.
    std::string&    prefabToPropagate;
    entt::entity&   propagateExclude;
    const std::function<void(const std::string&)>* onEditPrefab;
};

static void DrawEntityNode(entt::entity entity, const HierarchyDrawCtx& ctx)
{
    auto& reg = ctx.scene->GetRegistry();
    const std::string& name = ctx.scene->GetEntityName(entity);

    bool hasChildren = reg.all_of<HierarchyComponent>(entity)
                    && !reg.get<HierarchyComponent>(entity).children.empty();
    bool selected    = ctx.edCtx->IsSelected(entity);
    bool renaming    = (ctx.renamingEntity == entity);

    // --- Rename mode ---------------------------------------------------------
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

    // Prefab instance roots draw blue; broken ones (source file missing) red.
    bool isPrefabRoot = reg.all_of<PrefabInstanceComponent>(entity);
    bool prefabBroken = isPrefabRoot && reg.get<PrefabInstanceComponent>(entity).broken;
    if (isPrefabRoot)
        ImGui::PushStyleColor(ImGuiCol_Text, prefabBroken ? ImVec4(0.95f, 0.38f, 0.38f, 1.0f)
                                                          : ImVec4(0.45f, 0.68f, 1.00f, 1.0f));

    bool opened = ImGui::TreeNodeEx((void*)(uintptr_t)(uint32_t)entity, flags, "%s", name.c_str());

    if (isPrefabRoot)
        ImGui::PopStyleColor();

    if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
        if (ImGui::GetIO().KeyCtrl) {
            ctx.edCtx->ToggleSelect(entity);
            ctx.selectionPivot = entity;
        } else if (!ctx.edCtx->IsSelected(entity)) {
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

            auto sharedChild = std::make_shared<entt::entity>(child);
            auto* scene = ctx.scene; auto* edCtx = ctx.edCtx;
            ctx.edCtx->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                [scene, sharedChild, entity, edCtx]() {
                    *sharedChild = scene->CreateEntity("Empty Entity");
                    scene->SetParent(*sharedChild, entity);
                    edCtx->SelectOnly(*sharedChild);
                },
                [scene, sharedChild, edCtx]() {
                    edCtx->ClearSelection();
                    if (scene->GetRegistry().valid(*sharedChild))
                        scene->DestroyEntity(*sharedChild);
                },
                "Create Child Entity"));
        }
        if (ImGui::MenuItem("Duplicate", "Ctrl+D")) {
            ctx.toDuplicate = entity;
        }
        // Instance-root actions: push scene edits back to the .prefab (then
        // live-link the file's other instances), or open it in prefab edit mode.
        if (reg.all_of<PrefabInstanceComponent>(entity)) {
            const auto& pi = reg.get<PrefabInstanceComponent>(entity);
            if (ImGui::MenuItem("Save to Prefab") && !pi.sourcePath.empty()) {
                ctx.prefabToPropagate = pi.sourcePath;
                ctx.propagateExclude  = entity;
            }
            if (ImGui::MenuItem("Edit Prefab") && !pi.sourcePath.empty()
                && ctx.onEditPrefab && *ctx.onEditPrefab)
                (*ctx.onEditPrefab)(pi.sourcePath);
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
            entt::entity oldParent = reg.get<HierarchyComponent>(entity).parent;
            ctx.scene->UnsetParent(entity);
            auto* scene = ctx.scene;
            ctx.edCtx->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                [scene, entity]()           { scene->UnsetParent(entity); },
                [scene, entity, oldParent]() { scene->SetParent(entity, oldParent); },
                "Unparent Entity"));
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Delete")) {
            *ctx.toDeleteSnap = SnapshotEntity(ctx.scene, entity);
            ctx.toDelete = entity;
        }
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
                auto* scene = ctx.scene; auto* edCtx = ctx.edCtx;
                if (ctx.edCtx->IsSelected(dragged) && ctx.edCtx->selectedEntities.size() > 1) {
                    // Multi-entity reparent — save old parents for all
                    std::vector<std::pair<entt::entity, entt::entity>> oldParents;
                    for (auto e : ctx.edCtx->selectedEntities) {
                        if (e == entity || ctx.scene->IsAncestorOf(e, entity)) continue;
                        entt::entity op = reg.all_of<HierarchyComponent>(e)
                            ? reg.get<HierarchyComponent>(e).parent : entt::null;
                        oldParents.push_back({e, op});
                        ctx.scene->SetParent(e, entity);
                    }
                    ctx.edCtx->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                        [scene, oldParents, newParent = entity]() {
                            for (auto [e, _] : oldParents) scene->SetParent(e, newParent);
                        },
                        [scene, oldParents]() {
                            for (auto [e, op] : oldParents) {
                                if (op != entt::null && scene->GetRegistry().valid(op))
                                    scene->SetParent(e, op);
                                else
                                    scene->UnsetParent(e);
                            }
                        },
                        "Reparent Entities"));
                } else {
                    entt::entity oldParent = reg.all_of<HierarchyComponent>(dragged)
                        ? reg.get<HierarchyComponent>(dragged).parent : entt::null;
                    ctx.scene->SetParent(dragged, entity);
                    ctx.edCtx->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                        [scene, dragged, newParent = entity]() { scene->SetParent(dragged, newParent); },
                        [scene, dragged, oldParent]() {
                            if (oldParent != entt::null && scene->GetRegistry().valid(oldParent))
                                scene->SetParent(dragged, oldParent);
                            else
                                scene->UnsetParent(dragged);
                        },
                        "Reparent Entity"));
                }
            }
        }
        // .prefab from the Content Browser → instantiate parented under this entity.
        if (auto* payload = ImGui::AcceptDragDropPayload("CONTENT_ITEM_PATH")) {
            std::string path(static_cast<const char*>(payload->Data));
            if (IsPrefabPath(path)) {
                ctx.prefabToSpawn     = path;
                ctx.prefabSpawnParent = entity;
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
    if (!m_Open) return;
    ImGui::Begin("Hierarchy", &m_Open);

    if (!m_Context || !m_Context->ActiveScene) {
        ImGui::End();
        return;
    }

    Scene* scene = m_Context->ActiveScene;
    auto&  reg   = scene->GetRegistry();

    if (ImGui::Button("+ Add Entity")) {
        entt::entity created = scene->CreateEntity("Empty Entity");
        auto sharedCreated = std::make_shared<entt::entity>(created);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, sharedCreated]() { *sharedCreated = scene->CreateEntity("Empty Entity"); },
            [scene, sharedCreated]() {
                if (scene->GetRegistry().valid(*sharedCreated))
                    scene->DestroyEntity(*sharedCreated);
            },
            "Add Entity"));
    }

    ImGui::Separator();

    entt::entity   toDelete     = entt::null;
    EntitySnapshot toDeleteSnap;
    entt::entity   toDuplicate  = entt::null;
    std::string    prefabToSpawn;
    entt::entity   prefabSpawnParent = entt::null;
    std::string    prefabToPropagate;
    entt::entity   propagateExclude  = entt::null;

    HierarchyDrawCtx ctx {
        scene,
        m_Context,
        toDelete,
        &toDeleteSnap,
        toDuplicate,
        m_RenamingEntity,
        m_RenameBuffer,
        m_RenameFocusSet,
        m_SelectionPivot,
        prefabToSpawn,
        prefabSpawnParent,
        prefabToPropagate,
        propagateExclude,
        &m_OnEditPrefab
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
                    std::vector<std::pair<entt::entity, entt::entity>> oldParents;
                    for (auto e : m_Context->selectedEntities) {
                        entt::entity op = reg.all_of<HierarchyComponent>(e)
                            ? reg.get<HierarchyComponent>(e).parent : entt::null;
                        oldParents.push_back({e, op});
                        scene->UnsetParent(e);
                    }
                    m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                        [scene, oldParents]() {
                            for (auto [e, _] : oldParents) scene->UnsetParent(e);
                        },
                        [scene, oldParents]() {
                            for (auto [e, op] : oldParents)
                                if (op != entt::null && scene->GetRegistry().valid(op))
                                    scene->SetParent(e, op);
                        },
                        "Make Entities Root"));
                } else {
                    entt::entity oldParent = reg.all_of<HierarchyComponent>(dragged)
                        ? reg.get<HierarchyComponent>(dragged).parent : entt::null;
                    scene->UnsetParent(dragged);
                    m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                        [scene, dragged]() { scene->UnsetParent(dragged); },
                        [scene, dragged, oldParent]() {
                            if (oldParent != entt::null && scene->GetRegistry().valid(oldParent))
                                scene->SetParent(dragged, oldParent);
                        },
                        "Make Entity Root"));
                }
            }
            // .prefab from the Content Browser → instantiate as a scene root.
            if (auto* payload = ImGui::AcceptDragDropPayload("CONTENT_ITEM_PATH")) {
                std::string path(static_cast<const char*>(payload->Data));
                if (IsPrefabPath(path)) {
                    prefabToSpawn     = path;
                    prefabSpawnParent = entt::null;
                }
            }
            ImGui::EndDragDropTarget();
        }
    }

    // Deferred prefab instantiation (queued by the drop targets above).
    if (!prefabToSpawn.empty()) {
        auto* edCtx = m_Context;
        auto sharedRoot = std::make_shared<entt::entity>(entt::null);
        auto doSpawn = [scene, edCtx, sharedRoot,
                        path = prefabToSpawn, parent = prefabSpawnParent]() {
            entt::entity root = PrefabSerializer::Instantiate(*scene, path);
            *sharedRoot = root;
            if (root == entt::null) return;
            if (parent != entt::null && scene->GetRegistry().valid(parent))
                scene->SetParent(root, parent);
            edCtx->SelectOnly(root);
        };
        doSpawn();
        if (*sharedRoot != entt::null) {
            m_SelectionPivot = *sharedRoot;
            m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                doSpawn,
                [scene, edCtx, sharedRoot]() {
                    edCtx->ClearSelection();
                    if (scene->GetRegistry().valid(*sharedRoot))
                        scene->DestroyEntity(*sharedRoot);
                },
                "Instantiate Prefab"));
        }
    }

    // Deferred "Save to Prefab" + live-link (queued in the context menu): the
    // save itself is cheap, but rebuilding the file's other instances
    // destroys/creates entities, which the tree iteration can't survive.
    // Re-instantiation invalidates arbitrary entity handles, so the undo
    // history is dropped and dead selections pruned.
    if (!prefabToPropagate.empty() && reg.valid(propagateExclude)) {
        int n = PrefabSerializer::SaveAndPropagate(*scene, propagateExclude, prefabToPropagate);
        if (n > 0) {
            m_Context->Commands.Clear();
            for (auto it = m_Context->selectedEntities.begin();
                 it != m_Context->selectedEntities.end();)
                it = reg.valid(*it) ? std::next(it) : m_Context->selectedEntities.erase(it);
            if (!reg.valid(m_SelectionPivot)) m_SelectionPivot = entt::null;
        }
    }

    // Ctrl+A — select all entities
    if (ImGui::IsWindowFocused() && ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_A)) {
        for (auto& [entity, name] : scene->GetEntityNames())
            m_Context->selectedEntities.insert(entity);
    }

    // Delete key — snapshot then destroy all selected (filtering out descendants
    // already covered by their ancestor's snapshot).
    if (ImGui::IsWindowFocused() && (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace))
        && !m_Context->selectedEntities.empty()
        && m_RenamingEntity == entt::null)
    {
        std::vector<EntitySnapshot> snapshots;
        std::vector<entt::entity>   toDestroyAll;

        for (auto e : m_Context->selectedEntities) {
            if (!reg.valid(e)) continue;
            bool hasSelectedAncestor = false;
            for (auto other : m_Context->selectedEntities) {
                if (other != e && reg.valid(other) && scene->IsAncestorOf(other, e)) {
                    hasSelectedAncestor = true;
                    break;
                }
            }
            if (!hasSelectedAncestor) {
                snapshots.push_back(SnapshotEntity(scene, e));
                toDestroyAll.push_back(e);
            }
        }

        m_Context->ClearSelection();
        m_SelectionPivot = entt::null;
        for (auto e : toDestroyAll)
            if (reg.valid(e))
                scene->DestroyEntity(e);

        auto sharedHandles = std::make_shared<std::vector<entt::entity>>();
        auto* ctx = m_Context;
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, snapshots, sharedHandles, ctx]() {
                ctx->ClearSelection();
                for (auto e : *sharedHandles)
                    if (scene->GetRegistry().valid(e))
                        scene->DestroyEntity(e);
                sharedHandles->clear();
            },
            [scene, snapshots, sharedHandles, ctx]() {
                sharedHandles->clear();
                for (const auto& s : snapshots) {
                    entt::entity e = RestoreEntity(scene, s);
                    sharedHandles->push_back(e);
                    ctx->selectedEntities.insert(e);
                }
            },
            snapshots.size() == 1 ? "Delete Entity" : "Delete Entities"));
    }

    // Context menu single-entity delete (deferred so mc/lc refs are dead).
    if (toDelete != entt::null) {
        // Clean up selection
        std::vector<entt::entity> toRemove;
        for (auto e : m_Context->selectedEntities)
            if (e == toDelete || scene->IsAncestorOf(toDelete, e))
                toRemove.push_back(e);
        for (auto e : toRemove)
            m_Context->selectedEntities.erase(e);
        if (m_SelectionPivot == toDelete || scene->IsAncestorOf(toDelete, m_SelectionPivot))
            m_SelectionPivot = entt::null;

        EntitySnapshot snap = toDeleteSnap;
        scene->DestroyEntity(toDelete);

        auto sharedHandle = std::make_shared<entt::entity>(entt::null);
        auto* ctx = m_Context;
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, snap, sharedHandle, ctx]() {
                ctx->ClearSelection();
                if (scene->GetRegistry().valid(*sharedHandle))
                    scene->DestroyEntity(*sharedHandle);
                *sharedHandle = entt::null;
            },
            [scene, snap, sharedHandle, ctx]() {
                entt::entity e = RestoreEntity(scene, snap);
                *sharedHandle = e;
                ctx->SelectOnly(e);
            },
            "Delete Entity"));
    }

    // Duplicate — context menu target, or Ctrl+D on the selection (filtering
    // out descendants already covered by their ancestor's subtree).
    std::vector<entt::entity> toDuplicateAll;
    if (toDuplicate != entt::null && reg.valid(toDuplicate)) {
        toDuplicateAll.push_back(toDuplicate);
    }
    else if (ImGui::IsWindowFocused() && ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_D)
             && !m_Context->selectedEntities.empty()
             && m_RenamingEntity == entt::null)
    {
        for (auto e : m_Context->selectedEntities) {
            if (!reg.valid(e)) continue;
            bool hasSelectedAncestor = false;
            for (auto other : m_Context->selectedEntities) {
                if (other != e && reg.valid(other) && scene->IsAncestorOf(other, e)) {
                    hasSelectedAncestor = true;
                    break;
                }
            }
            if (!hasSelectedAncestor)
                toDuplicateAll.push_back(e);
        }
    }

    if (!toDuplicateAll.empty()) {
        std::vector<EntitySnapshot> snapshots;
        for (auto e : toDuplicateAll) {
            EntitySnapshot s = SnapshotEntity(scene, e);
            MakeSnapshotIndependent(s);
            s.name += " (Copy)";
            snapshots.push_back(std::move(s));
        }

        auto sharedHandles = std::make_shared<std::vector<entt::entity>>();
        auto* edCtx = m_Context;
        auto doDuplicate = [scene, snapshots, sharedHandles, edCtx]() {
            edCtx->ClearSelection();
            sharedHandles->clear();
            for (const auto& s : snapshots) {
                entt::entity e = RestoreEntity(scene, s);
                sharedHandles->push_back(e);
                edCtx->selectedEntities.insert(e);
            }
        };
        doDuplicate();
        m_SelectionPivot = sharedHandles->empty() ? entt::null : sharedHandles->front();

        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            doDuplicate,
            [scene, sharedHandles, edCtx]() {
                edCtx->ClearSelection();
                for (auto e : *sharedHandles)
                    if (scene->GetRegistry().valid(e))
                        scene->DestroyEntity(e);
                sharedHandles->clear();
            },
            snapshots.size() == 1 ? "Duplicate Entity" : "Duplicate Entities"));
    }

    ImGui::End();
}
