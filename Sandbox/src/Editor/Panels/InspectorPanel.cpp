#include "InspectorPanel.h"
#include "ContentPanel.h"
#include "../Command.h"
#include "../PhysicsMaterialAsset.h"
#include "../AnimStateMachineAsset.h"
#include "../MaterialAsset.h"
#include "../RagdollAsset.h"      // ragdoll auto-gen + .ragdoll I/O (BuildDefaultRagdoll, SaveRagdoll)
#include <spdlog/spdlog.h>
#include <imgui.h>
#include <imgui_internal.h>   // ImGuiItemFlags_MixedValue for tri-state checkboxes
#include <glm/gtc/type_ptr.hpp>
#include <cstring>
#include "Scene/Components.h"
#include "Scene/ComponentRegistry.h"
#include "Scene/Physics/Collision.h"
#include "Scene/Physics/Rigidbody.h"
#include "Scene/Physics/Constraint.h"
#include "Scene/Physics/RagdollComponent.h"
#include "Scene/Physics/PhysicsAPI.h"
#include "Animation/IKComponent.h"
#include "Assets/ModelImporter.h"
#include "Assets/GltfImporter.h"
#include "Renderer/MeshData.h"
#include "Renderer/TextureData.h"
#include "Renderer/Font.h"
#include <algorithm>
#include <filesystem>

#ifndef ASSETS_DIR
#define ASSETS_DIR "Assets"
#endif

using namespace Diamond;

// ---- helpers ----------------------------------------------------------------

static std::string Basename(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    return pos == std::string::npos ? path : path.substr(pos + 1);
}

static std::string LowerExtOf(const std::string& path) {
    size_t dot = path.rfind('.');
    if (dot == std::string::npos) return {};
    std::string e = path.substr(dot);
    for (char& c : e) c = (char)std::tolower((unsigned char)c);
    return e;
}

static bool IsMeshPath(const std::string& p) {
    std::string e = LowerExtOf(p);
    return e == ".obj" || e == ".fbx" || e == ".gltf" || e == ".glb";
}

static bool IsSkinnedMeshPath(const std::string& p) {
    std::string e = LowerExtOf(p);
    return e == ".gltf" || e == ".glb";   // rigged-character format (cgltf path)
}

static bool IsTexturePath(const std::string& p) {
    std::string e = LowerExtOf(p);
    return e == ".png" || e == ".jpg" || e == ".jpeg" || e == ".bmp" || e == ".tga";
}

// ---- mesh slot --------------------------------------------------------------
// A 54 px tall bordered box: thumbnail on the left, filename + path on the right.
// Accepts CONTENT_ITEM_PATH drag-drop payloads; updates mc on a valid mesh drop.
// Returns old state if a change occurred so the caller can submit an undo command.

struct MeshSlotChange {
    bool                       changed    = false;
    std::shared_ptr<Mesh>      oldMesh;
    AABB                       oldBounds  = {};
    std::string                oldMeshPath;
    int                        oldSubIndex = 0;
    std::shared_ptr<PBRMaterial> oldMaterial;
};

static MeshSlotChange DrawMeshSlot(MeshComponent& mc, ContentPanel* cp) {
    MeshSlotChange result;

    float   avail  = ImGui::GetContentRegionAvail().x;
    float   slotH  = 54.0f;
    ImVec2  origin = ImGui::GetCursorScreenPos();

    ImGui::InvisibleButton("##meshslot", {avail, slotH});
    bool hov = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRect(origin, {origin.x + avail, origin.y + slotH},
                hov ? IM_COL32(80, 140, 255, 180) : IM_COL32(65, 65, 65, 255), 4.0f);

    // Thumbnail
    float   thumbSz = slotH - 6.0f;
    ImVec2  tMin    = {origin.x + 3.0f, origin.y + 3.0f};
    ImVec2  tMax    = {tMin.x + thumbSz, tMin.y + thumbSz};

    uint64_t thumbID = (cp && !mc.meshPath.empty())
                       ? cp->GetThumbnail(mc.meshPath, AssetType::Mesh)
                       : 0;

    if (thumbID)
        dl->AddImage((ImTextureID)thumbID, tMin, tMax);
    else
        dl->AddRectFilled(tMin, tMax, IM_COL32(40, 40, 40, 220), 3.0f);

    // Name + path text
    float lineH = ImGui::GetTextLineHeight();
    float tx    = tMax.x + 6.0f;
    float ty    = origin.y + (slotH * 0.5f) - lineH - 1.0f;

    std::string meshName = mc.meshPath.empty() ? "None" : Basename(mc.meshPath);
    dl->AddText({tx, ty},
                mc.meshPath.empty() ? IM_COL32(100, 100, 100, 255) : IM_COL32(220, 220, 220, 255),
                meshName.c_str());

    if (!mc.meshPath.empty())
        dl->AddText({tx, ty + lineH + 2.0f}, IM_COL32(95, 95, 95, 255), mc.meshPath.c_str());
    else
        dl->AddText({tx, ty + lineH + 2.0f}, IM_COL32(70, 70, 70, 255), "Drop .obj / .fbx / .gltf here");

    // Drop target
    if (ImGui::BeginDragDropTarget()) {
        if (auto* payload = ImGui::AcceptDragDropPayload("CONTENT_ITEM_PATH")) {
            std::string path((const char*)payload->Data);
            if (IsMeshPath(path)) {
                auto meshes = ModelImporter::Load(path);
                if (!meshes.empty()) {
                    result.oldMesh     = mc.mesh;
                    result.oldBounds   = mc.localBounds;
                    result.oldMeshPath = mc.meshPath;
                    result.oldSubIndex = mc.meshSubIndex;
                    result.oldMaterial = mc.material;

                    mc.mesh         = Mesh::Create(meshes[0]);
                    mc.localBounds  = meshes[0].ComputeAABB();
                    mc.meshPath     = path;
                    mc.meshSubIndex = 0;
                    if (!mc.material)
                        mc.material = std::make_shared<PBRMaterial>();

                    result.changed = true;
                }
            }
        }
        ImGui::EndDragDropTarget();
    }

    return result;
}

// ---- skinned mesh slot ------------------------------------------------------
// Same bordered box as the mesh slot, but accepts only rigged glTF/GLB. On a
// valid drop it runs the full cgltf import (geometry + skeleton + clips) and
// repopulates `smc`. Returns a copy of the pre-drop component so the caller can
// bracket the change in one undo command (the component is heavyweight but
// trivially copyable — vectors of shared_ptr, the skeleton, and the clips).

struct SkinnedSlotChange {
    bool                 changed = false;
    SkinnedMeshComponent oldComp;
};

static SkinnedSlotChange DrawSkinnedMeshSlot(SkinnedMeshComponent& smc, ContentPanel* cp) {
    SkinnedSlotChange result;

    float   avail  = ImGui::GetContentRegionAvail().x;
    float   slotH  = 54.0f;
    ImVec2  origin = ImGui::GetCursorScreenPos();

    ImGui::InvisibleButton("##skinnedslot", {avail, slotH});
    bool hov = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRect(origin, {origin.x + avail, origin.y + slotH},
                hov ? IM_COL32(80, 215, 205, 200) : IM_COL32(65, 65, 65, 255), 4.0f);

    float   thumbSz = slotH - 6.0f;
    ImVec2  tMin    = {origin.x + 3.0f, origin.y + 3.0f};
    ImVec2  tMax    = {tMin.x + thumbSz, tMin.y + thumbSz};

    uint64_t thumbID = (cp && !smc.meshPath.empty())
                       ? cp->GetThumbnail(smc.meshPath, AssetType::SkinnedMesh)
                       : 0;
    if (thumbID)
        dl->AddImage((ImTextureID)thumbID, tMin, tMax);
    else
        dl->AddRectFilled(tMin, tMax, IM_COL32(40, 40, 40, 220), 3.0f);

    float lineH = ImGui::GetTextLineHeight();
    float tx    = tMax.x + 6.0f;
    float ty    = origin.y + (slotH * 0.5f) - lineH - 1.0f;

    std::string meshName = smc.meshPath.empty() ? "None" : Basename(smc.meshPath);
    dl->AddText({tx, ty},
                smc.meshPath.empty() ? IM_COL32(100, 100, 100, 255) : IM_COL32(220, 220, 220, 255),
                meshName.c_str());
    dl->AddText({tx, ty + lineH + 2.0f},
                smc.meshPath.empty() ? IM_COL32(70, 70, 70, 255) : IM_COL32(95, 95, 95, 255),
                smc.meshPath.empty() ? "Drop a rigged .gltf / .glb here" : smc.meshPath.c_str());

    if (ImGui::BeginDragDropTarget()) {
        if (auto* payload = ImGui::AcceptDragDropPayload("CONTENT_ITEM_PATH")) {
            std::string path((const char*)payload->Data);
            if (IsSkinnedMeshPath(path)) {
                ImportedModel model = GltfImporter::LoadModel(path);
                if (!model.skeleton.bones.empty() && !model.meshes.empty()) {
                    result.oldComp = smc;   // snapshot for undo

                    smc.meshes.clear();
                    AABB bounds;
                    for (auto& md : model.meshes) {
                        smc.meshes.push_back(Mesh::Create(md));
                        AABB b     = md.ComputeAABB();
                        bounds.min = glm::min(bounds.min, b.min);
                        bounds.max = glm::max(bounds.max, b.max);
                    }
                    smc.skeleton    = std::move(model.skeleton);
                    smc.clips       = std::move(model.animations);
                    smc.localBounds = bounds;
                    smc.meshPath    = path;
                    smc.material    = model.material ? model.material
                                    : (smc.material ? smc.material : std::make_shared<PBRMaterial>());

                    result.changed = true;
                }
            }
        }
        ImGui::EndDragDropTarget();
    }

    return result;
}

// ---- texture row ------------------------------------------------------------
// [24×24 thumbnail]  Label  filename  [x]
// Returns old state in TextureRowChange::changed so the caller can submit an undo command.

struct TextureRowChange {
    bool                     changed = false;
    std::shared_ptr<Texture> oldTex;
    std::string              oldPath;
};

static TextureRowChange DrawTextureRow(
    const char*               label,
    std::shared_ptr<Texture>& texPtr,
    std::string&              texPath,
    ContentPanel*             cp)
{
    TextureRowChange result;
    ImGui::PushID(label);

    const float sz = 24.0f;
    ImVec2 p = ImGui::GetCursorScreenPos();

    ImGui::InvisibleButton("##th", {sz, sz});
    bool hov = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);

    ImDrawList* dl = ImGui::GetWindowDrawList();
    uint64_t thumbID = (cp && !texPath.empty())
                       ? cp->GetThumbnail(texPath, AssetType::Texture)
                       : 0;

    if (thumbID)
        dl->AddImage((ImTextureID)thumbID, p, {p.x + sz, p.y + sz});
    else
        dl->AddRectFilled(p, {p.x + sz, p.y + sz}, IM_COL32(42, 42, 42, 255), 2.0f);

    dl->AddRect(p, {p.x + sz, p.y + sz},
                hov ? IM_COL32(80, 140, 255, 210) : IM_COL32(75, 75, 75, 200), 2.0f);

    if (ImGui::BeginDragDropTarget()) {
        if (auto* payload = ImGui::AcceptDragDropPayload("CONTENT_ITEM_PATH")) {
            std::string path((const char*)payload->Data);
            if (IsTexturePath(path)) {
                result.oldTex  = texPtr;
                result.oldPath = texPath;
                texPtr  = Texture::Create(path, false);
                texPath = path;
                result.changed = true;
            }
        }
        ImGui::EndDragDropTarget();
    }

    ImGui::SameLine(0.0f, 6.0f);

    // Slot label (grayed)
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(130, 130, 130, 255));
    ImGui::Text("%-10s", label);
    ImGui::PopStyleColor();

    ImGui::SameLine(0.0f, 4.0f);

    // Filename or "None"
    if (texPath.empty()) {
        ImGui::TextDisabled("None");
    } else {
        ImGui::TextUnformatted(Basename(texPath).c_str());
        ImGui::SameLine(0.0f, 4.0f);
        if (ImGui::SmallButton("x")) {
            result.oldTex  = texPtr;
            result.oldPath = texPath;
            texPtr.reset();
            texPath.clear();
            result.changed = true;
        }
    }

    ImGui::PopID();
    return result;
}

// ---- multi-edit helpers -------------------------------------------------------
// Unity-style multi-object editing: a widget shows the value when it is
// identical across the selection and "--" when mixed; editing writes to every
// selected entity and records a single undo command for the whole selection.

// Old values captured at drag start. One active widget at a time in ImGui, so
// a single per-type stash is safe.
template<typename T>
struct MultiOldVals { static inline std::vector<std::pair<entt::entity, T>> v; };

// Builds get/set accessors for a component member from a pointer-to-member, so
// call sites stay one-liners and undo lambdas can capture them by value.
template<typename C, typename T>
static auto FieldGet(T C::* m) {
    return [m](Scene* s, entt::entity e) -> T { return s->GetRegistry().get<C>(e).*m; };
}
template<typename C, typename T>
static auto FieldSet(T C::* m) {
    return [m](Scene* s, entt::entity e, const T& v) { s->GetRegistry().get<C>(e).*m = v; };
}

// widget(T& value, bool mixed) draws exactly one ImGui item and returns true if
// it edited the value. instantCommit is for click-complete widgets (checkbox,
// combo) that have no drag cycle to bracket with activate/deactivate snapshots.
template<typename T, typename GetFn, typename SetFn, typename WidgetFn>
static void MultiEdit(EditorContext* ed, Scene* scene, const std::vector<entt::entity>& ents,
                      GetFn get, SetFn set, WidgetFn widget, const char* desc,
                      bool instantCommit = false)
{
    T first = get(scene, ents[0]);
    bool mixed = false;
    for (size_t i = 1; i < ents.size() && !mixed; ++i)
        mixed = !(get(scene, ents[i]) == first);

    T v = first;
    bool edited = widget(v, mixed);

    if (instantCommit) {
        if (!edited) return;
        std::vector<std::pair<entt::entity, T>> old;
        for (auto e : ents) old.push_back({e, get(scene, e)});
        for (auto e : ents) set(scene, e, v);
        T nv = v;
        ed->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, old, nv, set]() { for (auto& p : old) set(scene, p.first, nv); },
            [scene, old, set]()     { for (auto& p : old) set(scene, p.first, p.second); },
            desc));
        return;
    }

    if (ImGui::IsItemActivated()) {
        auto& old = MultiOldVals<T>::v;
        old.clear();
        for (auto e : ents) old.push_back({e, get(scene, e)});
    }
    if (edited)
        for (auto e : ents) set(scene, e, v);
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        auto old = MultiOldVals<T>::v;
        T nv = v;
        ed->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, old, nv, set]() { for (auto& p : old) set(scene, p.first, nv); },
            [scene, old, set]()     { for (auto& p : old) set(scene, p.first, p.second); },
            desc));
    }
}

// Checkbox that renders the ImGui mixed-value (dash) state when the selection
// disagrees. Clicking always commits a uniform value to every entity.
static bool MixedCheckbox(const char* label, bool& v, bool mixed)
{
    if (mixed) ImGui::PushItemFlag(ImGuiItemFlags_MixedValue, true);
    bool r = ImGui::Checkbox(label, &v);
    if (mixed) ImGui::PopItemFlag();
    return r;
}

// ---- panel ------------------------------------------------------------------

void InspectorPanel::OnImGuiRender() {
    ImGui::Begin("Inspector");

    if (!m_Context || !m_Context->HasSelection()) {
        ImGui::Text("(nothing selected)");
        ImGui::End();
        return;
    }

    if (m_Context->selectedEntities.size() > 1) {
        auto& mreg = m_Context->ActiveScene->GetRegistry();
        std::vector<entt::entity> ents;
        for (auto e : m_Context->selectedEntities)
            if (mreg.valid(e)) ents.push_back(e);
        std::sort(ents.begin(), ents.end());   // stable display order across frames
        if (ents.size() > 1) {
            DrawMultiInspector(ents);
            ImGui::End();
            return;
        }
    }

    entt::entity entity = m_Context->PrimarySelection();
    if (!m_Context->ActiveScene->GetRegistry().valid(entity)) {
        m_Context->ClearSelection();
        ImGui::Text("(nothing selected)");
        ImGui::End();
        return;
    }
    auto&        registry = m_Context->ActiveScene->GetRegistry();
    Scene*       scene    = m_Context->ActiveScene;

    // Entity name — click to rename inline
    const std::string& name = m_Context->ActiveScene->GetEntityName(entity);

    // Reset rename state when selection changes
    static entt::entity lastEntity = entt::null;
    if (entity != lastEntity) {
        m_Renaming       = false;
        m_RenameFocusSet = false;
        lastEntity       = entity;
    }

    if (m_Renaming) {
        if (!m_RenameFocusSet) {
            ImGui::SetKeyboardFocusHere();
            m_RenameFocusSet = true;
        }
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::InputText("##ename", m_RenameBuffer, sizeof(m_RenameBuffer),
                             ImGuiInputTextFlags_EnterReturnsTrue))
        {
            if (m_RenameBuffer[0] != '\0')
                m_Context->ActiveScene->SetEntityName(entity, m_RenameBuffer);
            m_Renaming = false;
        }
        else if (!ImGui::IsItemActive() && ImGui::IsItemDeactivated())
        {
            if (m_RenameBuffer[0] != '\0')
                m_Context->ActiveScene->SetEntityName(entity, m_RenameBuffer);
            m_Renaming = false;
        }
        else if (ImGui::IsKeyPressed(ImGuiKey_Escape))
        {
            m_Renaming = false;
        }
    } else {
        ImGui::Text("%s", name.c_str());
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            m_Renaming       = true;
            m_RenameFocusSet = false;
            std::strncpy(m_RenameBuffer, name.c_str(), sizeof(m_RenameBuffer) - 1);
            m_RenameBuffer[sizeof(m_RenameBuffer) - 1] = '\0';
        }
    }

    ImGui::Separator();

    // Transform
    if (registry.all_of<TransformComponent>(entity)) {
        auto& tc = registry.get<TransformComponent>(entity);

        ImGui::Text("Transform");

        ImGui::DragFloat3("Position", glm::value_ptr(tc.position), 0.1f);
        if (ImGui::IsItemActivated())
            m_OldPosition = tc.position;
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            glm::vec3 newVal = tc.position;
            m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<glm::vec3>>(
                [scene, entity](const glm::vec3& v) {
                    scene->GetRegistry().get<TransformComponent>(entity).position = v;
                },
                m_OldPosition, newVal, "Move Entity"));
        }

        if (ImGui::DragFloat3("Rotation", glm::value_ptr(tc.eulerDegrees), 0.5f))
            tc.rotation = glm::quat(glm::radians(tc.eulerDegrees));
        if (ImGui::IsItemActivated())
            m_OldEulerDegrees = tc.eulerDegrees;
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            glm::vec3 newEuler = tc.eulerDegrees;
            m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<glm::vec3>>(
                [scene, entity](const glm::vec3& v) {
                    auto& t = scene->GetRegistry().get<TransformComponent>(entity);
                    t.eulerDegrees = v;
                    t.rotation     = glm::quat(glm::radians(v));
                },
                m_OldEulerDegrees, newEuler, "Rotate Entity"));
        }

        ImGui::DragFloat3("Scale", glm::value_ptr(tc.scale), 0.1f, 0.001f);
        if (ImGui::IsItemActivated())
            m_OldScale = tc.scale;
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            glm::vec3 newScale = tc.scale;
            m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<glm::vec3>>(
                [scene, entity](const glm::vec3& v) {
                    scene->GetRegistry().get<TransformComponent>(entity).scale = v;
                },
                m_OldScale, newScale, "Scale Entity"));
        }
    }

    bool removeMesh        = false;
    bool removeLight       = false;
    bool removeCamera      = false;
    bool removeCollider    = false;
    bool removeRigidbody   = false;
    bool removeConstraint  = false;
    bool removeSkinnedMesh = false;
    bool removeAnimator    = false;
    bool removeAnimSM      = false;
    bool removeCanvas      = false;  CanvasComponent          savedCanvas;
    bool removeRect        = false;  RectTransformComponent   savedRect;
    bool removeUIImage     = false;  UIImageComponent         savedUIImage;
    bool removeUIText      = false;  UITextComponent          savedUIText;
    bool removeUIProgress  = false;  UIProgressBarComponent   savedUIProgress;
    bool removeUIButton    = false;  UIButtonComponent        savedUIButton;

    // Pre-edit stash for the single UI fields below. ImGui has one active widget
    // at a time, so a shared stash per type is safe.
    static glm::vec2   s_uiOldVec2(0.0f);
    static float       s_uiOldFloat = 0.0f;
    static glm::vec4   s_uiOldVec4(0.0f);
    static std::string s_uiOldStr;

    // Mesh Component
    if (registry.all_of<MeshComponent>(entity)) {
        ImGui::Separator();
        auto& mc = registry.get<MeshComponent>(entity);

        ImGui::Text("Mesh Renderer");
        if (ImGui::BeginPopupContextItem("##MeshCompCtx")) {
            if (ImGui::MenuItem("Remove Component")) {
                m_PendingRemovedMesh = mc;
                removeMesh = true;
            }
            ImGui::EndPopup();
        }

        if (!removeMesh) {
            if (ImGui::Checkbox("Visible", &mc.visible)) {
                bool v = mc.visible;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<bool>>(
                    [scene, entity](const bool& x) { scene->GetRegistry().get<MeshComponent>(entity).visible = x; },
                    !v, v, "Toggle Visibility"));
            }
            ImGui::SameLine();
            if (ImGui::Checkbox("Cast Shadows", &mc.castsShadow)) {
                bool v = mc.castsShadow;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<bool>>(
                    [scene, entity](const bool& x) { scene->GetRegistry().get<MeshComponent>(entity).castsShadow = x; },
                    !v, v, "Toggle Cast Shadows"));
            }
            ImGui::SameLine();
            if (ImGui::Checkbox("Recv Shadows", &mc.receivesShadow)) {
                bool v = mc.receivesShadow;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<bool>>(
                    [scene, entity](const bool& x) { scene->GetRegistry().get<MeshComponent>(entity).receivesShadow = x; },
                    !v, v, "Toggle Receive Shadows"));
            }
            ImGui::SameLine();
            if (ImGui::Checkbox("Transparent", &mc.transparent)) {
                bool v = mc.transparent;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<bool>>(
                    [scene, entity](const bool& x) { scene->GetRegistry().get<MeshComponent>(entity).transparent = x; },
                    !v, v, "Toggle Transparent"));
            }

            // -- Mesh slot --
            ImGui::Spacing();
            ImGui::TextDisabled("Mesh");
            ImGui::Spacing();
            if (auto r = DrawMeshSlot(mc, m_ContentPanel); r.changed) {
                auto om = r.oldMesh, nm = mc.mesh;
                auto ob = r.oldBounds, nb = mc.localBounds;
                auto op = r.oldMeshPath, np = mc.meshPath;
                auto oi = r.oldSubIndex, ni = mc.meshSubIndex;
                auto omat = r.oldMaterial, nmat = mc.material;
                m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                    [scene, entity, nm, nb, np, ni, nmat]() {
                        auto& c = scene->GetRegistry().get<MeshComponent>(entity);
                        c.mesh = nm; c.localBounds = nb; c.meshPath = np; c.meshSubIndex = ni; c.material = nmat;
                    },
                    [scene, entity, om, ob, op, oi, omat]() {
                        auto& c = scene->GetRegistry().get<MeshComponent>(entity);
                        c.mesh = om; c.localBounds = ob; c.meshPath = op; c.meshSubIndex = oi; c.material = omat;
                    },
                    "Change Mesh"));
            }

            // -- Primitive shapes -- assign a built-in cube/sphere (the "__cube"
            // / "__sphere" meshes the serializer rebuilds on load). Material is
            // left untouched. One undo command captures the swap.
            auto assignPrimitive = [&](const std::string& primPath, MeshData md) {
                auto om = mc.mesh;       auto ob = mc.localBounds;
                auto op = mc.meshPath;   auto oi = mc.meshSubIndex;
                auto nm = Mesh::Create(md);
                auto nb = md.ComputeAABB();
                mc.mesh = nm; mc.localBounds = nb; mc.meshPath = primPath; mc.meshSubIndex = 0;
                m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                    [scene, entity, nm, nb, primPath]() {
                        auto& c = scene->GetRegistry().get<MeshComponent>(entity);
                        c.mesh = nm; c.localBounds = nb; c.meshPath = primPath; c.meshSubIndex = 0;
                    },
                    [scene, entity, om, ob, op, oi]() {
                        auto& c = scene->GetRegistry().get<MeshComponent>(entity);
                        c.mesh = om; c.localBounds = ob; c.meshPath = op; c.meshSubIndex = oi;
                    },
                    "Assign Primitive Mesh"));
            };
            ImGui::Spacing();
            ImGui::TextDisabled("Primitive:");
            ImGui::SameLine();
            if (ImGui::SmallButton("Cube"))   assignPrimitive("__cube",   MeshData::UnitCube());
            ImGui::SameLine();
            if (ImGui::SmallButton("Sphere")) assignPrimitive("__sphere", MeshData::UVSphere());

            // -- Material --
            ImGui::Spacing();
            ImGui::TextDisabled("Material");

            if (!mc.material)
                mc.material = std::make_shared<PBRMaterial>();

            // Asset slot: drag a .mat to share a reusable material across meshes
            // (edits write through + save to the file). "Save as .mat" bottles the
            // current inline material; "Detach" makes a private inline copy again.
            bool assetBacked = !mc.materialPath.empty();
            {
                std::string disp = assetBacked
                    ? std::filesystem::path(mc.materialPath).filename().string()
                    : "None (inline)";
                ImGui::Button(disp.c_str(), { ImGui::GetContentRegionAvail().x, 0 });
                if (ImGui::BeginDragDropTarget()) {
                    if (const ImGuiPayload* p = ImGui::AcceptDragDropPayload("CONTENT_ITEM_PATH")) {
                        std::string path((const char*)p->Data);
                        if (LowerExtOf(path) == ".mat") {
                            std::string norm = NormalizeMaterialPath(path);
                            if (auto shared = MaterialLibrary::Get(norm)) {
                                mc.materialPath = norm;
                                mc.material     = shared;   // asset replaces the inline material
                                assetBacked     = true;
                            }
                        }
                    }
                    ImGui::EndDragDropTarget();
                }

                if (assetBacked) {
                    if (ImGui::SmallButton("Detach to inline")) {
                        mc.material = std::make_shared<PBRMaterial>(*mc.material); // private copy
                        mc.materialPath.clear();
                        assetBacked = false;
                    }
                } else if (ImGui::SmallButton("Save as .mat")) {
                    namespace fs = std::filesystem;
                    fs::path dir  = m_ContentPanel ? m_ContentPanel->GetCurrentPath() : fs::path(ASSETS_DIR);
                    fs::path base = dir / "NewMaterial";
                    fs::path p = base; p += ".mat";
                    for (int n = 1; fs::exists(p); ++n) { p = base; p += " (" + std::to_string(n) + ").mat"; }
                    std::string np = p.make_preferred().string();
                    if (SaveMaterialAsset(np, *mc.material)) {
                        mc.materialPath = np;
                        mc.material     = MaterialLibrary::Get(np);   // share the new asset
                        assetBacked     = true;
                        if (m_ContentPanel) m_ContentPanel->Refresh();
                    }
                }
            }

            // For asset-backed materials, edits write through to the shared instance
            // (updating every mesh using it) and we re-save the .mat once on release.
            // Inline materials keep the undo-tracked, saved-with-scene behavior.
            bool matDirty = false;

            ImGui::DragFloat("UV Scale", &mc.material->UVScale, 0.01f, 0.01f, 64.0f);
            if (assetBacked) {
                if (ImGui::IsItemDeactivatedAfterEdit()) matDirty = true;
            } else {
                if (ImGui::IsItemActivated())   m_OldUVScale = mc.material->UVScale;
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float newVal = mc.material->UVScale, oldVal = m_OldUVScale;
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity, newVal]() { scene->GetRegistry().get<MeshComponent>(entity).material->UVScale = newVal; },
                        [scene, entity, oldVal]() { scene->GetRegistry().get<MeshComponent>(entity).material->UVScale = oldVal; },
                        "Change UV Scale"));
                }
            }

            ImGui::DragFloat("Emissive Strength", &mc.material->EmissiveStrength, 0.01f, 0.0f, 100.0f);
            if (assetBacked) {
                if (ImGui::IsItemDeactivatedAfterEdit()) matDirty = true;
            } else {
                if (ImGui::IsItemActivated())   m_OldEmissiveStrength = mc.material->EmissiveStrength;
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float newVal = mc.material->EmissiveStrength, oldVal = m_OldEmissiveStrength;
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity, newVal]() { scene->GetRegistry().get<MeshComponent>(entity).material->EmissiveStrength = newVal; },
                        [scene, entity, oldVal]() { scene->GetRegistry().get<MeshComponent>(entity).material->EmissiveStrength = oldVal; },
                        "Change Emissive Strength"));
                }
            }

            ImGui::Spacing();
            ImGui::TextDisabled("Textures");
            ImGui::Separator();

            // Helper: draw a texture row. For asset materials the swap is already
            // applied to the shared instance (just mark dirty); for inline materials
            // record an undo command. Pointer-to-member keeps all 6 slots one body.
            using TexMem  = std::shared_ptr<Texture> PBRMaterial::*;
            using PathMem = std::string              PBRMaterial::*;
            auto texRow = [&](const char* label,
                               std::shared_ptr<Texture>& texPtr, std::string& texPath,
                               TexMem tm, PathMem pm, const char* desc) {
                auto r = DrawTextureRow(label, texPtr, texPath, m_ContentPanel);
                if (!r.changed) return;
                if (assetBacked) { matDirty = true; return; }
                auto ot = r.oldTex, nt = texPtr;
                auto op = r.oldPath, np = texPath;
                m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                    [scene, entity, nt, np, tm, pm]() {
                        auto& mat = *scene->GetRegistry().get<MeshComponent>(entity).material;
                        mat.*tm = nt; mat.*pm = np;
                    },
                    [scene, entity, ot, op, tm, pm]() {
                        auto& mat = *scene->GetRegistry().get<MeshComponent>(entity).material;
                        mat.*tm = ot; mat.*pm = op;
                    },
                    desc));
            };

            texRow("Albedo",    mc.material->Albedo,    mc.material->AlbedoPath,    &PBRMaterial::Albedo,    &PBRMaterial::AlbedoPath,    "Change Albedo Texture");
            texRow("Normal",    mc.material->Normal,    mc.material->NormalPath,    &PBRMaterial::Normal,    &PBRMaterial::NormalPath,    "Change Normal Texture");
            texRow("Metallic",  mc.material->Metallic,  mc.material->MetallicPath,  &PBRMaterial::Metallic,  &PBRMaterial::MetallicPath,  "Change Metallic Texture");
            texRow("Roughness", mc.material->Roughness, mc.material->RoughnessPath, &PBRMaterial::Roughness, &PBRMaterial::RoughnessPath, "Change Roughness Texture");
            texRow("AO",        mc.material->AO,        mc.material->AOPath,        &PBRMaterial::AO,        &PBRMaterial::AOPath,        "Change AO Texture");
            texRow("Emissive",  mc.material->Emissive,  mc.material->EmissivePath,  &PBRMaterial::Emissive,  &PBRMaterial::EmissivePath,  "Change Emissive Texture");

            // Persist edits to the .mat once per release (asset-backed only) and
            // refresh its content-browser preview.
            if (assetBacked && matDirty) {
                SaveMaterialAsset(mc.materialPath, *mc.material);
                if (m_ContentPanel) m_ContentPanel->InvalidateThumbnail(mc.materialPath);
            }
        }
    }

    // Light Component
    if (registry.all_of<LightComponent>(entity)) {
        ImGui::Separator();
        auto& lc = registry.get<LightComponent>(entity);

        ImGui::Text("Light");
        if (ImGui::BeginPopupContextItem("##LightCompCtx")) {
            if (ImGui::MenuItem("Remove Component")) {
                m_PendingRemovedLight = lc;
                removeLight = true;
            }
            ImGui::EndPopup();
        }

        if (!removeLight) {
            const char* types[] = { "Sun", "Point", "Spot" };
            LightType oldType = lc.type;
            int typeIdx = (int)lc.type;
            if (ImGui::Combo("Type", &typeIdx, types, 3)) {
                lc.type = (LightType)typeIdx;
                LightType newType = lc.type;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<LightType>>(
                    [scene, entity](const LightType& t) { scene->GetRegistry().get<LightComponent>(entity).type = t; },
                    oldType, newType, "Change Light Type"));
            }

            ImGui::ColorEdit3("Color", glm::value_ptr(lc.color));
            if (ImGui::IsItemActivated())          m_OldLightColor = lc.color;
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                glm::vec3 n = lc.color, o = m_OldLightColor;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<glm::vec3>>(
                    [scene, entity](const glm::vec3& v) { scene->GetRegistry().get<LightComponent>(entity).color = v; },
                    o, n, "Change Light Color"));
            }

            ImGui::DragFloat("Intensity", &lc.intensity, 1.0f, 0.0f, 10000.0f, "%.1f");
            if (ImGui::IsItemActivated())          m_OldIntensity = lc.intensity;
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                float n = lc.intensity, o = m_OldIntensity;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                    [scene, entity](const float& v) { scene->GetRegistry().get<LightComponent>(entity).intensity = v; },
                    o, n, "Change Light Intensity"));
            }

            if (lc.type == LightType::Point || lc.type == LightType::Spot) {
                ImGui::DragFloat("Radius", &lc.radius, 0.1f, 0.1f, 1000.0f, "%.1f");
                if (ImGui::IsItemActivated())          m_OldRadius = lc.radius;
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float n = lc.radius, o = m_OldRadius;
                    m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                        [scene, entity](const float& v) { scene->GetRegistry().get<LightComponent>(entity).radius = v; },
                        o, n, "Change Light Radius"));
                }
            }

            if (lc.type == LightType::Spot) {
                ImGui::DragFloat("Inner Cone", &lc.innerConeAngle, 0.5f, 0.5f, 89.0f, "%.1f deg");
                if (ImGui::IsItemActivated()) { m_OldInnerCone = lc.innerConeAngle; m_OldOuterCone = lc.outerConeAngle; }
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float ni = lc.innerConeAngle, no = lc.outerConeAngle, oi = m_OldInnerCone, oo = m_OldOuterCone;
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity, ni, no]() { auto& l = scene->GetRegistry().get<LightComponent>(entity); l.innerConeAngle = ni; l.outerConeAngle = no; },
                        [scene, entity, oi, oo]() { auto& l = scene->GetRegistry().get<LightComponent>(entity); l.innerConeAngle = oi; l.outerConeAngle = oo; },
                        "Change Inner Cone"));
                }
                lc.outerConeAngle = std::max(lc.outerConeAngle, lc.innerConeAngle + 0.5f);

                ImGui::DragFloat("Outer Cone", &lc.outerConeAngle, 0.5f, 1.0f, 90.0f, "%.1f deg");
                if (ImGui::IsItemActivated()) { m_OldInnerCone = lc.innerConeAngle; m_OldOuterCone = lc.outerConeAngle; }
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float ni = lc.innerConeAngle, no = lc.outerConeAngle, oi = m_OldInnerCone, oo = m_OldOuterCone;
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity, ni, no]() { auto& l = scene->GetRegistry().get<LightComponent>(entity); l.innerConeAngle = ni; l.outerConeAngle = no; },
                        [scene, entity, oi, oo]() { auto& l = scene->GetRegistry().get<LightComponent>(entity); l.innerConeAngle = oi; l.outerConeAngle = oo; },
                        "Change Outer Cone"));
                }
                lc.innerConeAngle = std::min(lc.innerConeAngle, lc.outerConeAngle - 0.5f);
            }
        }
    }

    // Camera Component
    if (registry.all_of<CameraComponent>(entity)) {
        ImGui::Separator();
        auto& cc = registry.get<CameraComponent>(entity);

        ImGui::Text("Camera");
        if (ImGui::BeginPopupContextItem("##CameraCompCtx")) {
            if (ImGui::MenuItem("Remove Component")) {
                m_PendingRemovedCamera = cc;
                removeCamera = true;
            }
            ImGui::EndPopup();
        }

        if (!removeCamera) {
            if (ImGui::Checkbox("Primary Camera", &cc.isPrimary)) {
                bool v = cc.isPrimary;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<bool>>(
                    [scene, entity](const bool& x) { scene->GetRegistry().get<CameraComponent>(entity).isPrimary = x; },
                    !v, v, "Toggle Primary Camera"));
            }

            ImGui::DragFloat("FOV", &cc.fov, 0.5f, 1.0f, 179.0f, "%.1f deg");
            if (ImGui::IsItemActivated())           m_OldFov = cc.fov;
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                float n = cc.fov, o = m_OldFov;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                    [scene, entity](const float& v) { scene->GetRegistry().get<CameraComponent>(entity).fov = v; },
                    o, n, "Change Camera FOV"));
            }

            ImGui::DragFloat("Near Clip", &cc.nearClip, 0.01f, 0.001f, cc.farClip - 0.01f, "%.3f");
            if (ImGui::IsItemActivated())           m_OldNear = cc.nearClip;
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                float n = cc.nearClip, o = m_OldNear;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                    [scene, entity](const float& v) { scene->GetRegistry().get<CameraComponent>(entity).nearClip = v; },
                    o, n, "Change Near Clip"));
            }

            ImGui::DragFloat("Far Clip", &cc.farClip, 1.0f, cc.nearClip + 0.01f, 100000.0f, "%.1f");
            if (ImGui::IsItemActivated())           m_OldFar = cc.farClip;
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                float n = cc.farClip, o = m_OldFar;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                    [scene, entity](const float& v) { scene->GetRegistry().get<CameraComponent>(entity).farClip = v; },
                    o, n, "Change Far Clip"));
            }
        }
    }

    // Canvas Component
    if (registry.all_of<CanvasComponent>(entity)) {
        ImGui::Separator();
        auto& cv = registry.get<CanvasComponent>(entity);

        ImGui::Text("Canvas");
        if (ImGui::BeginPopupContextItem("##CanvasCompCtx")) {
            if (ImGui::MenuItem("Remove Component")) { savedCanvas = cv; removeCanvas = true; }
            ImGui::EndPopup();
        }

        if (!removeCanvas) {
            const char* modes[] = { "Constant Pixel Size", "Scale With Screen Size" };
            int modeIdx = (int)cv.scaleMode;
            if (ImGui::Combo("Scale Mode", &modeIdx, modes, 2)) {
                auto oldMode = cv.scaleMode;
                auto newMode = (CanvasComponent::ScaleMode)modeIdx;
                cv.scaleMode = newMode;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<CanvasComponent::ScaleMode>>(
                    [scene, entity](const CanvasComponent::ScaleMode& m) { scene->GetRegistry().get<CanvasComponent>(entity).scaleMode = m; },
                    oldMode, newMode, "Change Canvas Scale Mode"));
            }

            if (cv.scaleMode == CanvasComponent::ScaleMode::ScaleWithScreenSize) {
                ImGui::DragFloat2("Reference Resolution", glm::value_ptr(cv.referenceResolution), 1.0f, 1.0f, 16384.0f, "%.0f");
                if (ImGui::IsItemActivated()) s_uiOldVec2 = cv.referenceResolution;
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    glm::vec2 n = cv.referenceResolution, o = s_uiOldVec2;
                    m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<glm::vec2>>(
                        [scene, entity](const glm::vec2& v) { scene->GetRegistry().get<CanvasComponent>(entity).referenceResolution = v; },
                        o, n, "Change Reference Resolution"));
                }

                ImGui::SliderFloat("Match W/H", &cv.matchWidthHeight, 0.0f, 1.0f, "%.2f");
                if (ImGui::IsItemActivated()) s_uiOldFloat = cv.matchWidthHeight;
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float n = cv.matchWidthHeight, o = s_uiOldFloat;
                    m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                        [scene, entity](const float& v) { scene->GetRegistry().get<CanvasComponent>(entity).matchWidthHeight = v; },
                        o, n, "Change Canvas Match"));
                }
            }

            int sort = cv.sortOrder;
            if (ImGui::InputInt("Sort Order", &sort)) cv.sortOrder = sort;
        }
    }

    // Rect Transform Component
    if (registry.all_of<RectTransformComponent>(entity)) {
        ImGui::Separator();
        auto& rt = registry.get<RectTransformComponent>(entity);

        ImGui::Text("Rect Transform");
        if (ImGui::BeginPopupContextItem("##RectCompCtx")) {
            if (ImGui::MenuItem("Remove Component")) { savedRect = rt; removeRect = true; }
            ImGui::EndPopup();
        }

        if (!removeRect) {
            // Scope these widgets: "Position" would otherwise collide with the 3D
            // TransformComponent's "Position" field on the same entity.
            ImGui::PushID("RectTransform");

            // DragFloat2 + per-drag undo for one RectTransform vec2 field.
            auto rectVec2 = [&](const char* label, glm::vec2 RectTransformComponent::* mem,
                                float speed, const char* fmt, const char* desc) {
                auto& r = registry.get<RectTransformComponent>(entity);
                ImGui::DragFloat2(label, glm::value_ptr(r.*mem), speed, 0.0f, 0.0f, fmt);
                if (ImGui::IsItemActivated()) s_uiOldVec2 = r.*mem;
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    glm::vec2 n = r.*mem, o = s_uiOldVec2;
                    m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<glm::vec2>>(
                        [scene, entity, mem](const glm::vec2& v) { scene->GetRegistry().get<RectTransformComponent>(entity).*mem = v; },
                        o, n, desc));
                }
            };

            rectVec2("Anchor Min", &RectTransformComponent::anchorMin, 0.01f, "%.2f", "Change Anchor Min");
            rectVec2("Anchor Max", &RectTransformComponent::anchorMax, 0.01f, "%.2f", "Change Anchor Max");
            rectVec2("Pivot",      &RectTransformComponent::pivot,     0.01f, "%.2f", "Change Pivot");
            rectVec2("Position",   &RectTransformComponent::position,  1.0f,  "%.0f", "Change UI Position");
            rectVec2("Size",       &RectTransformComponent::size,      1.0f,  "%.0f", "Change UI Size");

            int z = rt.zOrder;
            if (ImGui::InputInt("Z Order", &z)) rt.zOrder = z;

            ImGui::TextDisabled("Resolved: %.0f, %.0f   %.0f x %.0f",
                                rt.resolvedPos.x, rt.resolvedPos.y,
                                rt.resolvedSize.x, rt.resolvedSize.y);

            ImGui::PopID();
        }
    }

    // ---- UI widget components -------------------------------------------------
    // Generic undo-recording field editors (write the live field, record one
    // command on commit). Setters re-fetch the component from the registry so the
    // command survives entity-pointer churn, matching the rest of this panel.
    auto uiColorEdit = [&](const char* label, glm::vec4& ref,
                           std::function<void(const glm::vec4&)> setter, const char* desc) {
        ImGui::ColorEdit4(label, glm::value_ptr(ref), ImGuiColorEditFlags_AlphaBar);
        if (ImGui::IsItemActivated()) s_uiOldVec4 = ref;
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            glm::vec4 n = ref, o = s_uiOldVec4;
            m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<glm::vec4>>(
                std::move(setter), o, n, desc));
        }
    };
    auto uiFloatEdit = [&](const char* label, float& ref, float speed, float lo, float hi,
                           const char* fmt, std::function<void(const float&)> setter, const char* desc) {
        ImGui::DragFloat(label, &ref, speed, lo, hi, fmt);
        if (ImGui::IsItemActivated()) s_uiOldFloat = ref;
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            float n = ref, o = s_uiOldFloat;
            m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                std::move(setter), o, n, desc));
        }
    };
    auto uiBoolEdit = [&](const char* label, bool& ref,
                          std::function<void(const bool&)> setter, const char* desc) {
        if (ImGui::Checkbox(label, &ref)) {
            bool n = ref;
            m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<bool>>(
                std::move(setter), !n, n, desc));
        }
    };
    auto uiComboEdit = [&](const char* label, int current, const char* const* items, int count,
                           std::function<void(const int&)> setter, const char* desc) {
        int idx = current;
        if (ImGui::Combo(label, &idx, items, count) && idx != current)
            m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<int>>(
                std::move(setter), current, idx, desc));
    };
    auto uiTextEdit = [&](const char* label, std::string& ref,
                          std::function<void(const std::string&)> setter, const char* desc) {
        char buf[512];
        std::strncpy(buf, ref.c_str(), sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        if (ImGui::InputText(label, buf, sizeof(buf))) ref = buf;
        if (ImGui::IsItemActivated()) s_uiOldStr = ref;
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            std::string n = ref, o = s_uiOldStr;
            m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<std::string>>(
                std::move(setter), o, n, desc));
        }
    };
    // Font asset slot: drag a .ttf/.otf from the content browser.
    auto uiFontSlot = [&](UITextComponent& tx) {
        std::string name = tx.fontPath.empty() ? "None" : Basename(tx.fontPath);
        ImGui::TextDisabled("Font"); ImGui::SameLine();
        ImGui::Button((name + "##fontslot").c_str(), { -1.0f, 0.0f });
        if (ImGui::BeginDragDropTarget()) {
            if (auto* p = ImGui::AcceptDragDropPayload("CONTENT_ITEM_PATH")) {
                std::string path((const char*)p->Data);
                std::string ext = LowerExtOf(path);
                if (ext == ".ttf" || ext == ".otf") {
                    auto of = tx.font; std::string op = tx.fontPath;
                    tx.fontPath = path;
                    tx.font     = Font::Create(path, kUIFontBakeHeight);
                    auto nf = tx.font; std::string np = tx.fontPath;
                    m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                        [scene, entity, nf, np]() { auto& c = scene->GetRegistry().get<UITextComponent>(entity); c.font = nf; c.fontPath = np; },
                        [scene, entity, of, op]() { auto& c = scene->GetRegistry().get<UITextComponent>(entity); c.font = of; c.fontPath = op; },
                        "Change Text Font"));
                }
            }
            ImGui::EndDragDropTarget();
        }
    };

    // Reusable component bodies — the Button section composes these so its
    // background image + label are edited inline under one "Button" header.
    auto drawUIImageBody = [&](UIImageComponent& im) {
        auto r = DrawTextureRow("Texture", im.texture, im.texturePath, m_ContentPanel);
        if (r.changed) {
            auto nt = im.texture, ot = r.oldTex;
            std::string np = im.texturePath, op = r.oldPath;
            m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                [scene, entity, nt, np]() { auto& c = scene->GetRegistry().get<UIImageComponent>(entity); c.texture = nt; c.texturePath = np; },
                [scene, entity, ot, op]() { auto& c = scene->GetRegistry().get<UIImageComponent>(entity); c.texture = ot; c.texturePath = op; },
                "Change Image Texture"));
        }
        uiColorEdit("Tint", im.tint,
            [scene, entity](const glm::vec4& v) { scene->GetRegistry().get<UIImageComponent>(entity).tint = v; },
            "Change Image Tint");
    };
    auto drawUITextBody = [&](UITextComponent& tx) {
        uiTextEdit("Text", tx.text,
            [scene, entity](const std::string& v) { scene->GetRegistry().get<UITextComponent>(entity).text = v; },
            "Change Text");
        uiFontSlot(tx);
        uiFloatEdit("Size", tx.sizePx, 0.5f, 1.0f, 512.0f, "%.0f",
            [scene, entity](const float& v) { scene->GetRegistry().get<UITextComponent>(entity).sizePx = v; },
            "Change Text Size");
        uiColorEdit("Color", tx.color,
            [scene, entity](const glm::vec4& v) { scene->GetRegistry().get<UITextComponent>(entity).color = v; },
            "Change Text Color");
        const char* hItems[] = { "Left", "Center", "Right" };
        uiComboEdit("H Align", (int)tx.hAlign, hItems, 3,
            [scene, entity](const int& v) { scene->GetRegistry().get<UITextComponent>(entity).hAlign = (UITextComponent::HAlign)v; },
            "Change Text H Align");
        const char* vItems[] = { "Top", "Middle", "Bottom" };
        uiComboEdit("V Align", (int)tx.vAlign, vItems, 3,
            [scene, entity](const int& v) { scene->GetRegistry().get<UITextComponent>(entity).vAlign = (UITextComponent::VAlign)v; },
            "Change Text V Align");
        uiFloatEdit("Line Spacing", tx.lineSpacing, 0.01f, 0.1f, 4.0f, "%.2f",
            [scene, entity](const float& v) { scene->GetRegistry().get<UITextComponent>(entity).lineSpacing = v; },
            "Change Line Spacing");
        uiBoolEdit("Wrap", tx.wrap,
            [scene, entity](const bool& v) { scene->GetRegistry().get<UITextComponent>(entity).wrap = v; },
            "Toggle Text Wrap");
        uiBoolEdit("Underline", tx.underline,
            [scene, entity](const bool& v) { scene->GetRegistry().get<UITextComponent>(entity).underline = v; },
            "Toggle Underline");
    };

    // UI Image (standalone; folded into the Button section when one is present)
    if (registry.all_of<UIImageComponent>(entity) && !registry.all_of<UIButtonComponent>(entity)) {
        ImGui::Separator();
        ImGui::Text("UI Image");
        if (ImGui::BeginPopupContextItem("##UIImageCtx")) {
            if (ImGui::MenuItem("Remove Component")) { savedUIImage = registry.get<UIImageComponent>(entity); removeUIImage = true; }
            ImGui::EndPopup();
        }
        if (!removeUIImage) {
            ImGui::PushID("UIImage");
            drawUIImageBody(registry.get<UIImageComponent>(entity));
            ImGui::PopID();
        }
    }

    // UI Text (standalone; folded into the Button section when one is present)
    if (registry.all_of<UITextComponent>(entity) && !registry.all_of<UIButtonComponent>(entity)) {
        ImGui::Separator();
        ImGui::Text("UI Text");
        if (ImGui::BeginPopupContextItem("##UITextCtx")) {
            if (ImGui::MenuItem("Remove Component")) { savedUIText = registry.get<UITextComponent>(entity); removeUIText = true; }
            ImGui::EndPopup();
        }
        if (!removeUIText) {
            ImGui::PushID("UIText");
            drawUITextBody(registry.get<UITextComponent>(entity));
            ImGui::PopID();
        }
    }

    // UI Progress Bar
    if (registry.all_of<UIProgressBarComponent>(entity)) {
        ImGui::Separator();
        ImGui::Text("UI Progress Bar");
        if (ImGui::BeginPopupContextItem("##UIProgressCtx")) {
            if (ImGui::MenuItem("Remove Component")) { savedUIProgress = registry.get<UIProgressBarComponent>(entity); removeUIProgress = true; }
            ImGui::EndPopup();
        }
        if (!removeUIProgress) {
            ImGui::PushID("UIProgress");
            auto& pb = registry.get<UIProgressBarComponent>(entity);
            uiFloatEdit("Progress", pb.progress, 0.005f, 0.0f, 1.0f, "%.2f",
                [scene, entity](const float& v) { scene->GetRegistry().get<UIProgressBarComponent>(entity).progress = v; },
                "Change Progress");
            uiColorEdit("Background", pb.backgroundColor,
                [scene, entity](const glm::vec4& v) { scene->GetRegistry().get<UIProgressBarComponent>(entity).backgroundColor = v; },
                "Change Bar Background");
            uiColorEdit("Fill", pb.fillColor,
                [scene, entity](const glm::vec4& v) { scene->GetRegistry().get<UIProgressBarComponent>(entity).fillColor = v; },
                "Change Bar Fill");
            auto r = DrawTextureRow("Fill Tex", pb.fillTexture, pb.fillTexturePath, m_ContentPanel);
            if (r.changed) {
                auto nt = pb.fillTexture, ot = r.oldTex;
                std::string np = pb.fillTexturePath, op = r.oldPath;
                m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                    [scene, entity, nt, np]() { auto& c = scene->GetRegistry().get<UIProgressBarComponent>(entity); c.fillTexture = nt; c.fillTexturePath = np; },
                    [scene, entity, ot, op]() { auto& c = scene->GetRegistry().get<UIProgressBarComponent>(entity); c.fillTexture = ot; c.fillTexturePath = op; },
                    "Change Bar Fill Texture"));
            }
            const char* dItems[] = { "Left To Right", "Right To Left", "Bottom To Top", "Top To Bottom" };
            uiComboEdit("Direction", (int)pb.direction, dItems, 4,
                [scene, entity](const int& v) { scene->GetRegistry().get<UIProgressBarComponent>(entity).direction = (UIProgressBarComponent::Direction)v; },
                "Change Bar Direction");
            ImGui::PopID();
        }
    }

    // Button — composed widget: shows its background image + label + interaction
    // tints under one header, per design. onClick is wired from scripts.
    if (registry.all_of<UIButtonComponent>(entity)) {
        ImGui::Separator();
        ImGui::Text("Button");
        if (ImGui::BeginPopupContextItem("##UIButtonCtx")) {
            if (ImGui::MenuItem("Remove Component")) { savedUIButton = registry.get<UIButtonComponent>(entity); removeUIButton = true; }
            ImGui::EndPopup();
        }
        if (!removeUIButton) {
            ImGui::PushID("UIButton");
            if (registry.all_of<UIImageComponent>(entity)) {
                ImGui::SeparatorText("Background");
                drawUIImageBody(registry.get<UIImageComponent>(entity));
            }
            if (registry.all_of<UITextComponent>(entity)) {
                ImGui::SeparatorText("Label");
                drawUITextBody(registry.get<UITextComponent>(entity));
            }
            ImGui::SeparatorText("Interaction");
            auto& bt = registry.get<UIButtonComponent>(entity);
            uiColorEdit("Normal Tint", bt.normalTint,
                [scene, entity](const glm::vec4& v) { scene->GetRegistry().get<UIButtonComponent>(entity).normalTint = v; },
                "Change Button Normal Tint");
            uiColorEdit("Hover Tint", bt.hoverTint,
                [scene, entity](const glm::vec4& v) { scene->GetRegistry().get<UIButtonComponent>(entity).hoverTint = v; },
                "Change Button Hover Tint");
            uiColorEdit("Pressed Tint", bt.pressedTint,
                [scene, entity](const glm::vec4& v) { scene->GetRegistry().get<UIButtonComponent>(entity).pressedTint = v; },
                "Change Button Pressed Tint");
            ImGui::TextDisabled("onClick is assigned in scripts (not serialized).");
            ImGui::PopID();
        }
    }

    // Collider Component
    if (registry.all_of<ColliderComponent>(entity)) {
        ImGui::Separator();
        auto& col = registry.get<ColliderComponent>(entity);

        ImGui::Text("Collider");
        if (ImGui::BeginPopupContextItem("##ColliderCompCtx")) {
            if (ImGui::MenuItem("Remove Component")) {
                m_PendingRemovedCollider = col;
                removeCollider = true;
            }
            ImGui::EndPopup();
        }

        if (!removeCollider) {
            const char* shapes[] = { "Box", "Sphere", "Capsule", "Convex Hull", "Triangle Mesh" };
            CollisionShape oldShape = col.shapeType;
            int shapeIdx = (int)col.shapeType;
            if (ImGui::Combo("Shape", &shapeIdx, shapes, 5)) {
                col.shapeType = (CollisionShape)shapeIdx;
                CollisionShape newShape = col.shapeType;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<CollisionShape>>(
                    [scene, entity](const CollisionShape& s) { scene->GetRegistry().get<ColliderComponent>(entity).shapeType = s; },
                    oldShape, newShape, "Change Collision Shape"));
            }

            switch (col.shapeType) {
                case CollisionShape::Box:
                    ImGui::DragFloat3("Half Extents", glm::value_ptr(col.halfExtents), 0.01f, 0.001f, 1000.0f);
                    if (ImGui::IsItemActivated())            m_OldColHalfExtents = col.halfExtents;
                    if (ImGui::IsItemDeactivatedAfterEdit()) {
                        glm::vec3 n = col.halfExtents, o = m_OldColHalfExtents;
                        m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<glm::vec3>>(
                            [scene, entity](const glm::vec3& v) { scene->GetRegistry().get<ColliderComponent>(entity).halfExtents = v; },
                            o, n, "Change Half Extents"));
                    }
                    break;
                case CollisionShape::Sphere:
                    ImGui::DragFloat("Radius", &col.radius, 0.01f, 0.001f, 1000.0f);
                    if (ImGui::IsItemActivated())            m_OldColRadius = col.radius;
                    if (ImGui::IsItemDeactivatedAfterEdit()) {
                        float n = col.radius, o = m_OldColRadius;
                        m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                            [scene, entity](const float& v) { scene->GetRegistry().get<ColliderComponent>(entity).radius = v; },
                            o, n, "Change Radius"));
                    }
                    break;
                case CollisionShape::Capsule:
                    ImGui::DragFloat("Radius", &col.radius, 0.01f, 0.001f, 1000.0f);
                    if (ImGui::IsItemActivated())            m_OldColRadius = col.radius;
                    if (ImGui::IsItemDeactivatedAfterEdit()) {
                        float n = col.radius, o = m_OldColRadius;
                        m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                            [scene, entity](const float& v) { scene->GetRegistry().get<ColliderComponent>(entity).radius = v; },
                            o, n, "Change Radius"));
                    }
                    ImGui::DragFloat("Half Height", &col.halfHeight, 0.01f, 0.001f, 1000.0f);
                    if (ImGui::IsItemActivated())            m_OldColHalfHeight = col.halfHeight;
                    if (ImGui::IsItemDeactivatedAfterEdit()) {
                        float n = col.halfHeight, o = m_OldColHalfHeight;
                        m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                            [scene, entity](const float& v) { scene->GetRegistry().get<ColliderComponent>(entity).halfHeight = v; },
                            o, n, "Change Half Height"));
                    }
                    break;
                case CollisionShape::ConvexHull:
                case CollisionShape::TriangleMesh:
                    ImGui::TextDisabled("(mesh shapes require asset pipeline — not yet editable)");
                    break;
            }

            ImGui::Spacing();
            ImGui::TextDisabled("Offset");
            ImGui::DragFloat3("Local Offset", glm::value_ptr(col.localOffset), 0.01f);
            if (ImGui::IsItemActivated())            m_OldColOffset = col.localOffset;
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                glm::vec3 n = col.localOffset, o = m_OldColOffset;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<glm::vec3>>(
                    [scene, entity](const glm::vec3& v) { scene->GetRegistry().get<ColliderComponent>(entity).localOffset = v; },
                    o, n, "Change Collider Offset"));
            }

            ImGui::Spacing();
            ImGui::Checkbox("Is Trigger", &col.isTrigger);

            // Collision group: bodies sharing the same non-zero group ignore each
            // other (0 = collide normally). Give every bone of one ragdoll the
            // same group so they don't explode at their joints.
            int grp = (int)col.collisionGroup;
            if (ImGui::InputInt("Collision Group", &grp)) {
                if (grp < 0) grp = 0;
                col.collisionGroup = (uint32_t)grp;
            }

            ImGui::Spacing();
            ImGui::TextDisabled("Physics Material");

            // --- asset slot (drag-drop target) ---
            {
                std::string dispName = col.physMatPath.empty()
                    ? "None"
                    : std::filesystem::path(col.physMatPath).filename().string();

                float avail   = ImGui::GetContentRegionAvail().x;
                float newBtnW = ImGui::CalcTextSize("New").x + ImGui::GetStyle().FramePadding.x * 2.0f;
                float clrBtnW = col.physMatPath.empty()
                    ? 0.0f
                    : ImGui::CalcTextSize("×").x + ImGui::GetStyle().FramePadding.x * 2.0f;
                float spacing = ImGui::GetStyle().ItemSpacing.x;
                float slotW   = avail - newBtnW - spacing
                                - (col.physMatPath.empty() ? 0.0f : clrBtnW + spacing);

                ImGui::Button(dispName.c_str(), {slotW, 0});

                if (ImGui::BeginDragDropTarget()) {
                    if (const ImGuiPayload* p = ImGui::AcceptDragDropPayload("CONTENT_ITEM_PATH")) {
                        std::string newPath(static_cast<const char*>(p->Data));
                        std::string ext = LowerExtOf(newPath);
                        if (ext == ".physmat") {
                            std::string oldPath = col.physMatPath;
                            auto        oldMat  = col.material;
                            col.physMatPath = NormalizePhysMatPath(newPath);
                            col.material    = LoadPhysicsMaterial(col.physMatPath);
                            auto newMat     = col.material;
                            std::string normalPath = col.physMatPath;
                            m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                                [scene, entity, normalPath, newMat]() {
                                    auto& c = scene->GetRegistry().get<ColliderComponent>(entity);
                                    c.physMatPath = normalPath; c.material = newMat;
                                },
                                [scene, entity, oldPath, oldMat]() {
                                    auto& c = scene->GetRegistry().get<ColliderComponent>(entity);
                                    c.physMatPath = oldPath; c.material = oldMat;
                                },
                                "Assign Physics Material"));
                        }
                    }
                    ImGui::EndDragDropTarget();
                }

                ImGui::SameLine();
                if (ImGui::Button("New")) {
                    namespace fs = std::filesystem;
                    fs::path dir = m_ContentPanel
                        ? m_ContentPanel->GetCurrentPath()
                        : fs::path(ASSETS_DIR);
                    fs::path base = dir / "NewPhysicsMaterial";
                    fs::path p = base; p += ".physmat";
                    for (int n = 1; fs::exists(p); ++n) {
                        p = base; p += " (" + std::to_string(n) + ").physmat";
                    }
                    std::string newPath = p.make_preferred().string();
                    PhysicsMaterial defaultMat{};
                    if (SavePhysicsMaterial(newPath, defaultMat)) {
                        std::string oldPath = col.physMatPath;
                        auto        oldMat  = col.material;
                        col.physMatPath = newPath;
                        col.material    = std::make_shared<PhysicsMaterial>(defaultMat);
                        auto newMat = col.material;
                        m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                            [scene, entity, newPath, newMat]() {
                                auto& c = scene->GetRegistry().get<ColliderComponent>(entity);
                                c.physMatPath = newPath; c.material = newMat;
                            },
                            [scene, entity, oldPath, oldMat]() {
                                auto& c = scene->GetRegistry().get<ColliderComponent>(entity);
                                c.physMatPath = oldPath; c.material = oldMat;
                            },
                            "New Physics Material"));
                        if (m_ContentPanel) m_ContentPanel->Refresh();
                    }
                }

                if (!col.physMatPath.empty()) {
                    ImGui::SameLine();
                    if (ImGui::Button("×")) {
                        std::string oldPath = col.physMatPath;
                        auto        oldMat  = col.material;
                        col.physMatPath = "";
                        col.material.reset();
                        m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                            [scene, entity]() {
                                auto& c = scene->GetRegistry().get<ColliderComponent>(entity);
                                c.physMatPath = ""; c.material.reset();
                            },
                            [scene, entity, oldPath, oldMat]() {
                                auto& c = scene->GetRegistry().get<ColliderComponent>(entity);
                                c.physMatPath = oldPath; c.material = oldMat;
                            },
                            "Clear Physics Material"));
                    }
                }
            }

            // --- editable values ---
            // Lazy-load: physMatPath set but material null means scene loaded before material
            // was in memory (serializer path loaded correctly but shared_ptr wasn't populated).
            if (!col.physMatPath.empty() && !col.material)
                col.material = LoadPhysicsMaterial(col.physMatPath);

            if (!col.physMatPath.empty() && col.material) {
                std::string matPath = col.physMatPath;

                ImGui::DragFloat("Static Friction", &col.material->staticFriction, 0.01f, 0.0f, 1.0f);
                if (ImGui::IsItemActivated()) m_OldStaticFriction = col.material->staticFriction;
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float nv = col.material->staticFriction, ov = m_OldStaticFriction;
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity, nv, matPath]() {
                            auto& c = scene->GetRegistry().get<ColliderComponent>(entity);
                            c.material->staticFriction = nv;
                            SavePhysicsMaterial(matPath, *c.material);
                        },
                        [scene, entity, ov, matPath]() {
                            auto& c = scene->GetRegistry().get<ColliderComponent>(entity);
                            c.material->staticFriction = ov;
                            SavePhysicsMaterial(matPath, *c.material);
                        },
                        "Change Static Friction"));
                }

                ImGui::DragFloat("Dynamic Friction", &col.material->dynamicFriction, 0.01f, 0.0f, 1.0f);
                if (ImGui::IsItemActivated()) m_OldDynamicFriction = col.material->dynamicFriction;
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float nv = col.material->dynamicFriction, ov = m_OldDynamicFriction;
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity, nv, matPath]() {
                            auto& c = scene->GetRegistry().get<ColliderComponent>(entity);
                            c.material->dynamicFriction = nv;
                            SavePhysicsMaterial(matPath, *c.material);
                        },
                        [scene, entity, ov, matPath]() {
                            auto& c = scene->GetRegistry().get<ColliderComponent>(entity);
                            c.material->dynamicFriction = ov;
                            SavePhysicsMaterial(matPath, *c.material);
                        },
                        "Change Dynamic Friction"));
                }

                ImGui::DragFloat("Restitution", &col.material->restitution, 0.01f, 0.0f, 1.0f);
                if (ImGui::IsItemActivated()) m_OldRestitution = col.material->restitution;
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float nv = col.material->restitution, ov = m_OldRestitution;
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity, nv, matPath]() {
                            auto& c = scene->GetRegistry().get<ColliderComponent>(entity);
                            c.material->restitution = nv;
                            SavePhysicsMaterial(matPath, *c.material);
                        },
                        [scene, entity, ov, matPath]() {
                            auto& c = scene->GetRegistry().get<ColliderComponent>(entity);
                            c.material->restitution = ov;
                            SavePhysicsMaterial(matPath, *c.material);
                        },
                        "Change Restitution"));
                }
            } else if (col.physMatPath.empty() && col.material) {
                ImGui::TextDisabled("(unsaved — drag a .physmat or click New)");
                ImGui::DragFloat("Static Friction",  &col.material->staticFriction,  0.01f, 0.0f, 1.0f);
                ImGui::DragFloat("Dynamic Friction", &col.material->dynamicFriction, 0.01f, 0.0f, 1.0f);
                ImGui::DragFloat("Restitution",      &col.material->restitution,     0.01f, 0.0f, 1.0f);
                if (ImGui::SmallButton("Remove")) col.material.reset();
            }
        }
    }

    // Rigidbody Component
    if (registry.all_of<RigidBodyComponent>(entity)) {
        ImGui::Separator();
        auto& rb = registry.get<RigidBodyComponent>(entity);

        ImGui::Text("Rigidbody");
        if (ImGui::BeginPopupContextItem("##RigidbodyCompCtx")) {
            if (ImGui::MenuItem("Remove Component")) {
                m_PendingRemovedRigidbody = rb;
                removeRigidbody = true;
            }
            ImGui::EndPopup();
        }

        if (!removeRigidbody) {
            const char* bodyTypes[] = { "Static", "Dynamic", "Kinematic" };
            BodyType oldType = rb.bodyType;
            int typeIdx = (int)rb.bodyType;
            if (ImGui::Combo("Body Type", &typeIdx, bodyTypes, 3)) {
                rb.bodyType = (BodyType)typeIdx;
                BodyType newType = rb.bodyType;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<BodyType>>(
                    [scene, entity](const BodyType& t) { scene->GetRegistry().get<RigidBodyComponent>(entity).bodyType = t; },
                    oldType, newType, "Change Body Type"));
            }

            if (rb.bodyType == BodyType::Dynamic) {
                ImGui::DragFloat("Mass", &rb.mass, 0.1f, 0.001f, 100000.0f);
                if (ImGui::IsItemActivated())            m_OldRbMass = rb.mass;
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float n = rb.mass, o = m_OldRbMass;
                    m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                        [scene, entity](const float& v) { scene->GetRegistry().get<RigidBodyComponent>(entity).mass = v; },
                        o, n, "Change Mass"));
                }

                ImGui::DragFloat("Gravity Scale", &rb.gravityScale, 0.01f, 0.0f, 10.0f);
                if (ImGui::IsItemActivated())            m_OldRbGravityScale = rb.gravityScale;
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float n = rb.gravityScale, o = m_OldRbGravityScale;
                    m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                        [scene, entity](const float& v) { scene->GetRegistry().get<RigidBodyComponent>(entity).gravityScale = v; },
                        o, n, "Change Gravity Scale"));
                }
            }

            if (rb.bodyType != BodyType::Static) {
                ImGui::DragFloat("Linear Damping",  &rb.linearDamping,  0.001f, 0.0f, 1.0f);
                if (ImGui::IsItemActivated())            m_OldRbLinearDamping = rb.linearDamping;
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float n = rb.linearDamping, o = m_OldRbLinearDamping;
                    m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                        [scene, entity](const float& v) { scene->GetRegistry().get<RigidBodyComponent>(entity).linearDamping = v; },
                        o, n, "Change Linear Damping"));
                }

                ImGui::DragFloat("Angular Damping", &rb.angularDamping, 0.001f, 0.0f, 1.0f);
                if (ImGui::IsItemActivated())            m_OldRbAngularDamping = rb.angularDamping;
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float n = rb.angularDamping, o = m_OldRbAngularDamping;
                    m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                        [scene, entity](const float& v) { scene->GetRegistry().get<RigidBodyComponent>(entity).angularDamping = v; },
                        o, n, "Change Angular Damping"));
                }

                ImGui::Spacing();
                ImGui::TextDisabled("Freeze Rotation");
                ImGui::Checkbox("X##lockRotX", &rb.lockRotX);
                ImGui::SameLine();
                ImGui::Checkbox("Y##lockRotY", &rb.lockRotY);
                ImGui::SameLine();
                ImGui::Checkbox("Z##lockRotZ", &rb.lockRotZ);

                ImGui::Spacing();
                ImGui::Checkbox("Continuous Collision (CCD)", &rb.continuousCollision);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Sweep the shape along its motion so a fast/thin body\n"
                                      "doesn't tunnel through thin geometry. Enable for\n"
                                      "projectiles or a swung limb. Slight extra cost.");
            }
        }
    }

    // Constraint Component
    if (registry.all_of<ConstraintComponent>(entity)) {
        ImGui::Separator();
        auto& cc = registry.get<ConstraintComponent>(entity);

        ImGui::Text("Constraint");
        if (ImGui::BeginPopupContextItem("##ConstraintCompCtx")) {
            if (ImGui::MenuItem("Remove Component")) {
                m_PendingRemovedConstraint = cc;
                removeConstraint = true;
            }
            ImGui::EndPopup();
        }

        if (!removeConstraint) {
            const char* types[] = { "Hinge", "Swing Twist", "Fixed", "Point" };
            int typeIdx = (int)cc.type;
            ImGui::Combo("Type", &typeIdx, types, IM_ARRAYSIZE(types));
            cc.type = (ConstraintType)typeIdx;

            // Target body — "World" (uuid 0) or another entity, stored by UUID so
            // the reference survives save/load. Self is excluded.
            entt::entity curTarget = scene->FindByUuid(cc.targetUuid);
            std::string  curLabel  = (cc.targetUuid == 0)
                ? "World"
                : (registry.valid(curTarget) ? scene->GetEntityName(curTarget) : "(missing)");
            if (ImGui::BeginCombo("Target", curLabel.c_str())) {
                if (ImGui::Selectable("World", cc.targetUuid == 0))
                    cc.targetUuid = 0;
                for (auto& [e, nm] : scene->GetEntityNames()) {
                    if (e == entity || !registry.all_of<IDComponent>(e)) continue;
                    uint64_t uuid = registry.get<IDComponent>(e).uuid;
                    if (ImGui::Selectable(nm.c_str(), cc.targetUuid == uuid))
                        cc.targetUuid = uuid;
                }
                ImGui::EndCombo();
            }

            // World-space anchor + axis. Read once when play starts, so plain
            // edits here are fine — undo/redo wiring comes in a later step.
            ImGui::DragFloat3("Anchor (world)", glm::value_ptr(cc.anchor), 0.05f);

            if (cc.type == ConstraintType::Hinge) {
                ImGui::DragFloat3("Axis", glm::value_ptr(cc.axis), 0.01f);
                ImGui::Checkbox("Limits", &cc.hasLimits);
                if (cc.hasLimits) {
                    ImGui::DragFloat("Min Angle", &cc.limitMin, 1.0f, -180.0f, 0.0f, "%.0f deg");
                    ImGui::DragFloat("Max Angle", &cc.limitMax, 1.0f,    0.0f, 180.0f, "%.0f deg");
                }

                ImGui::Spacing();
                const char* motorModes[] = { "Off", "Velocity", "Position" };
                int mIdx = (int)cc.motorMode;
                if (ImGui::Combo("Motor", &mIdx, motorModes, IM_ARRAYSIZE(motorModes)))
                    cc.motorMode = (MotorMode)mIdx;
                if (cc.motorMode != MotorMode::Off) {
                    ImGui::DragFloat(cc.motorMode == MotorMode::Position ? "Target (deg)" : "Target (deg/s)",
                                     &cc.motorTarget, 1.0f, -100000.0f, 100000.0f, "%.0f");
                    ImGui::DragFloat("Max Torque", &cc.motorMaxTorque, 1.0f, 0.0f, 100000.0f, "%.0f");
                    if (cc.motorMode == MotorMode::Position) {
                        ImGui::DragFloat("Frequency", &cc.motorFrequency, 0.1f, 0.0f, 60.0f, "%.1f Hz");
                        ImGui::DragFloat("Damping",   &cc.motorDamping,   0.05f, 0.0f, 10.0f, "%.2f");
                    }
                }
            } else if (cc.type == ConstraintType::SwingTwist) { // SwingTwist
                ImGui::DragFloat3("Twist Axis", glm::value_ptr(cc.axis), 0.01f);
                ImGui::TextDisabled("Swing cone (half-angles)");
                ImGui::DragFloat("Swing Normal", &cc.swingNormalDeg, 1.0f, 0.0f, 180.0f, "%.0f deg");
                ImGui::DragFloat("Swing Plane",  &cc.swingPlaneDeg,  1.0f, 0.0f, 180.0f, "%.0f deg");
                ImGui::TextDisabled("Twist range");
                ImGui::DragFloat("Twist Min", &cc.twistMinDeg, 1.0f, -180.0f, 0.0f, "%.0f deg");
                ImGui::DragFloat("Twist Max", &cc.twistMaxDeg, 1.0f,    0.0f, 180.0f, "%.0f deg");

                ImGui::Spacing();
                const char* motorModes[] = { "Off", "Velocity", "Position" };
                int mIdx = (int)cc.motorMode;
                if (ImGui::Combo("Motor", &mIdx, motorModes, IM_ARRAYSIZE(motorModes)))
                    cc.motorMode = (MotorMode)mIdx;
                if (cc.motorMode != MotorMode::Off) {
                    // Target pose as euler degrees (pitch/yaw/roll), or deg/s in Velocity mode.
                    ImGui::DragFloat3(cc.motorMode == MotorMode::Position ? "Target (deg)" : "Target (deg/s)",
                                      glm::value_ptr(cc.motorTargetEuler), 1.0f);
                    ImGui::DragFloat("Max Torque", &cc.motorMaxTorque, 1.0f, 0.0f, 100000.0f, "%.0f");
                    if (cc.motorMode == MotorMode::Position) {
                        ImGui::DragFloat("Frequency", &cc.motorFrequency, 0.1f, 0.0f, 60.0f, "%.1f Hz");
                        ImGui::DragFloat("Damping",   &cc.motorDamping,   0.05f, 0.0f, 10.0f, "%.2f");
                    }
                }
            } else if (cc.type == ConstraintType::Fixed) {
                ImGui::TextDisabled("Fixed constraint locks all linear and angular motion.");
                ImGui::TextDisabled("Only Target and Anchor are used.");
            } else if (cc.type == ConstraintType::Point) {
                ImGui::TextDisabled("Point constraint pins the anchor; rotation stays free.");
                ImGui::TextDisabled("Only Target and Anchor are used.");
            }

            ImGui::TextDisabled(cc.targetUuid == 0
                ? "Attached to world. Press Play."
                : "Connected to target entity. Press Play.");
        }
    }

    // Skinned Mesh Component
    if (registry.all_of<SkinnedMeshComponent>(entity)) {
        ImGui::Separator();
        auto& smc = registry.get<SkinnedMeshComponent>(entity);

        ImGui::Text("Skinned Mesh");
        if (ImGui::BeginPopupContextItem("##SkinnedMeshCompCtx")) {
            if (ImGui::MenuItem("Remove Component")) {
                m_PendingRemovedSkinnedMesh = smc;
                removeSkinnedMesh = true;
            }
            ImGui::EndPopup();
        }

        if (!removeSkinnedMesh) {
            if (ImGui::Checkbox("Visible", &smc.visible)) {
                bool v = smc.visible;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<bool>>(
                    [scene, entity](const bool& x) { scene->GetRegistry().get<SkinnedMeshComponent>(entity).visible = x; },
                    !v, v, "Toggle Visibility"));
            }
            ImGui::SameLine();
            if (ImGui::Checkbox("Cast Shadows", &smc.castsShadow)) {
                bool v = smc.castsShadow;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<bool>>(
                    [scene, entity](const bool& x) { scene->GetRegistry().get<SkinnedMeshComponent>(entity).castsShadow = x; },
                    !v, v, "Toggle Cast Shadows"));
            }

            ImGui::Spacing();
            ImGui::TextDisabled("Source");
            ImGui::Spacing();
            if (auto r = DrawSkinnedMeshSlot(smc, m_ContentPanel); r.changed) {
                SkinnedMeshComponent oldComp = r.oldComp;
                SkinnedMeshComponent newComp = smc;   // post-drop state
                m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                    [scene, entity, newComp]() { scene->GetRegistry().emplace_or_replace<SkinnedMeshComponent>(entity, newComp); },
                    [scene, entity, oldComp]() { scene->GetRegistry().emplace_or_replace<SkinnedMeshComponent>(entity, oldComp); },
                    "Change Skinned Mesh"));
            }
            ImGui::Spacing();
            ImGui::Text("%zu bones, %zu submeshes, %zu clips",
                        smc.skeleton.bones.size(), smc.meshes.size(), smc.clips.size());

            if (!smc.clips.empty()) {
                ImGui::Spacing();
                ImGui::TextDisabled("Clips");
                for (size_t i = 0; i < smc.clips.size(); ++i)
                    ImGui::BulletText("%zu: %s (%.2fs)", i,
                                      smc.clips[i].name.empty() ? "(unnamed)" : smc.clips[i].name.c_str(),
                                      smc.clips[i].duration);
            }

            // --- Ragdoll -------------------------------------------------------
            // Auto-generate (or load) a .ragdoll for this skeleton and attach a
            // RagdollComponent. At play start PhysicsSystem builds kinematic bodies
            // that follow the animation; "Go Limp" flips them to dynamic and the
            // character flops (Docs/ragdoll-design.md).
            if (!smc.skeleton.bones.empty()) {
                ImGui::Spacing();
                ImGui::TextDisabled("Ragdoll");

                if (!registry.all_of<RagdollComponent>(entity)) {
                    if (ImGui::Button("Add Ragdoll")) {
                        std::string path = smc.meshPath.empty()
                            ? std::string("Assets/ragdoll.ragdoll")
                            : std::filesystem::path(smc.meshPath).replace_extension(".ragdoll").string();

                        // Reuse an existing .ragdoll if present, otherwise auto-generate + save.
                        auto cfg = std::make_shared<RagdollConfig>();
                        if (!LoadRagdoll(path, *cfg)) {
                            *cfg = BuildDefaultRagdoll(smc.skeleton);
                            SaveRagdoll(path, *cfg);
                        }
                        auto& rag    = registry.emplace<RagdollComponent>(entity);
                        rag.assetPath = NormalizeRagdollPath(path);
                        rag.config    = cfg;
                        spdlog::info("Ragdoll attached: {} bodies <- {}", cfg->bodies.size(), path);
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Auto-generate (or load) a .ragdoll next to the model\n"
                                          "and attach a RagdollComponent.");
                } else {
                    auto& rag = registry.get<RagdollComponent>(entity);
                    const char* modeName = rag.mode == RagdollMode::Limp    ? "Limp"
                                         : rag.mode == RagdollMode::Powered  ? "Powered"
                                                                            : "Animated";
                    ImGui::Text("%zu bodies  (%s)",
                                rag.config ? rag.config->bodies.size() : 0u, modeName);
                    ImGui::TextDisabled("%s", rag.assetPath.c_str());

                    const bool playing = scene && scene->IsPlaying();
                    ImGui::BeginDisabled(!playing);
                    if (ImGui::Button("Go Limp"))
                        Physics::SetRagdollMode(rag, RagdollMode::Limp);
                    ImGui::SameLine();
                    if (ImGui::Button("Go Active"))
                        Physics::SetRagdollMode(rag, RagdollMode::Powered);
                    ImGui::SameLine();
                    if (ImGui::Button("Reset to Animated"))
                        Physics::SetRagdollMode(rag, RagdollMode::Animated);
                    if (ImGui::IsItemHovered() && playing)
                        ImGui::SetTooltip("Active = motors fight toward the animation pose.");
                    if (ImGui::Button("Get Up"))
                        Physics::RagdollGetUp(rag);
                    if (ImGui::IsItemHovered() && playing)
                        ImGui::SetTooltip("Procedurally heave the ragdoll back upright (motors drive\n"
                                          "toward the stand pose). Also fires automatically once a\n"
                                          "knocked-down rig settles (see Settle Delay).");
                    // Muscle strength for Powered mode — live-tunable mid-play.
                    if (ImGui::SliderFloat("Strength", &rag.strength, 0.0f, 1.0f, "%.2f"))
                        Physics::SetRagdollStrength(rag, rag.strength);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Powered-mode muscle strength: scales every joint motor's\n"
                                          "torque budget. 1 = full strength, 0 = effectively limp.");
                    ImGui::EndDisabled();
                    if (!playing && ImGui::IsItemHovered())
                        ImGui::SetTooltip("Enter play mode to trigger the ragdoll.");

                    // --- Editable config -------------------------------------------
                    // Edits mutate the loaded RagdollConfig; "Save" writes it back to the
                    // .ragdoll asset. The live ragdoll is built from a snapshot at play
                    // start, so changes here take effect on the next play (not mid-flop).
                    if (rag.config) {
                        bool dirty = false;
                        ImGui::Spacing();

                        // Two-tier auto hit-react. Knockdown (>= threshold) goes fully
                        // Limp; the lighter flinch tier staggers in Powered mode and
                        // recovers. 0 disables a tier (manual Go Limp/Active still work).
                        if (ImGui::DragFloat("Knockdown Threshold", &rag.config->impactThreshold,
                                             0.5f, 0.0f, 100000.0f, "%.1f"))
                            { rag.config->impactThreshold = glm::max(0.0f, rag.config->impactThreshold); dirty = true; }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Impact momentum (kg.m/s) needed to knock the ragdoll fully\n"
                                              "limp automatically on a hit. 0 = off (manual Go Limp only).");

                        if (ImGui::DragFloat("Flinch Threshold", &rag.config->flinchThreshold,
                                             0.5f, 0.0f, 100000.0f, "%.1f"))
                            { rag.config->flinchThreshold = glm::max(0.0f, rag.config->flinchThreshold); dirty = true; }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Lighter hit tier (kg.m/s): a hit at/above this but below the\n"
                                              "knockdown threshold staggers the rig (Powered flinch) instead of\n"
                                              "flooring it. Keep below Knockdown Threshold. 0 = off.");
                        if (ImGui::DragFloat("Flinch Strength", &rag.config->flinchStrength,
                                             0.01f, 0.0f, 1.0f, "%.2f"))
                            { rag.config->flinchStrength = glm::clamp(rag.config->flinchStrength, 0.0f, 1.0f); dirty = true; }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Muscle strength at the bottom of the flinch dip (0 = goes\n"
                                              "momentarily limp, 1 = barely flinches).");
                        if (ImGui::DragFloat("Flinch Recovery", &rag.config->flinchRecovery,
                                             0.01f, 0.01f, 5.0f, "%.2f s"))
                            { rag.config->flinchRecovery = glm::max(0.01f, rag.config->flinchRecovery); dirty = true; }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Seconds to ramp muscle strength back to full after a flinch,\n"
                                              "then return to animation-driven control.");

                        // Procedural get-up tuning (the sloppy stagger back to standing).
                        ImGui::Spacing();
                        ImGui::TextDisabled("Get-up (procedural recovery)");
                        if (ImGui::DragFloat("Get-up Strength", &rag.config->getupStrength,
                                             0.01f, 0.0f, 1.0f, "%.2f"))
                            { rag.config->getupStrength = glm::clamp(rag.config->getupStrength, 0.0f, 1.0f); dirty = true; }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Peak muscle strength while rising. <1 keeps it wobbly/drunk.");
                        if (ImGui::DragFloat("Get-up Duration", &rag.config->getupDuration,
                                             0.01f, 0.05f, 8.0f, "%.2f s"))
                            { rag.config->getupDuration = glm::max(0.05f, rag.config->getupDuration); dirty = true; }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Seconds to heave from sprawled to standing.");
                        if (ImGui::DragFloat("Get-up Wobble", &rag.config->getupWobble,
                                             0.01f, 0.0f, 2.0f, "%.2f"))
                            { rag.config->getupWobble = glm::max(0.0f, rag.config->getupWobble); dirty = true; }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Per-joint torque noise during get-up. 0 = clean, ~0.3 = drunk flail.");
                        if (ImGui::DragFloat("Settle Delay", &rag.config->getupDelay,
                                             0.05f, 0.0f, 10.0f, "%.2f s"))
                            { rag.config->getupDelay = glm::max(0.0f, rag.config->getupDelay); dirty = true; }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Seconds a knocked-down rig must lie still before it auto-gets-up.\n"
                                              "0 = manual Get-Up button only.");
                        if (ImGui::DragFloat("Balance Assist", &rag.config->getupBalance,
                                             0.05f, 0.0f, 5.0f, "%.2f"))
                            { rag.config->getupBalance = glm::max(0.0f, rag.config->getupBalance); dirty = true; }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Direct pelvis upright+lift assist (joint motors can't lift the\n"
                                              "root). 1 = normal, higher = stronger/snappier stand, 0 = pure\n"
                                              "motors (won't actually rise).");
                        if (ImGui::DragFloat("Hold Then Resume", &rag.config->getupHold,
                                             0.05f, 0.0f, 10.0f, "%.2f s"))
                            { rag.config->getupHold = glm::max(0.0f, rag.config->getupHold); dirty = true; }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("After standing, hold the pose this long, then resume the\n"
                                              "animation clip (switch to Animated). 0 = hold the stand\n"
                                              "indefinitely (manual hand-off).");
                        if (ImGui::DragFloat("Resume Blend", &rag.config->getupBlend,
                                             0.01f, 0.0f, 2.0f, "%.2f s"))
                            { rag.config->getupBlend = glm::max(0.0f, rag.config->getupBlend); dirty = true; }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Cross-blend the held stand pose into the live animation over\n"
                                              "this long before resuming the clip, so it eases in instead of\n"
                                              "popping. 0 = instant switch.");

                        if (ImGui::TreeNode("Bodies")) {
                            const char* shapeNames[] = { "Capsule", "Box", "Sphere" };
                            const char* jointNames[] = { "Hinge", "SwingTwist", "Fixed", "Point" };
                            for (size_t bi = 0; bi < rag.config->bodies.size(); ++bi) {
                                auto& d = rag.config->bodies[bi];
                                ImGui::PushID((int)bi);
                                if (ImGui::TreeNode(d.boneName.empty() ? "(unnamed)" : d.boneName.c_str())) {
                                    if (ImGui::DragFloat("Mass (kg)", &d.mass, 0.05f, 0.001f, 1000.0f, "%.3f")) dirty = true;

                                    int sh = (int)d.shape;
                                    if (ImGui::Combo("Shape", &sh, shapeNames, IM_ARRAYSIZE(shapeNames)))
                                        { d.shape = (RagdollBodyDef::Shape)sh; dirty = true; }
                                    switch (d.shape) {
                                        case RagdollBodyDef::Shape::Box:
                                            if (ImGui::DragFloat3("Half Extents", glm::value_ptr(d.halfExtents), 0.005f, 0.001f, 10.0f)) dirty = true;
                                            break;
                                        case RagdollBodyDef::Shape::Sphere:
                                            if (ImGui::DragFloat("Radius", &d.radius, 0.005f, 0.001f, 10.0f)) dirty = true;
                                            break;
                                        default: // Capsule
                                            if (ImGui::DragFloat("Radius",      &d.radius,     0.005f, 0.001f, 10.0f)) dirty = true;
                                            if (ImGui::DragFloat("Half Height", &d.halfHeight, 0.005f, 0.0f,   10.0f)) dirty = true;
                                            break;
                                    }

                                    if (d.parentBoneName.empty()) {
                                        ImGui::TextDisabled("Root body (no joint)");
                                    } else {
                                        ImGui::TextDisabled("Joint to %s", d.parentBoneName.c_str());
                                        int jt = (int)d.jointType; // enum order matches jointNames
                                        if (ImGui::Combo("Joint", &jt, jointNames, IM_ARRAYSIZE(jointNames)))
                                            { d.jointType = (ConstraintType)jt; dirty = true; }
                                        if (ImGui::DragFloat3("Twist Axis", glm::value_ptr(d.twistAxisLocal), 0.01f)) dirty = true;
                                        if (d.jointType == ConstraintType::SwingTwist) {
                                            ImGui::TextDisabled("Swing cone (half-angles)");
                                            if (ImGui::DragFloat("Swing Normal", &d.swingNormalDeg, 1.0f, 0.0f, 180.0f, "%.0f deg")) dirty = true;
                                            if (ImGui::DragFloat("Swing Plane",  &d.swingPlaneDeg,  1.0f, 0.0f, 180.0f, "%.0f deg")) dirty = true;
                                            ImGui::TextDisabled("Twist range");
                                            if (ImGui::DragFloat("Twist Min", &d.twistMinDeg, 1.0f, -180.0f, 0.0f, "%.0f deg")) dirty = true;
                                            if (ImGui::DragFloat("Twist Max", &d.twistMaxDeg, 1.0f,    0.0f, 180.0f, "%.0f deg")) dirty = true;
                                        } else if (d.jointType == ConstraintType::Hinge) {
                                            if (ImGui::DragFloat("Min Angle", &d.hingeMinDeg, 1.0f, -180.0f, 0.0f, "%.0f deg")) dirty = true;
                                            if (ImGui::DragFloat("Max Angle", &d.hingeMaxDeg, 1.0f,    0.0f, 180.0f, "%.0f deg")) dirty = true;
                                        }

                                        // Powered-mode motor (the "muscle"). Hinge + SwingTwist
                                        // only — Fixed/Point have nothing to drive.
                                        if (d.jointType == ConstraintType::SwingTwist ||
                                            d.jointType == ConstraintType::Hinge) {
                                            ImGui::TextDisabled("Motor (Powered mode)");
                                            if (ImGui::DragFloat("Max Torque", &d.motorMaxTorque, 1.0f, 0.0f, 100000.0f, "%.0f N.m")) dirty = true;
                                            if (ImGui::DragFloat("Frequency",  &d.motorFrequency, 0.1f, 0.1f, 120.0f, "%.1f Hz"))  dirty = true;
                                            if (ImGui::DragFloat("Damping",    &d.motorDamping,   0.05f, 0.0f, 10.0f, "%.2f"))      dirty = true;
                                        }
                                    }
                                    ImGui::TreePop();
                                }
                                ImGui::PopID();
                            }
                            ImGui::TreePop();
                        }

                        if (ImGui::Button("Save .ragdoll")) {
                            if (SaveRagdoll(rag.assetPath, *rag.config))
                                spdlog::info("Ragdoll saved: {}", rag.assetPath);
                            else
                                spdlog::error("Ragdoll save failed: {}", rag.assetPath);
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Regenerate")) {
                            *rag.config = BuildDefaultRagdoll(smc.skeleton);
                            SaveRagdoll(rag.assetPath, *rag.config);
                            spdlog::info("Ragdoll regenerated from skeleton: {} bodies", rag.config->bodies.size());
                        }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Discard edits and rebuild the config from the skeleton, then save.");
                        if (dirty) { ImGui::SameLine(); ImGui::TextDisabled("(unsaved)"); }
                    }

                    if (ImGui::Button("Remove Ragdoll"))
                        registry.remove<RagdollComponent>(entity);
                }
            }

            // --- Inverse Kinematics --------------------------------------------
            // Per-chain hand reach / foot placement. UpdateIK pulls each chain's
            // end-effector bone to its target and blends into the animated pose
            // (Docs/ik-design.md). Two-bone analytic solver.
            if (!smc.skeleton.bones.empty()) {
                ImGui::Spacing();
                ImGui::TextDisabled("Inverse Kinematics");

                if (!registry.all_of<IKComponent>(entity)) {
                    if (ImGui::Button("Add IK"))
                        registry.emplace<IKComponent>(entity);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Attach an IKComponent, then add reach chains below.");
                } else {
                    auto& ik = registry.get<IKComponent>(entity);

                    // Keep the viewport's active-chain index (gizmo + highlight) in range.
                    if (m_Context) {
                        if (m_Context->activeIKChain >= (int)ik.chains.size())
                            m_Context->activeIKChain = (int)ik.chains.size() - 1;
                        if (m_Context->activeIKChain < 0)
                            m_Context->activeIKChain = 0;
                    }

                    for (size_t ci = 0; ci < ik.chains.size(); ++ci) {
                        ImGui::PushID((int)ci);
                        IKChain& chain = ik.chains[ci];

                        // Active-chain radio — the active chain gets the viewport
                        // target gizmo and the highlighted debug draw.
                        bool isActive = m_Context && m_Context->activeIKChain == (int)ci;
                        if (ImGui::RadioButton("Active", isActive) && m_Context)
                            m_Context->activeIKChain = (int)ci;
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Gizmo + highlight target this chain in the viewport (enable Debug Draw).");
                        ImGui::SameLine();
                        ImGui::Text("Chain %d", (int)ci);

                        // End-effector bone picker (combo over the skeleton bones).
                        const char* curName = chain.endEffectorBone.empty()
                            ? "(pick bone)" : chain.endEffectorBone.c_str();
                        if (ImGui::BeginCombo("End Effector", curName)) {
                            for (const auto& bone : smc.skeleton.bones) {
                                bool sel = bone.name == chain.endEffectorBone;
                                if (ImGui::Selectable(bone.name.c_str(), sel)) {
                                    chain.endEffectorBone = bone.name;
                                    chain._tipBone = -1;
                                    chain._bindResolved = false;   // re-resolve bind position
                                }
                            }
                            ImGui::EndCombo();
                        }

                        ImGui::Checkbox("Foot Chain", &chain.isFootChain);

                        if (chain.isFootChain) {
                            // Foot placement drives the target from a downward raycast;
                            // the authored Target Source is ignored.
                            ImGui::DragFloat("Cast Offset",     &chain.castOffset,     0.01f, 0.0f, 5.0f);
                            if (ImGui::IsItemHovered())
                                ImGui::SetTooltip("Ray origin = animated ankle + up * castOffset.");
                            ImGui::DragFloat("Max Step Height", &chain.maxStepHeight,  0.01f, 0.0f, 2.0f);
                            if (ImGui::IsItemHovered())
                                ImGui::SetTooltip("Total ray length = castOffset + maxStepHeight.");
                            ImGui::DragFloat("Ankle Height",    &chain.ankleHeight,    0.005f, 0.0f, 0.5f);
                            if (ImGui::IsItemHovered())
                                ImGui::SetTooltip("Ankle center above the floor (added to hit point).");
                            ImGui::DragFloat("Swing Threshold", &chain.swingThreshold, 0.01f,  0.0f, 1.0f);
                            if (ImGui::IsItemHovered())
                                ImGui::SetTooltip("Ankle model-space rise above bind pose that triggers swing (IK releases).");
                            ImGui::Checkbox("Tilt to Normal", &chain.tiltToNormal);
                            if (chain.tiltToNormal)
                                ImGui::SliderFloat("Tilt Weight", &chain.tiltWeight, 0.0f, 1.0f);
                            ImGui::TextDisabled("drop %.3f  |  normal (%.2f, %.2f, %.2f)",
                                chain._rawDrop,
                                chain._groundNormal.x, chain._groundNormal.y, chain._groundNormal.z);
                        } else {
                            // Target source: a world point, or follow another entity.
                            const bool followingEntity =
                                chain.targetEntity != entt::null && registry.valid(chain.targetEntity);
                            const char* srcLabel = followingEntity
                                ? scene->GetEntityName(chain.targetEntity).c_str()
                                : "World Point";
                            if (ImGui::BeginCombo("Target Source", srcLabel)) {
                                if (ImGui::Selectable("World Point", !followingEntity))
                                    chain.targetEntity = entt::null;
                                for (auto& [e, nm] : scene->GetEntityNames()) {
                                    if (e == entity) continue;
                                    if (ImGui::Selectable(nm.c_str(), chain.targetEntity == e))
                                        chain.targetEntity = e;
                                }
                                ImGui::EndCombo();
                            }
                            if (followingEntity)
                                ImGui::DragFloat3("Target Offset", &chain.targetOffset.x, 0.01f);
                            else
                                ImGui::DragFloat3("Target (world)", &chain.targetWorldPos.x, 0.01f);
                        }

                        ImGui::DragFloat3("Pole Offset",    &chain.poleOffset.x, 0.01f);
                        ImGui::SliderFloat("Weight",        &chain.weight,   0.0f, 1.0f);
                        ImGui::SliderFloat("Ease Time (s)", &chain.easeTime, 0.0f, 0.5f);
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Smoothing for weight + target. 0 = snap.");

                        // Resolved-chain readout so a bad bone pick is obvious.
                        int tip = chain._tipBone, mid = -1, root = -1;
                        if (tip >= 0 && tip < (int)smc.skeleton.bones.size()) {
                            mid = smc.skeleton.bones[tip].parent;
                            if (mid >= 0) root = smc.skeleton.bones[mid].parent;
                        }
                        if (root >= 0)
                            ImGui::TextDisabled("chain: %s <- %s <- %s",
                                smc.skeleton.bones[tip].name.c_str(),
                                smc.skeleton.bones[mid].name.c_str(),
                                smc.skeleton.bones[root].name.c_str());
                        else
                            ImGui::TextDisabled("chain: needs a bone with two parents");

                        if (ImGui::SmallButton("Remove Chain")) {
                            ik.chains.erase(ik.chains.begin() + ci);
                            ImGui::PopID();
                            break;
                        }
                        ImGui::Separator();
                        ImGui::PopID();
                    }

                    // Pelvis correction (shown when any foot chain is present or already configured).
                    {
                        bool hasFoot = false;
                        for (const auto& c : ik.chains) if (c.isFootChain) { hasFoot = true; break; }
                        if (hasFoot || !ik.pelvisBone.empty()) {
                            ImGui::Separator();
                            ImGui::TextDisabled("Pelvis Correction");
                            const char* pelvName = ik.pelvisBone.empty() ? "(none)" : ik.pelvisBone.c_str();
                            if (ImGui::BeginCombo("Pelvis Bone", pelvName)) {
                                if (ImGui::Selectable("(none)", ik.pelvisBone.empty())) {
                                    ik.pelvisBone     = "";
                                    ik._pelvisBoneIdx = -1;
                                }
                                for (const auto& bone : smc.skeleton.bones) {
                                    bool sel = bone.name == ik.pelvisBone;
                                    if (ImGui::Selectable(bone.name.c_str(), sel)) {
                                        ik.pelvisBone     = bone.name;
                                        ik._pelvisBoneIdx = -1;
                                    }
                                }
                                ImGui::EndCombo();
                            }
                            ImGui::DragFloat("Max Pelvis Drop",  &ik.maxPelvisDrop,  0.01f, 0.0f, 2.0f);
                            ImGui::DragFloat("Pelvis Ease Time", &ik.pelvisEaseTime, 0.01f, 0.0f, 1.0f);
                            ImGui::TextDisabled("cur drop: %.3f", ik._curPelvisDrop);
                        }
                    }

                    if (ImGui::Button("Add Chain"))
                        ik.chains.emplace_back();
                    ImGui::SameLine();
                    if (ImGui::Button("Remove IK"))
                        registry.remove<IKComponent>(entity);
                }
            }
        }
    }

    // Animator Component
    if (registry.all_of<AnimatorComponent>(entity)) {
        ImGui::Separator();
        auto& anim = registry.get<AnimatorComponent>(entity);

        ImGui::Text("Animator");
        if (ImGui::BeginPopupContextItem("##AnimatorCompCtx")) {
            if (ImGui::MenuItem("Remove Component")) {
                m_PendingRemovedAnimator = anim;
                removeAnimator = true;
            }
            ImGui::EndPopup();
        }

        if (!removeAnimator) {
            // Clip list comes from the sibling SkinnedMeshComponent when present.
            auto* smc = registry.try_get<SkinnedMeshComponent>(entity);
            int   clipCount = smc ? (int)smc->clips.size() : 0;

            auto clipLabel = [&](int idx) -> std::string {
                if (idx < 0) return "(bind pose)";
                if (smc && idx < clipCount && !smc->clips[idx].name.empty())
                    return std::to_string(idx) + ": " + smc->clips[idx].name;
                return std::to_string(idx);
            };

            // Picking a clip cross-fades to it over the Blend time. The fade is
            // applied live via CrossFade(), so we RecordCommand (not Execute) — an
            // Execute would re-apply the setter and wipe the in-progress fade. Undo
            // snaps instantly (no fade).
            auto selectClip = [&](int newClip) {
                int oldClip = anim.clip;
                if (newClip == oldClip) return;
                anim.CrossFade(newClip, m_AnimBlendTime);
                m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                    [scene, entity, newClip]() {
                        auto& a = scene->GetRegistry().get<AnimatorComponent>(entity);
                        a.clip = newClip; a.prevClip = -1; a.time = 0.0f;
                    },
                    [scene, entity, oldClip]() {
                        auto& a = scene->GetRegistry().get<AnimatorComponent>(entity);
                        a.clip = oldClip; a.prevClip = -1; a.time = 0.0f;
                    },
                    "Change Clip"));
            };

            if (ImGui::BeginCombo("Clip", clipLabel(anim.clip).c_str())) {
                if (ImGui::Selectable("(bind pose)", anim.clip < 0))
                    selectClip(-1);
                for (int i = 0; i < clipCount; ++i)
                    if (ImGui::Selectable(clipLabel(i).c_str(), anim.clip == i))
                        selectClip(i);
                ImGui::EndCombo();
            }

            // Cross-fade duration used when switching clips (and, later, the
            // default for state-machine transitions). Editor-only; not serialized.
            ImGui::DragFloat("Blend (s)", &m_AnimBlendTime, 0.01f, 0.0f, 2.0f, "%.2f");

            // Playback state — direct edits (transient, not undo-tracked).
            if (ImGui::Button(anim.playing ? "Pause" : "Play"))
                anim.playing = !anim.playing;
            ImGui::SameLine();
            if (ImGui::Button("Restart"))
                anim.time = 0.0f;
            ImGui::SameLine();
            ImGui::Checkbox("Loop", &anim.loop);

            float duration = (smc && anim.clip >= 0 && anim.clip < clipCount)
                             ? smc->clips[anim.clip].duration : 0.0f;
            ImGui::SliderFloat("Time", &anim.time, 0.0f, duration > 0.0f ? duration : 1.0f, "%.2fs");

            ImGui::DragFloat("Speed", &anim.speed, 0.01f, -10.0f, 10.0f, "%.2fx");
            if (ImGui::IsItemActivated())          m_OldAnimSpeed = anim.speed;
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                float n = anim.speed, o = m_OldAnimSpeed;
                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<float>>(
                    [scene, entity](const float& v) { scene->GetRegistry().get<AnimatorComponent>(entity).speed = v; },
                    o, n, "Change Animation Speed"));
            }

            if (!smc)
                ImGui::TextDisabled("(no Skinned Mesh on this entity)");
        }
    }

    // Animator State Machine Component
    if (registry.all_of<AnimStateMachineComponent>(entity)) {
        ImGui::Separator();
        auto& sm = registry.get<AnimStateMachineComponent>(entity);

        ImGui::Text("Animator State Machine");
        if (ImGui::BeginPopupContextItem("##AnimSMCompCtx")) {
            if (ImGui::MenuItem("Remove Component")) {
                m_PendingRemovedAnimSM = sm;
                removeAnimSM = true;
            }
            ImGui::EndPopup();
        }

        if (!removeAnimSM) {
            // -- Asset slot (drag a .animsm here) --
            ImGui::TextDisabled("Asset");
            std::string dispName = sm.assetPath.empty()
                ? "None"
                : std::filesystem::path(sm.assetPath).filename().string();
            ImGui::Button(dispName.c_str(), {ImGui::GetContentRegionAvail().x, 0});
            if (ImGui::BeginDragDropTarget()) {
                if (const ImGuiPayload* p = ImGui::AcceptDragDropPayload("CONTENT_ITEM_PATH")) {
                    std::string path((const char*)p->Data);
                    if (LowerExtOf(path) == ".animsm") {
                        sm.assetPath = NormalizeAnimSmPath(path);
                        sm.machine   = LoadAnimStateMachine(sm.assetPath);
                        sm.floats.clear(); sm.bools.clear(); sm.triggers.clear();
                        sm.currentState = -1;   // re-enter entry state
                        sm.SyncParams();
                    }
                }
                ImGui::EndDragDropTarget();
            }
            if (!sm.assetPath.empty() && !sm.machine)
                ImGui::TextDisabled("(failed to load asset)");

            if (sm.machine) {
                const auto& M = *sm.machine;
                ImGui::Text("%zu states, %zu transitions, %zu params",
                            M.states.size(), M.transitions.size(), M.parameters.size());

                // Current state (runtime). -1 until play begins and the entry state
                // is entered.
                const char* cur = (sm.currentState >= 0 && sm.currentState < (int)M.states.size())
                                  ? M.states[sm.currentState].name.c_str()
                                  : "(entry on play)";
                ImGui::Text("Current: %s", cur);

                if (M.states.empty())
                    ImGui::TextDisabled("(empty graph — author states in the Animator window)");

                // -- Live parameters: drag/toggle/fire to drive transitions in play mode --
                if (!M.parameters.empty()) {
                    ImGui::Spacing();
                    ImGui::TextDisabled("Parameters");
                    for (const auto& p : M.parameters) {
                        ImGui::PushID(p.name.c_str());
                        switch (p.type) {
                            case Diamond::AnimParamType::Float: {
                                float f = sm.GetFloat(p.name);
                                if (ImGui::DragFloat(p.name.c_str(), &f, 0.01f))
                                    sm.SetFloat(p.name, f);
                                break;
                            }
                            case Diamond::AnimParamType::Bool: {
                                bool b = sm.GetBool(p.name);
                                if (ImGui::Checkbox(p.name.c_str(), &b))
                                    sm.SetBool(p.name, b);
                                break;
                            }
                            case Diamond::AnimParamType::Trigger: {
                                bool pending = sm.triggers.count(p.name) > 0;
                                if (pending) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.55f, 0.05f, 1.0f));
                                if (ImGui::Button(p.name.c_str()))
                                    sm.SetTrigger(p.name);
                                if (pending) ImGui::PopStyleColor();
                                ImGui::SameLine();
                                ImGui::TextDisabled(pending ? "(set)" : "trigger");
                                break;
                            }
                        }
                        ImGui::PopID();
                    }
                }

                if (!registry.all_of<SkinnedMeshComponent>(entity))
                    ImGui::TextDisabled("(needs a Skinned Mesh to resolve clips)");
            }
        }
    }

    // Registered game components
    const ComponentDescriptor* removeUserComp = nullptr;

    for (const auto& desc : ComponentRegistry::Get().GetAll())
    {
        if (!desc.has(*scene, entity)) continue;

        // Isolate each registered component's widget IDs so identical labels in
        // different components (e.g. an "Enabled" checkbox in both CameraDirector
        // and AudioListener) don't collide into the same ImGui ID.
        ImGui::PushID(desc.name.c_str());

        ImGui::Separator();
        ImGui::Text("%s", desc.name.c_str());

        std::string ctxId = "##Ctx_" + desc.name;
        if (ImGui::BeginPopupContextItem(ctxId.c_str()))
        {
            if (ImGui::MenuItem("Remove Component"))
                removeUserComp = &desc;
            ImGui::EndPopup();
        }

        if (desc.drawInspector)
            desc.drawInspector(*scene, entity);

        ImGui::PopID();
    }

    // Deferred removals — component refs (mc/lc) are no longer live past here
    if (removeMesh && m_PendingRemovedMesh) {
        MeshComponent saved = *m_PendingRemovedMesh;
        m_PendingRemovedMesh.reset();
        registry.remove<MeshComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()       { scene->GetRegistry().remove<MeshComponent>(entity); },
            [scene, entity, saved]() { scene->GetRegistry().emplace_or_replace<MeshComponent>(entity, saved); },
            "Remove Mesh Component"));
    }
    if (removeLight && m_PendingRemovedLight) {
        LightComponent saved = *m_PendingRemovedLight;
        m_PendingRemovedLight.reset();
        registry.remove<LightComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()       { scene->GetRegistry().remove<LightComponent>(entity); },
            [scene, entity, saved]() { scene->GetRegistry().emplace_or_replace<LightComponent>(entity, saved); },
            "Remove Light Component"));
    }
    if (removeCamera && m_PendingRemovedCamera) {
        CameraComponent saved = *m_PendingRemovedCamera;
        m_PendingRemovedCamera.reset();
        registry.remove<CameraComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()        { scene->GetRegistry().remove<CameraComponent>(entity); },
            [scene, entity, saved]() { scene->GetRegistry().emplace_or_replace<CameraComponent>(entity, saved); },
            "Remove Camera Component"));
    }
    if (removeCollider && m_PendingRemovedCollider) {
        ColliderComponent saved = *m_PendingRemovedCollider;
        m_PendingRemovedCollider.reset();
        registry.remove<ColliderComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()        { scene->GetRegistry().remove<ColliderComponent>(entity); },
            [scene, entity, saved]() { scene->GetRegistry().emplace_or_replace<ColliderComponent>(entity, saved); },
            "Remove Collider Component"));
    }
    if (removeRigidbody && m_PendingRemovedRigidbody) {
        RigidBodyComponent saved = *m_PendingRemovedRigidbody;
        m_PendingRemovedRigidbody.reset();
        registry.remove<RigidBodyComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()        { scene->GetRegistry().remove<RigidBodyComponent>(entity); },
            [scene, entity, saved]() { scene->GetRegistry().emplace_or_replace<RigidBodyComponent>(entity, saved); },
            "Remove Rigidbody Component"));
    }
    if (removeConstraint && m_PendingRemovedConstraint) {
        ConstraintComponent saved = *m_PendingRemovedConstraint;
        m_PendingRemovedConstraint.reset();
        registry.remove<ConstraintComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()        { scene->GetRegistry().remove<ConstraintComponent>(entity); },
            [scene, entity, saved]() { scene->GetRegistry().emplace_or_replace<ConstraintComponent>(entity, saved); },
            "Remove Constraint Component"));
    }
    if (removeSkinnedMesh && m_PendingRemovedSkinnedMesh) {
        SkinnedMeshComponent saved = *m_PendingRemovedSkinnedMesh;
        m_PendingRemovedSkinnedMesh.reset();
        registry.remove<SkinnedMeshComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()        { scene->GetRegistry().remove<SkinnedMeshComponent>(entity); },
            [scene, entity, saved]() { scene->GetRegistry().emplace_or_replace<SkinnedMeshComponent>(entity, saved); },
            "Remove Skinned Mesh Component"));
    }
    if (removeAnimator && m_PendingRemovedAnimator) {
        AnimatorComponent saved = *m_PendingRemovedAnimator;
        m_PendingRemovedAnimator.reset();
        registry.remove<AnimatorComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()        { scene->GetRegistry().remove<AnimatorComponent>(entity); },
            [scene, entity, saved]() { scene->GetRegistry().emplace_or_replace<AnimatorComponent>(entity, saved); },
            "Remove Animator Component"));
    }
    if (removeAnimSM && m_PendingRemovedAnimSM) {
        AnimStateMachineComponent saved = *m_PendingRemovedAnimSM;
        m_PendingRemovedAnimSM.reset();
        registry.remove<AnimStateMachineComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()        { scene->GetRegistry().remove<AnimStateMachineComponent>(entity); },
            [scene, entity, saved]() { scene->GetRegistry().emplace_or_replace<AnimStateMachineComponent>(entity, saved); },
            "Remove Animator State Machine Component"));
    }
    if (removeCanvas) {
        registry.remove<CanvasComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()             { scene->GetRegistry().remove<CanvasComponent>(entity); },
            [scene, entity, savedCanvas]() { scene->GetRegistry().emplace_or_replace<CanvasComponent>(entity, savedCanvas); },
            "Remove Canvas Component"));
    }
    if (removeRect) {
        registry.remove<RectTransformComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()           { scene->GetRegistry().remove<RectTransformComponent>(entity); },
            [scene, entity, savedRect]() { scene->GetRegistry().emplace_or_replace<RectTransformComponent>(entity, savedRect); },
            "Remove Rect Transform Component"));
    }
    if (removeUIImage) {
        registry.remove<UIImageComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()              { scene->GetRegistry().remove<UIImageComponent>(entity); },
            [scene, entity, savedUIImage]() { scene->GetRegistry().emplace_or_replace<UIImageComponent>(entity, savedUIImage); },
            "Remove UI Image Component"));
    }
    if (removeUIText) {
        registry.remove<UITextComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()             { scene->GetRegistry().remove<UITextComponent>(entity); },
            [scene, entity, savedUIText]() { scene->GetRegistry().emplace_or_replace<UITextComponent>(entity, savedUIText); },
            "Remove UI Text Component"));
    }
    if (removeUIProgress) {
        registry.remove<UIProgressBarComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()                 { scene->GetRegistry().remove<UIProgressBarComponent>(entity); },
            [scene, entity, savedUIProgress]() { scene->GetRegistry().emplace_or_replace<UIProgressBarComponent>(entity, savedUIProgress); },
            "Remove UI Progress Bar Component"));
    }
    if (removeUIButton) {
        registry.remove<UIButtonComponent>(entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity]()               { scene->GetRegistry().remove<UIButtonComponent>(entity); },
            [scene, entity, savedUIButton]() { scene->GetRegistry().emplace_or_replace<UIButtonComponent>(entity, savedUIButton); },
            "Remove Button Component"));
    }
    if (removeUserComp)
    {
        auto removeFn  = removeUserComp->remove;
        auto addFn     = removeUserComp->add;
        std::string nm = removeUserComp->name;
        removeFn(*scene, entity);
        m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
            [scene, entity, removeFn]() { removeFn(*scene, entity); },
            [scene, entity, addFn]()    { addFn(*scene, entity); },
            "Remove " + nm));
    }

    ImGui::Spacing();
    ImGui::Spacing();

    // ---- Add Component button --------------------------------------------------
    const float btnW = 180.0f;
    ImGui::SetCursorPosX((ImGui::GetContentRegionAvail().x - btnW) * 0.5f);
    if (ImGui::Button("Add Component +", {btnW, 0.0f}))
        ImGui::OpenPopup("##AddComponentPopup");

    if (ImGui::BeginPopup("##AddComponentPopup")) {
        ImGui::TextDisabled("Components");
        ImGui::Separator();

        bool hasMesh        = registry.all_of<MeshComponent>(entity);
        bool hasLight       = registry.all_of<LightComponent>(entity);
        bool hasCamera      = registry.all_of<CameraComponent>(entity);
        bool hasCollider    = registry.all_of<ColliderComponent>(entity);
        bool hasRigidbody   = registry.all_of<RigidBodyComponent>(entity);
        bool hasConstraint  = registry.all_of<ConstraintComponent>(entity);
        // Animator + state machine only make sense alongside a skinned mesh (they
        // index / resolve its clips).
        bool hasSkinnedMesh = registry.all_of<SkinnedMeshComponent>(entity);
        bool hasAnimator    = registry.all_of<AnimatorComponent>(entity);
        bool hasAnimSM      = registry.all_of<AnimStateMachineComponent>(entity);
        bool hasCanvas      = registry.all_of<CanvasComponent>(entity);
        bool hasRect        = registry.all_of<RectTransformComponent>(entity);
        bool hasUIImage     = registry.all_of<UIImageComponent>(entity);
        bool hasUIText      = registry.all_of<UITextComponent>(entity);
        bool hasUIProgress  = registry.all_of<UIProgressBarComponent>(entity);
        bool hasUIButton    = registry.all_of<UIButtonComponent>(entity);
        bool canAddAnimator = hasSkinnedMesh && !hasAnimator;
        bool canAddAnimSM   = hasSkinnedMesh && !hasAnimSM;
        // UI widgets need a rect to resolve into; only offer them on a rect entity.
        bool canAddUIImage    = hasRect && !hasUIImage;
        bool canAddUIText     = hasRect && !hasUIText;
        bool canAddUIProgress = hasRect && !hasUIProgress;
        bool canAddUIButton   = hasRect && !hasUIButton;
        bool anyShown       = !hasMesh || !hasLight || !hasCamera || !hasCollider
                              || !hasRigidbody || !hasConstraint || !hasSkinnedMesh
                              || canAddAnimator || canAddAnimSM || !hasCanvas || !hasRect
                              || canAddUIImage || canAddUIText || canAddUIProgress || canAddUIButton;

        constexpr ImGuiSelectableFlags kCompFlags =
            ImGuiSelectableFlags_AllowDoubleClick | ImGuiSelectableFlags_DontClosePopups;

        if (!hasMesh) {
            if (ImGui::Selectable("Mesh Renderer", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() { scene->GetRegistry().emplace<MeshComponent>(entity).material = std::make_shared<PBRMaterial>(); },
                        [scene, entity]() { scene->GetRegistry().remove<MeshComponent>(entity); },
                        "Add Mesh Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (!hasLight) {
            if (ImGui::Selectable("Light", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() { scene->GetRegistry().emplace<LightComponent>(entity); },
                        [scene, entity]() { scene->GetRegistry().remove<LightComponent>(entity); },
                        "Add Light Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (!hasCamera) {
            if (ImGui::Selectable("Camera", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() { scene->GetRegistry().emplace<CameraComponent>(entity); },
                        [scene, entity]() { scene->GetRegistry().remove<CameraComponent>(entity); },
                        "Add Camera Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (!hasCollider) {
            if (ImGui::Selectable("Collider", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() { scene->GetRegistry().emplace<ColliderComponent>(entity); },
                        [scene, entity]() { scene->GetRegistry().remove<ColliderComponent>(entity); },
                        "Add Collider Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (!hasRigidbody) {
            if (ImGui::Selectable("Rigidbody", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() { scene->GetRegistry().emplace<RigidBodyComponent>(entity); },
                        [scene, entity]() { scene->GetRegistry().remove<RigidBodyComponent>(entity); },
                        "Add Rigidbody Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (!hasConstraint) {
            if (ImGui::Selectable("Constraint", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() { scene->GetRegistry().emplace<ConstraintComponent>(entity); },
                        [scene, entity]() { scene->GetRegistry().remove<ConstraintComponent>(entity); },
                        "Add Constraint Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (!hasSkinnedMesh) {
            // Added empty; the inspector shows a drop slot to assign a rigged glTF.
            if (ImGui::Selectable("Skinned Mesh", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() { scene->GetRegistry().emplace<SkinnedMeshComponent>(entity); },
                        [scene, entity]() { scene->GetRegistry().remove<SkinnedMeshComponent>(entity); },
                        "Add Skinned Mesh Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (canAddAnimator) {
            if (ImGui::Selectable("Animator", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() { scene->GetRegistry().emplace<AnimatorComponent>(entity); },
                        [scene, entity]() { scene->GetRegistry().remove<AnimatorComponent>(entity); },
                        "Add Animator Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (canAddAnimSM) {
            // Drives the Animator from a .animsm asset (auto-adds an Animator too).
            if (ImGui::Selectable("Animator State Machine", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() {
                            auto& r = scene->GetRegistry();
                            r.emplace<AnimStateMachineComponent>(entity);
                            if (!r.all_of<AnimatorComponent>(entity)) r.emplace<AnimatorComponent>(entity);
                        },
                        [scene, entity]() { scene->GetRegistry().remove<AnimStateMachineComponent>(entity); },
                        "Add Animator State Machine Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (!hasCanvas) {
            // Root of a UI tree; full-screen rect that RectTransform children anchor to.
            if (ImGui::Selectable("Canvas", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() { scene->GetRegistry().emplace<CanvasComponent>(entity); },
                        [scene, entity]() { scene->GetRegistry().remove<CanvasComponent>(entity); },
                        "Add Canvas Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (!hasRect) {
            // UI element rect; resolves against its parent (a Canvas or another rect).
            if (ImGui::Selectable("Rect Transform", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() { scene->GetRegistry().emplace<RectTransformComponent>(entity); },
                        [scene, entity]() { scene->GetRegistry().remove<RectTransformComponent>(entity); },
                        "Add Rect Transform Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (canAddUIImage) {
            if (ImGui::Selectable("UI Image", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() { scene->GetRegistry().emplace<UIImageComponent>(entity); },
                        [scene, entity]() { scene->GetRegistry().remove<UIImageComponent>(entity); },
                        "Add UI Image Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (canAddUIText) {
            if (ImGui::Selectable("UI Text", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() { scene->GetRegistry().emplace<UITextComponent>(entity); },
                        [scene, entity]() { scene->GetRegistry().remove<UITextComponent>(entity); },
                        "Add UI Text Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (canAddUIProgress) {
            if (ImGui::Selectable("UI Progress Bar", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() { scene->GetRegistry().emplace<UIProgressBarComponent>(entity); },
                        [scene, entity]() { scene->GetRegistry().remove<UIProgressBarComponent>(entity); },
                        "Add UI Progress Bar Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (canAddUIButton) {
            // Composed widget: a button needs an image background + text label to
            // edit/draw, so add those too if missing.
            if (ImGui::Selectable("Button", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity]() {
                            auto& r = scene->GetRegistry();
                            r.emplace<UIButtonComponent>(entity);
                            if (!r.all_of<UIImageComponent>(entity)) r.emplace<UIImageComponent>(entity);
                            if (!r.all_of<UITextComponent>(entity))  r.emplace<UITextComponent>(entity);
                        },
                        [scene, entity]() { scene->GetRegistry().remove<UIButtonComponent>(entity); },
                        "Add Button Component"));
                    ImGui::CloseCurrentPopup();
                }
        }

        for (const auto& desc : ComponentRegistry::Get().GetAll())
        {
            if (desc.has(*scene, entity)) continue;
            anyShown = true;
            if (ImGui::Selectable(desc.name.c_str(), false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                {
                    auto addFn    = desc.add;
                    auto removeFn = desc.remove;
                    std::string nm = desc.name;
                    m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                        [scene, entity, addFn]()    { addFn(*scene, entity); },
                        [scene, entity, removeFn]() { removeFn(*scene, entity); },
                        "Add " + nm));
                    ImGui::CloseCurrentPopup();
                }
        }

        if (!anyShown)
            ImGui::TextDisabled("(all components already added)");

        ImGui::EndPopup();
    }

    ImGui::End();
}

// ---- multi-entity inspector ---------------------------------------------------
// Shows components common to every selected entity. Fields display the shared
// value, or "--" when the selection disagrees; editing applies to all.

void InspectorPanel::DrawMultiInspector(const std::vector<entt::entity>& ents)
{
    Scene*         scene = m_Context->ActiveScene;
    auto&          reg   = scene->GetRegistry();
    EditorContext* ed    = m_Context;

    ImGui::Text("%zu entities selected", ents.size());

    auto allOf = [&](auto componentTag) {
        using C = decltype(componentTag);
        return std::all_of(ents.begin(), ents.end(),
                           [&](entt::entity e) { return reg.template all_of<C>(e); });
    };

    // -- Transform --------------------------------------------------------------
    if (allOf(TransformComponent{})) {
        ImGui::Separator();
        ImGui::Text("Transform");

        MultiEdit<glm::vec3>(ed, scene, ents,
            FieldGet(&TransformComponent::position), FieldSet(&TransformComponent::position),
            [](glm::vec3& v, bool mixed) {
                return ImGui::DragFloat3("Position", glm::value_ptr(v), 0.1f, 0.0f, 0.0f, mixed ? "--" : "%.3f");
            },
            "Move Entities");

        MultiEdit<glm::vec3>(ed, scene, ents,
            FieldGet(&TransformComponent::eulerDegrees),
            [](Scene* s, entt::entity e, const glm::vec3& v) {
                auto& t        = s->GetRegistry().get<TransformComponent>(e);
                t.eulerDegrees = v;
                t.rotation     = glm::quat(glm::radians(v));
            },
            [](glm::vec3& v, bool mixed) {
                return ImGui::DragFloat3("Rotation", glm::value_ptr(v), 0.5f, 0.0f, 0.0f, mixed ? "--" : "%.3f");
            },
            "Rotate Entities");

        MultiEdit<glm::vec3>(ed, scene, ents,
            FieldGet(&TransformComponent::scale), FieldSet(&TransformComponent::scale),
            [](glm::vec3& v, bool mixed) {
                return ImGui::DragFloat3("Scale", glm::value_ptr(v), 0.1f, 0.001f, 0.0f, mixed ? "--" : "%.3f");
            },
            "Scale Entities");
    }

    // -- Mesh Renderer ------------------------------------------------------------
    if (allOf(MeshComponent{})) {
        ImGui::Separator();
        ImGui::Text("Mesh Renderer");

        MultiEdit<bool>(ed, scene, ents,
            FieldGet(&MeshComponent::visible), FieldSet(&MeshComponent::visible),
            [](bool& v, bool mixed) { return MixedCheckbox("Visible", v, mixed); },
            "Toggle Visibility", true);
        ImGui::SameLine();
        MultiEdit<bool>(ed, scene, ents,
            FieldGet(&MeshComponent::castsShadow), FieldSet(&MeshComponent::castsShadow),
            [](bool& v, bool mixed) { return MixedCheckbox("Cast Shadows", v, mixed); },
            "Toggle Cast Shadows", true);
        ImGui::SameLine();
        MultiEdit<bool>(ed, scene, ents,
            FieldGet(&MeshComponent::receivesShadow), FieldSet(&MeshComponent::receivesShadow),
            [](bool& v, bool mixed) { return MixedCheckbox("Recv Shadows", v, mixed); },
            "Toggle Receive Shadows", true);
        ImGui::SameLine();
        MultiEdit<bool>(ed, scene, ents,
            FieldGet(&MeshComponent::transparent), FieldSet(&MeshComponent::transparent),
            [](bool& v, bool mixed) { return MixedCheckbox("Transparent", v, mixed); },
            "Toggle Transparent", true);

        ImGui::TextDisabled("(mesh & material: select a single entity)");
    }

    // -- Light --------------------------------------------------------------------
    if (allOf(LightComponent{})) {
        ImGui::Separator();
        ImGui::Text("Light");

        MultiEdit<LightType>(ed, scene, ents,
            FieldGet(&LightComponent::type), FieldSet(&LightComponent::type),
            [](LightType& v, bool mixed) {
                const char* items[] = { "Sun", "Point", "Spot" };
                bool changed = false;
                if (ImGui::BeginCombo("Type", mixed ? "--" : items[(int)v])) {
                    for (int i = 0; i < 3; ++i)
                        if (ImGui::Selectable(items[i], !mixed && (int)v == i)) {
                            v = (LightType)i;
                            changed = true;
                        }
                    ImGui::EndCombo();
                }
                return changed;
            },
            "Change Light Type", true);

        MultiEdit<glm::vec3>(ed, scene, ents,
            FieldGet(&LightComponent::color), FieldSet(&LightComponent::color),
            [](glm::vec3& v, bool) { return ImGui::ColorEdit3("Color", glm::value_ptr(v)); },
            "Change Light Color");

        MultiEdit<float>(ed, scene, ents,
            FieldGet(&LightComponent::intensity), FieldSet(&LightComponent::intensity),
            [](float& v, bool mixed) {
                return ImGui::DragFloat("Intensity", &v, 1.0f, 0.0f, 10000.0f, mixed ? "--" : "%.1f");
            },
            "Change Light Intensity");

        // Type-dependent fields only make sense when the type is uniform.
        LightType t0 = reg.get<LightComponent>(ents[0]).type;
        bool typeUniform = std::all_of(ents.begin(), ents.end(),
            [&](entt::entity e) { return reg.get<LightComponent>(e).type == t0; });

        if (typeUniform && (t0 == LightType::Point || t0 == LightType::Spot)) {
            MultiEdit<float>(ed, scene, ents,
                FieldGet(&LightComponent::radius), FieldSet(&LightComponent::radius),
                [](float& v, bool mixed) {
                    return ImGui::DragFloat("Radius", &v, 0.1f, 0.1f, 1000.0f, mixed ? "--" : "%.1f");
                },
                "Change Light Radius");
        }
        if (typeUniform && t0 == LightType::Spot) {
            MultiEdit<float>(ed, scene, ents,
                FieldGet(&LightComponent::innerConeAngle), FieldSet(&LightComponent::innerConeAngle),
                [](float& v, bool mixed) {
                    return ImGui::DragFloat("Inner Cone", &v, 0.5f, 0.5f, 89.0f, mixed ? "--" : "%.1f deg");
                },
                "Change Inner Cone");
            MultiEdit<float>(ed, scene, ents,
                FieldGet(&LightComponent::outerConeAngle), FieldSet(&LightComponent::outerConeAngle),
                [](float& v, bool mixed) {
                    return ImGui::DragFloat("Outer Cone", &v, 0.5f, 1.0f, 90.0f, mixed ? "--" : "%.1f deg");
                },
                "Change Outer Cone");
        }
    }

    // -- Camera ---------------------------------------------------------------------
    if (allOf(CameraComponent{})) {
        ImGui::Separator();
        ImGui::Text("Camera");

        MultiEdit<bool>(ed, scene, ents,
            FieldGet(&CameraComponent::isPrimary), FieldSet(&CameraComponent::isPrimary),
            [](bool& v, bool mixed) { return MixedCheckbox("Primary Camera", v, mixed); },
            "Toggle Primary Camera", true);

        MultiEdit<float>(ed, scene, ents,
            FieldGet(&CameraComponent::fov), FieldSet(&CameraComponent::fov),
            [](float& v, bool mixed) {
                return ImGui::DragFloat("FOV", &v, 0.5f, 1.0f, 179.0f, mixed ? "--" : "%.1f deg");
            },
            "Change Camera FOV");

        MultiEdit<float>(ed, scene, ents,
            FieldGet(&CameraComponent::nearClip), FieldSet(&CameraComponent::nearClip),
            [](float& v, bool mixed) {
                return ImGui::DragFloat("Near Clip", &v, 0.01f, 0.001f, 10000.0f, mixed ? "--" : "%.3f");
            },
            "Change Near Clip");

        MultiEdit<float>(ed, scene, ents,
            FieldGet(&CameraComponent::farClip), FieldSet(&CameraComponent::farClip),
            [](float& v, bool mixed) {
                return ImGui::DragFloat("Far Clip", &v, 1.0f, 0.01f, 100000.0f, mixed ? "--" : "%.1f");
            },
            "Change Far Clip");
    }

    // -- Collider ---------------------------------------------------------------------
    if (allOf(ColliderComponent{})) {
        ImGui::Separator();
        ImGui::Text("Collider");

        MultiEdit<CollisionShape>(ed, scene, ents,
            FieldGet(&ColliderComponent::shapeType), FieldSet(&ColliderComponent::shapeType),
            [](CollisionShape& v, bool mixed) {
                const char* items[] = { "Box", "Sphere", "Capsule", "Convex Hull", "Triangle Mesh" };
                bool changed = false;
                if (ImGui::BeginCombo("Shape", mixed ? "--" : items[(int)v])) {
                    for (int i = 0; i < 5; ++i)
                        if (ImGui::Selectable(items[i], !mixed && (int)v == i)) {
                            v = (CollisionShape)i;
                            changed = true;
                        }
                    ImGui::EndCombo();
                }
                return changed;
            },
            "Change Collision Shape", true);

        CollisionShape s0 = reg.get<ColliderComponent>(ents[0]).shapeType;
        bool shapeUniform = std::all_of(ents.begin(), ents.end(),
            [&](entt::entity e) { return reg.get<ColliderComponent>(e).shapeType == s0; });

        if (shapeUniform) {
            switch (s0) {
                case CollisionShape::Box:
                    MultiEdit<glm::vec3>(ed, scene, ents,
                        FieldGet(&ColliderComponent::halfExtents), FieldSet(&ColliderComponent::halfExtents),
                        [](glm::vec3& v, bool mixed) {
                            return ImGui::DragFloat3("Half Extents", glm::value_ptr(v), 0.01f, 0.001f, 1000.0f, mixed ? "--" : "%.3f");
                        },
                        "Change Half Extents");
                    break;
                case CollisionShape::Capsule:
                    MultiEdit<float>(ed, scene, ents,
                        FieldGet(&ColliderComponent::halfHeight), FieldSet(&ColliderComponent::halfHeight),
                        [](float& v, bool mixed) {
                            return ImGui::DragFloat("Half Height", &v, 0.01f, 0.001f, 1000.0f, mixed ? "--" : "%.3f");
                        },
                        "Change Half Height");
                    [[fallthrough]];
                case CollisionShape::Sphere:
                    MultiEdit<float>(ed, scene, ents,
                        FieldGet(&ColliderComponent::radius), FieldSet(&ColliderComponent::radius),
                        [](float& v, bool mixed) {
                            return ImGui::DragFloat("Radius", &v, 0.01f, 0.001f, 1000.0f, mixed ? "--" : "%.3f");
                        },
                        "Change Radius");
                    break;
                default:
                    ImGui::TextDisabled("(mesh shapes require asset pipeline — not yet editable)");
                    break;
            }
        }

        MultiEdit<glm::vec3>(ed, scene, ents,
            FieldGet(&ColliderComponent::localOffset), FieldSet(&ColliderComponent::localOffset),
            [](glm::vec3& v, bool mixed) {
                return ImGui::DragFloat3("Local Offset", glm::value_ptr(v), 0.01f, 0.0f, 0.0f, mixed ? "--" : "%.3f");
            },
            "Change Collider Offset");

        MultiEdit<bool>(ed, scene, ents,
            FieldGet(&ColliderComponent::isTrigger), FieldSet(&ColliderComponent::isTrigger),
            [](bool& v, bool mixed) { return MixedCheckbox("Is Trigger", v, mixed); },
            "Toggle Is Trigger", true);

        ImGui::TextDisabled("(physics material: select a single entity)");
    }

    // -- Rigidbody ----------------------------------------------------------------------
    if (allOf(RigidBodyComponent{})) {
        ImGui::Separator();
        ImGui::Text("Rigidbody");

        MultiEdit<BodyType>(ed, scene, ents,
            FieldGet(&RigidBodyComponent::bodyType), FieldSet(&RigidBodyComponent::bodyType),
            [](BodyType& v, bool mixed) {
                const char* items[] = { "Static", "Dynamic", "Kinematic" };
                bool changed = false;
                if (ImGui::BeginCombo("Body Type", mixed ? "--" : items[(int)v])) {
                    for (int i = 0; i < 3; ++i)
                        if (ImGui::Selectable(items[i], !mixed && (int)v == i)) {
                            v = (BodyType)i;
                            changed = true;
                        }
                    ImGui::EndCombo();
                }
                return changed;
            },
            "Change Body Type", true);

        BodyType b0 = reg.get<RigidBodyComponent>(ents[0]).bodyType;
        bool typeUniform = std::all_of(ents.begin(), ents.end(),
            [&](entt::entity e) { return reg.get<RigidBodyComponent>(e).bodyType == b0; });

        if (typeUniform && b0 == BodyType::Dynamic) {
            MultiEdit<float>(ed, scene, ents,
                FieldGet(&RigidBodyComponent::mass), FieldSet(&RigidBodyComponent::mass),
                [](float& v, bool mixed) {
                    return ImGui::DragFloat("Mass", &v, 0.1f, 0.001f, 100000.0f, mixed ? "--" : "%.3f");
                },
                "Change Mass");
            MultiEdit<float>(ed, scene, ents,
                FieldGet(&RigidBodyComponent::gravityScale), FieldSet(&RigidBodyComponent::gravityScale),
                [](float& v, bool mixed) {
                    return ImGui::DragFloat("Gravity Scale", &v, 0.01f, 0.0f, 10.0f, mixed ? "--" : "%.3f");
                },
                "Change Gravity Scale");
        }

        if (typeUniform && b0 != BodyType::Static) {
            MultiEdit<float>(ed, scene, ents,
                FieldGet(&RigidBodyComponent::linearDamping), FieldSet(&RigidBodyComponent::linearDamping),
                [](float& v, bool mixed) {
                    return ImGui::DragFloat("Linear Damping", &v, 0.001f, 0.0f, 1.0f, mixed ? "--" : "%.3f");
                },
                "Change Linear Damping");
            MultiEdit<float>(ed, scene, ents,
                FieldGet(&RigidBodyComponent::angularDamping), FieldSet(&RigidBodyComponent::angularDamping),
                [](float& v, bool mixed) {
                    return ImGui::DragFloat("Angular Damping", &v, 0.001f, 0.0f, 1.0f, mixed ? "--" : "%.3f");
                },
                "Change Angular Damping");

            ImGui::Spacing();
            ImGui::TextDisabled("Freeze Rotation");
            MultiEdit<bool>(ed, scene, ents,
                FieldGet(&RigidBodyComponent::lockRotX), FieldSet(&RigidBodyComponent::lockRotX),
                [](bool& v, bool mixed) { return MixedCheckbox("X##lockRotX", v, mixed); },
                "Toggle Rotation Lock X", true);
            ImGui::SameLine();
            MultiEdit<bool>(ed, scene, ents,
                FieldGet(&RigidBodyComponent::lockRotY), FieldSet(&RigidBodyComponent::lockRotY),
                [](bool& v, bool mixed) { return MixedCheckbox("Y##lockRotY", v, mixed); },
                "Toggle Rotation Lock Y", true);
            ImGui::SameLine();
            MultiEdit<bool>(ed, scene, ents,
                FieldGet(&RigidBodyComponent::lockRotZ), FieldSet(&RigidBodyComponent::lockRotZ),
                [](bool& v, bool mixed) { return MixedCheckbox("Z##lockRotZ", v, mixed); },
                "Toggle Rotation Lock Z", true);

            ImGui::Spacing();
            MultiEdit<bool>(ed, scene, ents,
                FieldGet(&RigidBodyComponent::continuousCollision), FieldSet(&RigidBodyComponent::continuousCollision),
                [](bool& v, bool mixed) { return MixedCheckbox("Continuous Collision (CCD)", v, mixed); },
                "Toggle Continuous Collision", true);
        }
    }

    // -- Skinned Mesh / Animator — playback state is per-entity, edit one at a time ----
    if (allOf(SkinnedMeshComponent{})) {
        ImGui::Separator();
        ImGui::Text("Skinned Mesh");
        ImGui::TextDisabled("(select a single entity to edit)");
    }
    if (allOf(AnimatorComponent{})) {
        ImGui::Separator();
        ImGui::Text("Animator");
        ImGui::TextDisabled("(select a single entity to edit)");
    }
    if (allOf(AnimStateMachineComponent{})) {
        ImGui::Separator();
        ImGui::Text("Animator State Machine");
        ImGui::TextDisabled("(select a single entity to edit)");
    }

    // -- Script components — listed when shared; editing needs single selection ----
    for (const auto& desc : ComponentRegistry::Get().GetAll()) {
        bool common = std::all_of(ents.begin(), ents.end(),
            [&](entt::entity e) { return desc.has(*scene, e); });
        if (!common) continue;

        ImGui::Separator();
        ImGui::Text("%s", desc.name.c_str());
        ImGui::TextDisabled("(multi-edit not supported for script components)");
    }
}
