#include "InspectorPanel.h"
#include "ContentPanel.h"
#include "../Command.h"
#include <imgui.h>
#include <glm/gtc/type_ptr.hpp>
#include <cstring>
#include "Scene/Components.h"
#include "Scene/ComponentRegistry.h"
#include "Scene/Physics/Collision.h"
#include "Scene/Physics/Rigidbody.h"
#include "Assets/ModelImporter.h"
#include "Renderer/MeshData.h"
#include "Renderer/TextureData.h"
#include <algorithm>

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

    uint32_t thumbID = (cp && !mc.meshPath.empty())
                       ? cp->GetThumbnail(mc.meshPath, AssetType::Mesh)
                       : 0;

    if (thumbID)
        dl->AddImage((ImTextureID)(uintptr_t)thumbID, tMin, tMax);
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
    uint32_t thumbID = (cp && !texPath.empty())
                       ? cp->GetThumbnail(texPath, AssetType::Texture)
                       : 0;

    if (thumbID)
        dl->AddImage((ImTextureID)(uintptr_t)thumbID, p, {p.x + sz, p.y + sz});
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

// ---- panel ------------------------------------------------------------------

void InspectorPanel::OnImGuiRender() {
    ImGui::Begin("Inspector");

    if (!m_Context || !m_Context->HasSelection()) {
        ImGui::Text("(nothing selected)");
        ImGui::End();
        return;
    }

    if (m_Context->selectedEntities.size() > 1) {
        ImGui::Text("%zu entities selected", m_Context->selectedEntities.size());
        ImGui::End();
        return;
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

    bool removeMesh      = false;
    bool removeLight     = false;
    bool removeCamera    = false;
    bool removeCollider  = false;
    bool removeRigidbody = false;

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

            // -- Material --
            ImGui::Spacing();
            ImGui::TextDisabled("Material");

            if (!mc.material)
                mc.material = std::make_shared<PBRMaterial>();

            ImGui::DragFloat("UV Scale", &mc.material->UVScale, 0.01f, 0.01f, 64.0f);
            if (ImGui::IsItemActivated())   m_OldUVScale = mc.material->UVScale;
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                float newVal = mc.material->UVScale, oldVal = m_OldUVScale;
                m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                    [scene, entity, newVal]() { scene->GetRegistry().get<MeshComponent>(entity).material->UVScale = newVal; },
                    [scene, entity, oldVal]() { scene->GetRegistry().get<MeshComponent>(entity).material->UVScale = oldVal; },
                    "Change UV Scale"));
            }

            ImGui::DragFloat("Emissive Strength", &mc.material->EmissiveStrength, 0.01f, 0.0f, 100.0f);
            if (ImGui::IsItemActivated())   m_OldEmissiveStrength = mc.material->EmissiveStrength;
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                float newVal = mc.material->EmissiveStrength, oldVal = m_OldEmissiveStrength;
                m_Context->Commands.ExecuteCommand(std::make_unique<FunctionCommand>(
                    [scene, entity, newVal]() { scene->GetRegistry().get<MeshComponent>(entity).material->EmissiveStrength = newVal; },
                    [scene, entity, oldVal]() { scene->GetRegistry().get<MeshComponent>(entity).material->EmissiveStrength = oldVal; },
                    "Change Emissive Strength"));
            }

            ImGui::Spacing();
            ImGui::TextDisabled("Textures");
            ImGui::Separator();

            // Helper: draw a texture row and submit an undo command if it changed.
            // Uses pointer-to-member so all 6 slots share one lambda body.
            using TexMem  = std::shared_ptr<Texture> PBRMaterial::*;
            using PathMem = std::string              PBRMaterial::*;
            auto texRow = [&](const char* label,
                               std::shared_ptr<Texture>& texPtr, std::string& texPath,
                               TexMem tm, PathMem pm, const char* desc) {
                auto r = DrawTextureRow(label, texPtr, texPath, m_ContentPanel);
                if (!r.changed) return;
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

            ImGui::Spacing();
            ImGui::TextDisabled("Physics Material");
            if (!col.material) {
                ImGui::TextDisabled("None");
                ImGui::SameLine();
                if (ImGui::SmallButton("Create"))
                    col.material = std::make_shared<PhysicsMaterial>();
            } else {
                ImGui::DragFloat("Static Friction",  &col.material->staticFriction,  0.01f, 0.0f, 1.0f);
                ImGui::DragFloat("Dynamic Friction", &col.material->dynamicFriction, 0.01f, 0.0f, 1.0f);
                ImGui::DragFloat("Restitution",      &col.material->restitution,     0.01f, 0.0f, 1.0f);
                if (ImGui::SmallButton("Remove Material")) col.material.reset();
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
            }
        }
    }

    // Registered game components
    const ComponentDescriptor* removeUserComp = nullptr;

    for (const auto& desc : ComponentRegistry::Get().GetAll())
    {
        if (!desc.has(*scene, entity)) continue;

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

        bool hasMesh      = registry.all_of<MeshComponent>(entity);
        bool hasLight     = registry.all_of<LightComponent>(entity);
        bool hasCamera    = registry.all_of<CameraComponent>(entity);
        bool hasCollider  = registry.all_of<ColliderComponent>(entity);
        bool hasRigidbody = registry.all_of<RigidBodyComponent>(entity);
        bool anyShown     = !hasMesh || !hasLight || !hasCamera || !hasCollider || !hasRigidbody;

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
