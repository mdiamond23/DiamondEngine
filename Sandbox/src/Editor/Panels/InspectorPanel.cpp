#include "InspectorPanel.h"
#include "ContentPanel.h"
#include <imgui.h>
#include <glm/gtc/type_ptr.hpp>
#include <cstring>
#include "Scene/Components.h"
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

static void DrawMeshSlot(MeshComponent& mc, ContentPanel* cp) {
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
                    mc.mesh         = Mesh::Create(meshes[0]);
                    mc.localBounds  = meshes[0].ComputeAABB();
                    mc.meshPath     = path;
                    mc.meshSubIndex = 0;
                    if (!mc.material)
                        mc.material = std::make_shared<PBRMaterial>();
                }
            }
        }
        ImGui::EndDragDropTarget();
    }
}

// ---- texture row ------------------------------------------------------------
// [24×24 thumbnail]  Label  filename  [x]
// The thumbnail square and the filename text are both drag-drop targets.

static void DrawTextureRow(
    const char*                      label,
    std::shared_ptr<Texture>&        texPtr,
    std::string&                     texPath,
    ContentPanel*                    cp)
{
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
                texPtr  = Texture::Create(path, false);
                texPath = path;
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
            texPtr.reset();
            texPath.clear();
        }
    }

    ImGui::PopID();
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

        if (ImGui::DragFloat3("Rotation", glm::value_ptr(tc.eulerDegrees), 0.5f))
            tc.rotation = glm::quat(glm::radians(tc.eulerDegrees));

        ImGui::DragFloat3("Scale", glm::value_ptr(tc.scale), 0.1f, 0.001f);
    }

    bool removeMesh  = false;
    bool removeLight = false;

    // Mesh Component
    if (registry.all_of<MeshComponent>(entity)) {
        ImGui::Separator();
        auto& mc = registry.get<MeshComponent>(entity);

        ImGui::Text("Mesh Renderer");
        if (ImGui::BeginPopupContextItem("##MeshCompCtx")) {
            if (ImGui::MenuItem("Remove Component"))
                removeMesh = true;
            ImGui::EndPopup();
        }

        if (!removeMesh) {
            ImGui::Checkbox("Visible",         &mc.visible);
            ImGui::SameLine();
            ImGui::Checkbox("Cast Shadows",    &mc.castsShadow);
            ImGui::SameLine();
            ImGui::Checkbox("Recv Shadows",    &mc.receivesShadow);

            // -- Mesh slot --
            ImGui::Spacing();
            ImGui::TextDisabled("Mesh");
            ImGui::Spacing();
            DrawMeshSlot(mc, m_ContentPanel);

            // -- Material --
            ImGui::Spacing();
            ImGui::TextDisabled("Material");

            if (!mc.material)
                mc.material = std::make_shared<PBRMaterial>();

            ImGui::DragFloat("UV Scale",          &mc.material->UVScale,          0.01f, 0.01f,  64.0f);
            ImGui::DragFloat("Emissive Strength", &mc.material->EmissiveStrength, 0.01f, 0.0f,  100.0f);

            ImGui::Spacing();
            ImGui::TextDisabled("Textures");
            ImGui::Separator();

            DrawTextureRow("Albedo",    mc.material->Albedo,    mc.material->AlbedoPath,    m_ContentPanel);
            DrawTextureRow("Normal",    mc.material->Normal,    mc.material->NormalPath,    m_ContentPanel);
            DrawTextureRow("Metallic",  mc.material->Metallic,  mc.material->MetallicPath,  m_ContentPanel);
            DrawTextureRow("Roughness", mc.material->Roughness, mc.material->RoughnessPath, m_ContentPanel);
            DrawTextureRow("AO",        mc.material->AO,        mc.material->AOPath,        m_ContentPanel);
            DrawTextureRow("Emissive",  mc.material->Emissive,  mc.material->EmissivePath,  m_ContentPanel);
        }
    }

    // Light Component
    if (registry.all_of<LightComponent>(entity)) {
        ImGui::Separator();
        auto& lc = registry.get<LightComponent>(entity);

        ImGui::Text("Light");
        if (ImGui::BeginPopupContextItem("##LightCompCtx")) {
            if (ImGui::MenuItem("Remove Component"))
                removeLight = true;
            ImGui::EndPopup();
        }

        if (!removeLight) {
            const char* types[] = { "Sun", "Point", "Spot" };
            int typeIdx = (int)lc.type;
            if (ImGui::Combo("Type", &typeIdx, types, 3))
                lc.type = (LightType)typeIdx;

            ImGui::ColorEdit3("Color", glm::value_ptr(lc.color));
            ImGui::DragFloat("Intensity", &lc.intensity, 1.0f, 0.0f, 10000.0f, "%.1f");

            if (lc.type == LightType::Point || lc.type == LightType::Spot)
                ImGui::DragFloat("Radius", &lc.radius, 0.1f, 0.1f, 1000.0f, "%.1f");

            if (lc.type == LightType::Spot) {
                ImGui::DragFloat("Inner Cone", &lc.innerConeAngle, 0.5f, 0.5f, 89.0f, "%.1f deg");
                lc.outerConeAngle = std::max(lc.outerConeAngle, lc.innerConeAngle + 0.5f);
                ImGui::DragFloat("Outer Cone", &lc.outerConeAngle, 0.5f, 1.0f, 90.0f, "%.1f deg");
                lc.innerConeAngle = std::min(lc.innerConeAngle, lc.outerConeAngle - 0.5f);
            }
        }
    }

    // Deferred removals — must happen after all component UI so no dangling refs
    if (removeMesh)  registry.remove<MeshComponent>(entity);
    if (removeLight) registry.remove<LightComponent>(entity);

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

        bool hasMesh  = registry.all_of<MeshComponent>(entity);
        bool hasLight = registry.all_of<LightComponent>(entity);
        bool anyShown = !hasMesh || !hasLight;

        constexpr ImGuiSelectableFlags kCompFlags =
            ImGuiSelectableFlags_AllowDoubleClick | ImGuiSelectableFlags_DontClosePopups;

        if (!hasMesh) {
            if (ImGui::Selectable("Mesh Renderer", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    auto& mc    = registry.emplace<MeshComponent>(entity);
                    mc.material = std::make_shared<PBRMaterial>();
                    ImGui::CloseCurrentPopup();
                }
        }

        if (!hasLight) {
            if (ImGui::Selectable("Light", false, kCompFlags))
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    registry.emplace<LightComponent>(entity);
                    ImGui::CloseCurrentPopup();
                }
        }

        if (!anyShown)
            ImGui::TextDisabled("(all components already added)");

        ImGui::EndPopup();
    }

    ImGui::End();
}
