#include <imgui_internal.h>
#include <cstdint>
#include <filesystem>
#include "EditorLayers.h"
#include "SceneSerializer.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>

// Returns Assets/Scenes/NewScene.scene, or NewScene (1).scene etc. if taken.
static std::string UniqueNewScenePath()
{
    namespace fs = std::filesystem;
    std::string base = std::string(ASSETS_DIR) + "/Scenes/NewScene";
    std::string path = base + ".scene";
    if (!fs::exists(path)) return path;
    for (int i = 1; ; ++i) {
        path = base + " (" + std::to_string(i) + ").scene";
        if (!fs::exists(path)) return path;
    }
}

EditorLayer::EditorLayer(Scene* scene)
{
    m_Context.ActiveScene = scene;
    m_Hierarchy.SetContext(&m_Context);
    m_Inspector.SetContext(&m_Context);
    m_Inspector.SetContentPanel(&m_Content);
    m_Viewport.SetContext(&m_Context);

    m_Content.SetOnSceneOpen([this](const std::string& path) {
        m_Context.ClearSelection();
        if (SceneSerializer::Load(*m_Context.ActiveScene, path))
            m_Context.currentScenePath = path;
    });
}

void EditorLayer::SetupDockspace()
{
    if (m_LayoutInitialized) return;
    m_LayoutInitialized = true;

    ImGuiID id = ImGui::GetID("MainDockSpace");

    // Set up central dock
    ImGui::DockBuilderRemoveNode(id);
    ImGui::DockBuilderAddNode(id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(id, ImGui::GetMainViewport()->Size);

    // Split dock into 4
    ImGuiID center = id;
    ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.2f, nullptr, &center);
    ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.25f, nullptr, &center);
    ImGuiID down = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.28f, nullptr, &center);

    // Attach and finish dock
    ImGui::DockBuilderDockWindow("Hierarchy", left);
    ImGui::DockBuilderDockWindow("Viewport", center);
    ImGui::DockBuilderDockWindow("Inspector", right);
    ImGui::DockBuilderDockWindow("Content Browser", down);
    ImGui::DockBuilderFinish(id);
}

void EditorLayer::SetViewportTexture(uint32_t textureID)
{
    m_Viewport.SetTexture(textureID);
}

void EditorLayer::UpdateCamera(const glm::mat4& view, const glm::mat4& proj,
                               const glm::vec3& camPos)
{
    m_Context.viewMatrix = view;
    m_Context.projMatrix = proj;
    m_Context.cameraPos  = camPos;
}

std::string EditorLayer::OpenFileDialog()
{
    OPENFILENAMEA ofn{};
    char buf[MAX_PATH]{};
    ofn.lStructSize  = sizeof(ofn);
    ofn.lpstrFilter  = "Scene Files\0*.scene\0All Files\0*.*\0";
    ofn.lpstrFile    = buf;
    ofn.nMaxFile     = MAX_PATH;
    ofn.lpstrInitialDir = ASSETS_DIR "/Scenes";
    ofn.Flags        = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
    return GetOpenFileNameA(&ofn) ? buf : "";
}

std::string EditorLayer::SaveFileDialog()
{
    OPENFILENAMEA ofn{};
    char buf[MAX_PATH]{};
    ofn.lStructSize  = sizeof(ofn);
    ofn.lpstrFilter  = "Scene Files\0*.scene\0All Files\0*.*\0";
    ofn.lpstrFile    = buf;
    ofn.nMaxFile     = MAX_PATH;
    ofn.lpstrDefExt  = "scene";
    ofn.lpstrInitialDir = ASSETS_DIR "/Scenes";
    ofn.Flags        = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;
    return GetSaveFileNameA(&ofn) ? buf : "";
}

void EditorLayer::DrawMenuBar()
{
    if (!ImGui::BeginMainMenuBar()) return;

    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("New Scene", "Ctrl+N")) {
            m_Context.ClearSelection();
            m_Context.currentScenePath = "";
            m_Context.ActiveScene->Clear();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Open Scene...", "Ctrl+O")) {
            std::string path = OpenFileDialog();
            if (!path.empty()) {
                m_Context.ClearSelection();
                if (SceneSerializer::Load(*m_Context.ActiveScene, path))
                    m_Context.currentScenePath = path;
            }
        }
        if (ImGui::MenuItem("Save Scene", "Ctrl+S")) {
            if (m_Context.currentScenePath.empty())
                m_Context.currentScenePath = UniqueNewScenePath();
            SceneSerializer::Save(*m_Context.ActiveScene, m_Context.currentScenePath);
        }
        if (ImGui::MenuItem("Save Scene As...", "Ctrl+Shift+S")) {
            std::string path = SaveFileDialog();
            if (!path.empty()) {
                SceneSerializer::Save(*m_Context.ActiveScene, path);
                m_Context.currentScenePath = path;
            }
        }
        ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
}

void EditorLayer::OnImGuiRender()
{
    // Menu bar first — its height is needed to offset the dockspace host below it
    DrawMenuBar();

    float menuBarH = ImGui::GetFrameHeight();

    // Fullscreen invisible host window — sits below the menu bar
    ImGuiViewport* vp = ImGui::GetMainViewport();
    if (vp)
    {
        ImGui::SetNextWindowPos({ vp->Pos.x, vp->Pos.y + menuBarH });
        ImGui::SetNextWindowSize({ vp->Size.x, vp->Size.y - menuBarH });
        ImGui::SetNextWindowViewport(vp->ID);
    }

    ImGuiWindowFlags hostFlags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize   | ImGuiWindowFlags_NoMove     |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
        ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDocking;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    ImGui::Begin("DockSpaceHost", nullptr, hostFlags);
    ImGui::PopStyleVar();

    ImGuiID dockID = ImGui::GetID("MainDockSpace");
    ImGui::DockSpace(dockID, {0, 0}, ImGuiDockNodeFlags_PassthruCentralNode);
    SetupDockspace();

    ImGui::End();

    // Render each panel
    m_Hierarchy.OnImGuiRender();
    m_Viewport.OnImGuiRender();
    m_Inspector.OnImGuiRender();
    m_Content.OnImGuiRender();
}