#include <imgui_internal.h>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <cctype>
#include "EditorLayers.h"
#include "SceneSerializer.h"
#include "Scene/Components.h"
#include <IconsFontAwesome5.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>

// Converts "enemy health" or "enemy_health" → "EnemyHealth"
static std::string ToClassName(const char* input)
{
    std::string result;
    bool capitalize = true;
    for (char c : std::string(input)) {
        if (c == ' ' || c == '_' || c == '-') {
            capitalize = true;
        } else if (capitalize) {
            result += (char)std::toupper((unsigned char)c);
            capitalize = false;
        } else {
            result += c;
        }
    }
    return result;
}

// Writes Scripts/{ClassName}.h and appends its include to AllScripts.h.
// Returns false if the file already exists.
static bool CreateScriptFile(const std::string& className)
{
    namespace fs = std::filesystem;
    std::string dir      = SCRIPTS_DIR;
    std::string filePath = dir + "/" + className + ".h";
    if (fs::exists(filePath)) return false;

    {
        std::ofstream f(filePath);
        f << "#pragma once\n"
             "#include \"Scene/Scripting.h\"\n"
             "#include \"Scene/Scene.h\"\n"
             "#include \"Scene/ComponentRegistry.h\"\n"
             "#include <imgui.h>\n"
             "#include <nlohmann/json.hpp>\n"
             "#include <spdlog/spdlog.h>\n"
             "\n"
             "// ---- Data -------------------------------------------------------------------\n"
             "\n"
             "struct " << className << "Component\n"
             "{\n"
             "    // TODO: Add component fields here\n"
             "};\n"
             "\n"
             "// ---- Inspector UI -----------------------------------------------------------\n"
             "\n"
             "template<>\n"
             "inline void DrawComponentInspector<" << className << "Component>(" << className << "Component& c)\n"
             "{\n"
             "    // TODO: Add ImGui fields here\n"
             "    ImGui::Text(\"" << className << "\");\n"
             "}\n"
             "\n"
             "// ---- Serialization ----------------------------------------------------------\n"
             "\n"
             "template<>\n"
             "inline std::string SerializeComponent<" << className << "Component>(const " << className << "Component& c)\n"
             "{\n"
             "    nlohmann::json j;\n"
             "    // TODO: j[\"field\"] = c.field;\n"
             "    return j.dump();\n"
             "}\n"
             "\n"
             "template<>\n"
             "inline void DeserializeComponent<" << className << "Component>(" << className << "Component& c, const std::string& data)\n"
             "{\n"
             "    auto j = nlohmann::json::parse(data);\n"
             "    // TODO: c.field = j.value(\"field\", defaultValue);\n"
             "}\n"
             "\n"
             "// ---- Registration -----------------------------------------------------------\n"
             "\n"
             "DECLARE_COMPONENT(" << className << "Component, \"" << className << "\")\n"
             "\n"
             "// ---- Behavior ---------------------------------------------------------------\n"
             "\n"
             "class " << className << "System : public GameSystem\n"
             "{\n"
             "    DECLARE_SYSTEM(" << className << "System, 100)\n"
             "public:\n"
             "    void OnStart(Scene& scene) override {}\n"
             "\n"
             "    void OnUpdate(Scene& scene, float dt) override\n"
             "    {\n"
             "        for (auto [entity, comp] : scene.View<" << className << "Component>().each())\n"
             "        {\n"
             "            // TODO: Add behavior here\n"
             "        }\n"
             "    }\n"
             "\n"
             "    void OnDestroy(Scene& scene) override {}\n"
             "};\n";
    }

    std::ofstream manifest(dir + "/AllScripts.h", std::ios::app);
    manifest << "#include \"" << className << ".h\"\n";
    return true;
}

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

EditorLayer::EditorLayer(Scene* scene, ImFont* iconFont)
{
    m_Context.ActiveScene = scene;
    m_Context.IconFont    = iconFont;
    m_Context.Commands.SetConsole(&m_Console);
    m_Hierarchy.SetContext(&m_Context);
    m_Inspector.SetContext(&m_Context);
    m_Inspector.SetContentPanel(&m_Content);
    m_Viewport.SetContext(&m_Context);
    m_GameViewport.SetContext(&m_Context);
    m_Animator.SetContext(&m_Context);
    m_ParticlePreview.SetContext(&m_Context);

    m_Content.SetOnSceneOpen([this](const std::string& path) {
        m_Context.ClearSelection();
        m_Context.Commands.Clear();
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
    ImGui::DockBuilderDockWindow("Game", center);
    ImGui::DockBuilderDockWindow("Particle Preview", center);
    ImGui::DockBuilderDockWindow("Inspector", right);
    ImGui::DockBuilderDockWindow("Content Browser", down);
    ImGui::DockBuilderDockWindow("Console", down);
    ImGui::DockBuilderDockWindow("Animator", down);
    ImGui::DockBuilderFinish(id);
}

void EditorLayer::SetViewportTexture(uint32_t textureID)
{
    m_Viewport.SetTexture(textureID);
}

void EditorLayer::SetGameViewportTexture(uint32_t textureID)
{
    m_GameViewport.SetTexture(textureID);
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

void EditorLayer::DrawToolbar()
{
    Scene* scene   = m_Context.ActiveScene;
    bool   playing = scene->IsPlaying();
    bool   paused  = scene->IsPaused();

    static constexpr float btnW = 28.0f;
    static constexpr float gap  =  4.0f;

    // When playing we show two buttons (Stop + Pause/Resume), otherwise one.
    float totalW  = playing ? (btnW * 2 + gap) : btnW;
    float centerX = (ImGui::GetWindowWidth() - totalW) * 0.5f;
    ImGui::SetCursorPosX(centerX);

    if (m_Context.IconFont)
        ImGui::PushFont(m_Context.IconFont);

    if (!playing)
    {
        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.18f, 0.60f, 0.18f, 1.00f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.75f, 0.25f, 1.00f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.12f, 0.45f, 0.12f, 1.00f));
        if (ImGui::Button(ICON_FA_PLAY, ImVec2(btnW, 0)))
        {
            m_SceneSnapshot = SceneSerializer::Stringify(*scene);
            scene->StartPlay();
        }
        ImGui::PopStyleColor(3);
    }
    else
    {
        // Stop button
        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.65f, 0.15f, 0.15f, 1.00f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.80f, 0.20f, 0.20f, 1.00f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.50f, 0.10f, 0.10f, 1.00f));
        if (ImGui::Button(ICON_FA_STOP, ImVec2(btnW, 0)))
        {
            scene->StopPlay();
            m_Context.ClearSelection();
            m_Context.Commands.Clear();
            if (!m_SceneSnapshot.empty())
                SceneSerializer::FromString(*scene, m_SceneSnapshot);
        }
        ImGui::PopStyleColor(3);

        ImGui::SameLine(0.0f, gap);

        // Pause / Resume button
        if (paused)
        {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.18f, 0.60f, 0.18f, 1.00f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.75f, 0.25f, 1.00f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.12f, 0.45f, 0.12f, 1.00f));
            if (ImGui::Button(ICON_FA_PLAY, ImVec2(btnW, 0)))
                scene->Resume();
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.70f, 0.55f, 0.05f, 1.00f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.70f, 0.10f, 1.00f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.55f, 0.40f, 0.02f, 1.00f));
            if (ImGui::Button(ICON_FA_PAUSE, ImVec2(btnW, 0)))
                scene->Pause();
        }
        ImGui::PopStyleColor(3);
    }

    if (m_Context.IconFont)
        ImGui::PopFont();
}

void EditorLayer::DrawNewScriptDialog()
{
    if (m_OpenNewScriptDialog)
    {
        ImGui::OpenPopup("New Script");
        m_OpenNewScriptDialog = false;
    }

    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_Appearing);

    if (ImGui::BeginPopupModal("New Script", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Text("Script name:");
        ImGui::SetNextItemWidth(-1.0f);
        bool entered = ImGui::InputText("##scriptname", m_NewScriptNameBuf, sizeof(m_NewScriptNameBuf),
                                        ImGuiInputTextFlags_EnterReturnsTrue);

        std::string preview = ToClassName(m_NewScriptNameBuf);
        if (!preview.empty())
            ImGui::TextDisabled("-> %sComponent / %sSystem", preview.c_str(), preview.c_str());
        else
            ImGui::TextDisabled("Enter a name above");

        if (!m_NewScriptError.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", m_NewScriptError.c_str());

        ImGui::Spacing();

        bool create = entered || ImGui::Button("Create", ImVec2(160, 0));
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(160, 0)))
            ImGui::CloseCurrentPopup();

        if (create)
        {
            std::string name = ToClassName(m_NewScriptNameBuf);
            if (name.empty())
            {
                m_NewScriptError = "Name cannot be empty.";
            }
            else if (!CreateScriptFile(name))
            {
                m_NewScriptError = name + ".h already exists.";
            }
            else
            {
                ImGui::CloseCurrentPopup();
            }
        }

        ImGui::EndPopup();
    }
}

void EditorLayer::DrawMenuBar()
{
    if (!ImGui::BeginMainMenuBar()) return;

    if (ImGui::BeginMenu("Edit")) {
        if (ImGui::MenuItem("Undo", "Ctrl+Z", false, m_Context.Commands.CanUndo()))
            m_Context.Commands.Undo();
        if (ImGui::MenuItem("Redo", "Ctrl+Y", false, m_Context.Commands.CanRedo()))
            m_Context.Commands.Redo();
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Scripts")) {
        if (ImGui::MenuItem("New Script..."))
        {
            m_OpenNewScriptDialog = true;
            memset(m_NewScriptNameBuf, 0, sizeof(m_NewScriptNameBuf));
            m_NewScriptError.clear();
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("New Scene", "Ctrl+N")) {
            m_Context.ClearSelection();
            m_Context.Commands.Clear();
            m_Context.currentScenePath = "";
            m_Context.ActiveScene->Clear();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Open Scene...", "Ctrl+O")) {
            std::string path = OpenFileDialog();
            if (!path.empty()) {
                m_Context.ClearSelection();
                m_Context.Commands.Clear();
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

    DrawToolbar();

    ImGui::EndMainMenuBar();
}

void EditorLayer::OnImGuiRender()
{
    // Undo / Redo keyboard shortcuts
    if (ImGui::IsKeyDown(ImGuiKey_LeftCtrl) || ImGui::IsKeyDown(ImGuiKey_RightCtrl))
    {
        bool shift = ImGui::IsKeyDown(ImGuiKey_LeftShift) || ImGui::IsKeyDown(ImGuiKey_RightShift);
        if (!shift && ImGui::IsKeyPressed(ImGuiKey_Z, false))
            m_Context.Commands.Undo();
        if ((ImGui::IsKeyPressed(ImGuiKey_Y, false)) ||
            (shift && ImGui::IsKeyPressed(ImGuiKey_Z, false)))
            m_Context.Commands.Redo();
    }

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

    DrawNewScriptDialog();

    // Render each panel
    m_Hierarchy.OnImGuiRender();
    m_Viewport.OnImGuiRender();
    m_GameViewport.OnImGuiRender();
    m_Inspector.OnImGuiRender();
    m_Content.OnImGuiRender();
    m_Animator.OnImGuiRender();
    m_Console.OnImGuiRender();
    m_ParticlePreview.OnImGuiRender();
}