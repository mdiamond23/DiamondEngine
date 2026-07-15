#include <imgui_internal.h>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <cctype>
#include "EditorLayers.h"
#include "SceneSerializer.h"
#include "AssetPipeline/TextureCooker.h"
#include "AssetPipeline/AssetRegistry.h"
#include <spdlog/spdlog.h>
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
    m_Profiler.SetContext(&m_Context);

    m_Content.SetOnSceneOpen([this](const std::string& path) {
        // Opening a scene while editing a prefab would be clobbered by the
        // edit-mode exit restore — leave the mode (discarding unsaved prefab
        // edits) before switching scenes.
        if (m_PrefabEditMode) ExitPrefabEdit();
        m_Context.ClearSelection();
        m_Context.Commands.Clear();
        if (SceneSerializer::Load(*m_Context.ActiveScene, path))
            m_Context.currentScenePath = path;
        if (m_SceneCacheInvalidator) m_SceneCacheInvalidator();
    });

    m_Content.SetOnPrefabOpen([this](const std::string& path) { OpenPrefabForEdit(path); });
    m_Content.SetOnModelImport([this](const std::string& path) { ImportModelIntoScene(path); });
    m_Hierarchy.SetOnEditPrefab([this](const std::string& path) { OpenPrefabForEdit(path); });

    // Entity dragged from the Hierarchy into the Content Browser → write its
    // subtree as EntityName.prefab in the drop folder.
    m_Content.SetOnEntityDrop([this](uint32_t entityBits, const std::filesystem::path& destDir) {
        namespace fs = std::filesystem;
        Scene* s = m_Context.ActiveScene;
        entt::entity e = (entt::entity)entityBits;
        if (!s || !s->GetRegistry().valid(e)) return;

        // Entity names are free text — strip characters Windows filenames reject.
        const std::string forbidden = "\\/:*?\"<>|";
        std::string stem;
        for (char c : s->GetEntityName(e))
            if (forbidden.find(c) == std::string::npos)
                stem += c;
        if (stem.empty()) stem = "Entity";

        fs::path path = destDir / (stem + ".prefab");
        for (int n = 1; fs::exists(path); ++n)
            path = destDir / (stem + " (" + std::to_string(n) + ").prefab");

        if (PrefabSerializer::Save(*s, e, path.string())) {
            // Stamp the source entity so "Save to Prefab" can push later edits back.
            s->GetRegistry().emplace_or_replace<PrefabInstanceComponent>(e, path.string());
        }
    });
}

void EditorLayer::ImportModelIntoScene(const std::string& path)
{
    namespace fs = std::filesystem;
    Scene* scene = m_Context.ActiveScene;
    if (!scene) return;

    // First load is the expensive one (geometry + every texture the file's
    // materials reference); redo re-runs this and hits the registry cache.
    auto model = Assets::Load<Diamond::ImportedScene>(path);
    if (!model || model->nodes.empty()) {
        spdlog::error("Import into Scene: '{}' has no importable geometry", path);
        return;
    }

    auto* edCtx     = &m_Context;
    auto sharedRoot = std::make_shared<entt::entity>(entt::null);
    auto doSpawn    = [scene, edCtx, sharedRoot, path]() {
        auto model = Assets::Load<Diamond::ImportedScene>(path);
        if (!model) return;
        auto& reg = scene->GetRegistry();

        entt::entity root = scene->CreateEntity(fs::path(path).stem().string());
        *sharedRoot = root;

        // One copy per glTF material, shared by every entity of this import —
        // same sharing semantics as .mat assets, and it keeps GPU state
        // (Vulkan descriptor sets) at #materials, not #entities. Copied off
        // the registry asset so inspector edits can't mutate the shared cache.
        std::vector<std::shared_ptr<Diamond::PBRMaterial>> mats(model->materials.size());
        auto materialFor = [&](int mi) -> std::shared_ptr<Diamond::PBRMaterial> {
            if (mi < 0 || !model->materials[mi])
                return std::make_shared<Diamond::PBRMaterial>();
            if (!mats[mi])
                mats[mi] = std::make_shared<Diamond::PBRMaterial>(*model->materials[mi]);
            return mats[mi];
        };

        auto addMesh = [&](entt::entity e, int prim) {
            int mi  = model->primitiveMaterial[prim];
            auto mat = materialFor(mi);
            auto& mc = reg.emplace<MeshComponent>(e,
                Diamond::Mesh::Create(model->meshes[prim]), mat,
                model->meshes[prim].ComputeAABB());
            mc.meshPath     = path;
            mc.meshSubIndex = prim;
            if (mi >= 0 && model->materialTransparent[mi])
                mc.transparent = true;
        };

        for (const auto& node : model->nodes) {
            entt::entity nodeEnt = scene->CreateEntity(node.name);
            auto& tc        = reg.get<TransformComponent>(nodeEnt);
            tc.position     = node.position;
            tc.rotation     = node.rotation;
            tc.scale        = node.scale;
            tc.eulerDegrees = glm::degrees(glm::eulerAngles(node.rotation));
            scene->SetParent(nodeEnt, root);

            if (node.primitives.size() == 1) {
                addMesh(nodeEnt, node.primitives[0]);
            } else {
                // Multi-primitive node: identity-transform child per primitive
                // so each keeps its own material and stays selectable.
                for (size_t k = 0; k < node.primitives.size(); ++k) {
                    entt::entity primEnt =
                        scene->CreateEntity(node.name + " [" + std::to_string(k) + "]");
                    scene->SetParent(primEnt, nodeEnt);
                    addMesh(primEnt, node.primitives[k]);
                }
            }
        }
        edCtx->SelectOnly(root);
    };

    doSpawn();
    if (*sharedRoot != entt::null) {
        m_Context.Commands.RecordCommand(std::make_unique<FunctionCommand>(
            doSpawn,
            [scene, edCtx, sharedRoot]() {
                edCtx->ClearSelection();
                if (scene->GetRegistry().valid(*sharedRoot))
                    scene->DestroyEntity(*sharedRoot);
            },
            "Import Model"));
    }
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
    // Renderer docks first so Inspector ends up the selected tab in the node.
    ImGui::DockBuilderDockWindow("Renderer", right);
    ImGui::DockBuilderDockWindow("Inspector", right);
    ImGui::DockBuilderDockWindow("Content Browser", down);
    ImGui::DockBuilderDockWindow("Console", down);
    ImGui::DockBuilderDockWindow("Animator", down);
    ImGui::DockBuilderDockWindow("Mixer", down);
    ImGui::DockBuilderFinish(id);
}

void EditorLayer::SetViewportTexture(ImTextureID textureID, bool flipY)
{
    m_Viewport.SetTexture(textureID, flipY);
}

void EditorLayer::SetGameViewportTexture(ImTextureID textureID, bool flipY)
{
    m_GameViewport.SetTexture(textureID, flipY);
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
            if (m_SceneCacheInvalidator) m_SceneCacheInvalidator();
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

void EditorLayer::SaveScene()
{
    if (m_Context.currentScenePath.empty())
        m_Context.currentScenePath = UniqueNewScenePath();
    SceneSerializer::Save(*m_Context.ActiveScene, m_Context.currentScenePath);
}

void EditorLayer::OpenPrefabForEdit(const std::string& path)
{
    Scene* scene = m_Context.ActiveScene;
    if (scene->IsPlaying()) {
        spdlog::warn("Prefab edit: stop play mode first");
        return;
    }
    if (m_PrefabEditMode) ExitPrefabEdit();   // back to the scene, then re-enter

    std::string snapshot = SceneSerializer::Stringify(*scene);
    m_Context.ClearSelection();
    m_Context.Commands.Clear();
    scene->Clear();

    entt::entity root = PrefabSerializer::Instantiate(*scene, path);
    if (root == entt::null) {
        SceneSerializer::FromString(*scene, snapshot);
        if (m_SceneCacheInvalidator) m_SceneCacheInvalidator();
        return;
    }

    m_PrefabEditSnapshot = std::move(snapshot);
    m_PrefabEditPath     = path;
    m_PrefabEditRootUuid = scene->GetRegistry().get<IDComponent>(root).uuid;
    m_PrefabEditMode     = true;
    m_Context.SelectOnly(root);
    if (m_SceneCacheInvalidator) m_SceneCacheInvalidator();
}

void EditorLayer::SavePrefabEdit()
{
    Scene* scene = m_Context.ActiveScene;
    entt::entity root = scene->FindByUuid(m_PrefabEditRootUuid);
    if (root == entt::null) {
        // The original root was deleted during editing — save the first
        // remaining scene root instead (a prefab is one entity's subtree).
        auto& reg = scene->GetRegistry();
        for (auto& [e, name] : scene->GetEntityNames()) {
            bool isRoot = !reg.all_of<HierarchyComponent>(e)
                       || reg.get<HierarchyComponent>(e).parent == entt::null;
            if (isRoot) { root = e; break; }
        }
    }
    if (root == entt::null) {
        spdlog::error("Prefab save: no entities to save to '{}'", m_PrefabEditPath);
        return;
    }

    // Everything in the edit scene belongs to the prefab, but Save writes only
    // the root's subtree — entities added at scene-root level ("+ Add Entity")
    // would be silently dropped. Adopt them as children of the prefab root.
    auto& reg = scene->GetRegistry();
    std::vector<entt::entity> strays;
    for (auto& [e, name] : scene->GetEntityNames()) {
        if (e == root) continue;
        bool isRoot = !reg.all_of<HierarchyComponent>(e)
                   || reg.get<HierarchyComponent>(e).parent == entt::null;
        if (isRoot) strays.push_back(e);
    }
    for (entt::entity e : strays) {
        scene->SetParent(e, root);
        spdlog::info("Prefab save: parented stray root '{}' under the prefab root",
                     scene->GetEntityName(e));
    }

    m_PrefabEditRootUuid = reg.get<IDComponent>(root).uuid;
    if (PrefabSerializer::Save(*scene, root, m_PrefabEditPath))
        spdlog::info("Saved prefab '{}'", m_PrefabEditPath);
}

void EditorLayer::ExitPrefabEdit()
{
    if (!m_PrefabEditMode) return;
    Scene* scene = m_Context.ActiveScene;
    if (scene->IsPlaying()) scene->StopPlay();
    m_Context.ClearSelection();
    m_Context.Commands.Clear();
    SceneSerializer::FromString(*scene, m_PrefabEditSnapshot);
    m_PrefabEditSnapshot.clear();
    m_PrefabEditMode = false;
    if (m_SceneCacheInvalidator) m_SceneCacheInvalidator();
}

float EditorLayer::DrawPrefabEditBanner(float y)
{
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float h = ImGui::GetFrameHeight() + 8.0f;
    if (vp) {
        ImGui::SetNextWindowPos({ vp->Pos.x, vp->Pos.y + y });
        ImGui::SetNextWindowSize({ vp->Size.x, h });
        ImGui::SetNextWindowViewport(vp->ID);
    }
    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoDocking  | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.10f, 0.22f, 0.42f, 1.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, { 8.0f, 4.0f });
    ImGui::Begin("##PrefabEditBanner", nullptr, flags);

    std::string fname = std::filesystem::path(m_PrefabEditPath).filename().string();
    ImGui::Text("Editing Prefab: %s", fname.c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("(Ctrl+S saves to the .prefab)");

    float btnW  = 100.0f;
    float gap   = ImGui::GetStyle().ItemSpacing.x;
    ImGui::SameLine(ImGui::GetWindowWidth() - (btnW * 3 + gap * 2) - 8.0f);
    if (ImGui::Button("Save", { btnW, 0 })) SavePrefabEdit();
    ImGui::SameLine();
    if (ImGui::Button("Save & Close", { btnW, 0 })) { SavePrefabEdit(); ExitPrefabEdit(); }
    ImGui::SameLine();
    if (ImGui::Button("Close", { btnW, 0 })) ExitPrefabEdit();

    ImGui::End();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
    return h;
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

    if (ImGui::BeginMenu("Assets")) {
        if (ImGui::MenuItem("Cook Textures")) {
            const int n = Diamond::TextureCooker::CookAll();
            spdlog::info("[Assets] Cook Textures: {} cooked — press F5 to reload them now", n);
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("File")) {
        // Scene file ops are disabled while editing a prefab — the scene on
        // screen is the prefab's content, not the scene the paths refer to.
        bool sceneOps = !m_PrefabEditMode;
        if (ImGui::MenuItem("New Scene", "Ctrl+N", false, sceneOps)) {
            m_Context.ClearSelection();
            m_Context.Commands.Clear();
            m_Context.currentScenePath = "";
            m_Context.ActiveScene->Clear();
            if (m_SceneCacheInvalidator) m_SceneCacheInvalidator();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Open Scene...", "Ctrl+O", false, sceneOps)) {
            std::string path = OpenFileDialog();
            if (!path.empty()) {
                m_Context.ClearSelection();
                m_Context.Commands.Clear();
                if (SceneSerializer::Load(*m_Context.ActiveScene, path))
                    m_Context.currentScenePath = path;
                if (m_SceneCacheInvalidator) m_SceneCacheInvalidator();
            }
        }
        if (ImGui::MenuItem("Save Scene", "Ctrl+S", false, sceneOps))
            SaveScene();
        if (ImGui::MenuItem("Save Scene As...", "Ctrl+Shift+S", false, sceneOps)) {
            std::string path = SaveFileDialog();
            if (!path.empty()) {
                SceneSerializer::Save(*m_Context.ActiveScene, path);
                m_Context.currentScenePath = path;
            }
        }
        if (m_PrefabEditMode) {
            ImGui::Separator();
            if (ImGui::MenuItem("Save Prefab", "Ctrl+S"))
                SavePrefabEdit();
            if (ImGui::MenuItem("Close Prefab"))
                ExitPrefabEdit();
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View")) {
        for (Panel* panel : m_Panels)
            ImGui::MenuItem(panel->GetName(), nullptr, &panel->Open());
        ImGui::EndMenu();
    }

    DrawToolbar();

    ImGui::EndMainMenuBar();
}

void EditorLayer::OnImGuiRender()
{
    // Undo / Redo / Save keyboard shortcuts
    if (ImGui::IsKeyDown(ImGuiKey_LeftCtrl) || ImGui::IsKeyDown(ImGuiKey_RightCtrl))
    {
        bool shift = ImGui::IsKeyDown(ImGuiKey_LeftShift) || ImGui::IsKeyDown(ImGuiKey_RightShift);
        if (!shift && ImGui::IsKeyPressed(ImGuiKey_Z, false))
            m_Context.Commands.Undo();
        if ((ImGui::IsKeyPressed(ImGuiKey_Y, false)) ||
            (shift && ImGui::IsKeyPressed(ImGuiKey_Z, false)))
            m_Context.Commands.Redo();
        if (ImGui::IsKeyPressed(ImGuiKey_S, false))
        {
            // Runtime state isn't worth persisting — ignore saves during play.
            if (!m_Context.ActiveScene->IsPlaying())
            {
                if (m_PrefabEditMode)
                    SavePrefabEdit();
                else if (shift) {
                    std::string path = SaveFileDialog();
                    if (!path.empty()) {
                        SceneSerializer::Save(*m_Context.ActiveScene, path);
                        m_Context.currentScenePath = path;
                    }
                } else {
                    SaveScene();
                }
            }
        }
    }

    // Menu bar first — its height is needed to offset the dockspace host below it
    DrawMenuBar();

    float topOffset = ImGui::GetFrameHeight();
    if (m_PrefabEditMode)
        topOffset += DrawPrefabEditBanner(topOffset);

    // Fullscreen invisible host window — sits below the menu bar (and banner)
    ImGuiViewport* vp = ImGui::GetMainViewport();
    if (vp)
    {
        ImGui::SetNextWindowPos({ vp->Pos.x, vp->Pos.y + topOffset });
        ImGui::SetNextWindowSize({ vp->Size.x, vp->Size.y - topOffset });
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
    for (Panel* panel : m_Panels)
        panel->OnImGuiRender();
}