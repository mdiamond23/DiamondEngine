#include "Scene/SceneSystem.h"

#include <spdlog/spdlog.h>

#include <cctype>
#include <filesystem>

namespace {

// All main-thread (script OnUpdate + host loop) — no synchronization needed.
std::vector<std::string>  s_Scenes;
uint32_t                  s_Current = SceneSystem::kNoScene;
std::optional<uint32_t>   s_Pending;

// Case-insensitive stem: "Assets/Scenes/Level2.scene" -> "level2".
std::string StemLower(const std::string& path)
{
    std::string s = std::filesystem::path(path).stem().string();
    for (char& c : s) c = (char)std::tolower((unsigned char)c);
    return s;
}

} // namespace

namespace SceneSystem {

void SetSceneList(std::vector<std::string> portablePaths)
{
    s_Scenes  = std::move(portablePaths);
    s_Current = kNoScene;
    s_Pending.reset();
}

const std::vector<std::string>& SceneList() { return s_Scenes; }

bool LoadScene(const std::string& name)
{
    const std::string want = StemLower(name);
    for (uint32_t i = 0; i < (uint32_t)s_Scenes.size(); ++i) {
        if (StemLower(s_Scenes[i]) == want) {
            s_Pending = i;
            return true;
        }
    }
    spdlog::warn("[SceneSystem] LoadScene('{}'): not in the scene list ({} entries) — "
                 "add it to the build's scene list", name, s_Scenes.size());
    return false;
}

bool LoadSceneByIndex(uint32_t index)
{
    if (index >= s_Scenes.size()) {
        spdlog::warn("[SceneSystem] LoadSceneByIndex({}): list has {} entries",
                     index, s_Scenes.size());
        return false;
    }
    s_Pending = index;
    return true;
}

void ReloadCurrentScene()
{
    if (s_Current != kNoScene) s_Pending = s_Current;
    else spdlog::warn("[SceneSystem] ReloadCurrentScene: no current scene");
}

uint32_t CurrentIndex() { return s_Current; }

std::string CurrentName()
{
    if (s_Current == kNoScene || s_Current >= s_Scenes.size()) return {};
    return std::filesystem::path(s_Scenes[s_Current]).stem().string();
}

std::optional<uint32_t> ConsumePendingRequest()
{
    const std::optional<uint32_t> r = s_Pending;
    s_Pending.reset();
    return r;
}

void SetCurrent(uint32_t index) { s_Current = index; }

} // namespace SceneSystem
