#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// Scene transitions (Milestone 7). Scripts REQUEST a scene by name or index;
// the HOST loop (Runtime main / editor play loop) consumes the request at the
// top of the next frame and performs the actual swap. Deferral is not an
// optimization, it's the safety model: executing a load mid-UpdateSystems
// would destroy the registry the other systems are iterating — and the
// systems themselves, including the caller.
//
// The scene list is boot.json's "scenes" array (entry 0 = boot scene): the
// Runtime feeds it from boot.json at startup, the editor from the project's
// packager list at play start. Only listed scenes are loadable — the same
// contract the packager copies by and lints against.
namespace SceneSystem {

// CurrentIndex() when no listed scene is active (e.g. an unsaved editor
// scene, or an argv-override scene that isn't in the boot list).
inline constexpr uint32_t kNoScene = UINT32_MAX;

// Host-side setup. Replaces the list, clears any pending request, and resets
// the current index to kNoScene (call SetCurrent afterwards if known).
void SetSceneList(std::vector<std::string> portablePaths);
const std::vector<std::string>& SceneList();

// Script-facing API. Both QUEUE a request — the latest call in a frame wins —
// and return false (with a warning log) if the scene isn't in the list.
// Names match the filename stem, case-insensitively: LoadScene("Level2")
// matches "Assets/Scenes/Level2.scene".
bool LoadScene(const std::string& name);
bool LoadSceneByIndex(uint32_t index);
void ReloadCurrentScene();          // death/restart convenience; no-op if no current scene

uint32_t    CurrentIndex();         // kNoScene if unknown
std::string CurrentName();          // stem of the current entry; "" if unknown

// Host-side consumption: returns and clears the pending request, if any. The
// host then runs its swap ritual (stop play → load → start play → renderer
// cache invalidation) and reports the result via SetCurrent.
std::optional<uint32_t> ConsumePendingRequest();
void SetCurrent(uint32_t index);

} // namespace SceneSystem
