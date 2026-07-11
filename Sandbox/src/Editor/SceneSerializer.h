#pragma once
#include <string>
#include "Scene/Scene.h"

class SceneSerializer {
public:
    static void Save(Scene& scene, const std::string& path);
    static bool Load(Scene& scene, const std::string& path);

    // In-memory variants — used for play-mode snapshot/restore.
    static std::string Stringify(Scene& scene);
    static bool        FromString(Scene& scene, const std::string& data);
};

// A .prefab is the scene format restricted to one entity's subtree:
// { "prefab": { "root": <uuid>, "entities": [ ...same as scene... ] } }
//
// The file's uuids are a stable prefab-local id space: Save keeps each
// entity's original local id (via PrefabChildComponent) across re-saves, so
// instances elsewhere can diff/re-apply against the file without their
// override targets going stale. Scenes store instances as "prefabRef" nodes
// (path + root identity/transform + override diff) and re-instantiate them
// from the file on every load.
class PrefabSerializer {
public:
    // Serializes `root` and all its descendants to `path` (overwrites), and
    // stamps the subtree's PrefabChildComponent local ids so the live entities
    // become the canonical, zero-override instance of what was written.
    static bool Save(Scene& scene, entt::entity root, const std::string& path);

    // Instantiates the prefab into the scene. The instance root keeps a fresh
    // UUID; every child's UUID is derived deterministically from (root uuid,
    // prefab-local uuid) so interior references survive save/load. Stamps the
    // root with PrefabInstanceComponent{path} and every entity with its
    // PrefabChildComponent local id. Returns the root entity, or entt::null on
    // a missing/unparsable file. The overload places the root at `position`
    // (prefab roots are scene roots, so world == local).
    static entt::entity Instantiate(Scene& scene, const std::string& path);
    static entt::entity Instantiate(Scene& scene, const std::string& path, const glm::vec3& position);

    // Live link: pushes `root`'s state to `path`, then rebuilds the file's
    // other (non-legacy) instances in the scene from the new content. Their
    // override diffs are captured against the OLD file first, so values an
    // instance merely inherited update while its genuine per-instance edits
    // survive. Returns how many sibling instances were rebuilt (-1 if the file
    // write failed). Callers must treat rebuilt entity handles as invalidated
    // (clear undo history / prune selection).
    static int SaveAndPropagate(Scene& scene, entt::entity root, const std::string& path);

    // Rebuilds `instanceRoot` from its source file, discarding all overrides
    // but keeping its uuid and root transform. Also serves as "retry" for
    // broken placeholders. Returns the new root (entt::null on failure — the
    // old root is destroyed regardless, replaced by a placeholder on failure).
    static entt::entity Revert(Scene& scene, entt::entity instanceRoot);

    // Detaches the instance from its prefab: strips the root's link and every
    // subtree PrefabChildComponent stamp, so the entities serialize as plain
    // scene data from now on.
    static void Unpack(Scene& scene, entt::entity instanceRoot);
};
