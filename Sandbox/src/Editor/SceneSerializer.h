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
class PrefabSerializer {
public:
    // Serializes `root` and all its descendants to `path` (overwrites).
    static bool Save(Scene& scene, entt::entity root, const std::string& path);

    // Instantiates the prefab into the scene with fresh UUIDs per instance
    // (internal parent/target references remapped to the new ids). Stamps the
    // root with PrefabInstanceComponent{path}. Returns the root entity, or
    // entt::null on a missing/unparsable file. The overload places the root
    // at `position` (prefab roots are scene roots, so world == local).
    static entt::entity Instantiate(Scene& scene, const std::string& path);
    static entt::entity Instantiate(Scene& scene, const std::string& path, const glm::vec3& position);
};
