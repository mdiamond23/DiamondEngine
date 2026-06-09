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
