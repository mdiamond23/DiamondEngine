#pragma once
#include <string>
#include "Scene/Scene.h"

class SceneSerializer {
public:
    static void Save(Scene& scene, const std::string& path);

    // Clears the scene and reconstructs it from a .scene file.
    // Returns false if the file cannot be opened or parsed.
    static bool Load(Scene& scene, const std::string& path);
};
