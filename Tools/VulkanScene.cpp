// Wiring Slice 1 harness. Builds a real Scene/ECS and drives it through
// SceneRenderer's deferred chain into a Vulkan window — the first end-to-end test
// of the Scene → RHI bridge, isolated from the OpenGL editor in Sandbox/main.cpp.
#include "Platform/VulkanDemo.h"

int main() {
    return Diamond::RunVulkanSceneDemo();
}
