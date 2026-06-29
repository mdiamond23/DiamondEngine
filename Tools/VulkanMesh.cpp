// M3 Vulkan smoke-test harness. Brings up the Vulkan backend and draws a
// textured, lit, depth-tested cube — proves real MeshData geometry upload, a
// combined image-sampler descriptor, a depth buffer, and a perspective camera
// fed through the per-frame UBO, all through the backend-neutral RHI without
// touching the OpenGL editor in Sandbox/main.cpp.
#include "Platform/VulkanDemo.h"

int main() {
    return Diamond::RunVulkanMeshDemo();
}
