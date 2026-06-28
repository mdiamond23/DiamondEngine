// M1 Vulkan smoke-test harness. Brings up the Vulkan backend and draws one
// spinning indexed triangle — proves the graphics pipeline, vertex/index
// buffers, per-frame UBO descriptor set, and push constants all work end to end
// without touching the OpenGL editor in Sandbox/main.cpp.
#include "Platform/VulkanDemo.h"

int main() {
    return Diamond::RunVulkanTriangleDemo();
}
