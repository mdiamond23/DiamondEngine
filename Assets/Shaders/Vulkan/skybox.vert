#version 450

// Skybox background — Vulkan port of Engine/Shaders/IBL/background.vert. The cube's
// object-space position doubles as the world-space sample direction. The view's
// translation is dropped (mat3(view)) so the sky stays centred on the camera, and
// gl_Position.xyww forces depth = w → 1 after the divide, placing every fragment at
// the far plane (LessEqual depth test → fills only pixels no geometry wrote).

layout(location = 0) in vec3 aPos;

layout(location = 0) out vec3 vDir;

layout(set = 0, binding = 0) uniform CameraUBO {
    mat4 view;
    mat4 projection;
} cam;

void main() {
    vDir = aPos;
    mat4 rotView = mat4(mat3(cam.view));   // strip translation
    vec4 clip    = cam.projection * rotView * vec4(aPos, 1.0);
    gl_Position  = clip.xyww;              // depth = w → 1.0 (far plane)
}
