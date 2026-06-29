#version 450

// Deferred G-buffer geometry pass (Vulkan port of Deferred/gbuffer.vert — MRT
// slice). Emits view-space position + normal so the later deferred-lighting pass
// can light without re-deriving them. The full GL pass also carries tangents for
// TBN normal-mapping; this slice derives the normal straight from the vertex
// normal (no tangent/normal-map yet — that lands with the PBR material binding).

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUV;

layout(location = 0) out vec3 vViewPos;
layout(location = 1) out vec3 vViewNormal;
layout(location = 2) out vec2 vUV;

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4 view;
    mat4 viewProj;
} ubo;

layout(push_constant) uniform Push {
    mat4 model;
} pc;

void main() {
    vec4 worldPos = pc.model * vec4(inPos, 1.0);
    vViewPos    = vec3(ubo.view * worldPos);
    // Uniform scale assumed (no separate normal matrix) — rotate the normal into
    // world then view space.
    vViewNormal = mat3(ubo.view) * (mat3(pc.model) * inNormal);
    vUV         = inUV;
    gl_Position = ubo.viewProj * worldPos;
}
