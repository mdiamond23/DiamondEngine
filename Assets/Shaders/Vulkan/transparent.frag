#version 450

// Forward transparency — Vulkan port of Engine/Shaders/Forward/transparent.frag.
// Samples the albedo texture and emits it straight (with alpha) for hardware
// src-over blending against the HDR target. The GL shader also wrote a zero
// BrightColor MRT for bloom; this single-target HDR pass drops it (bloom, which runs
// later, self-extracts).

layout(location = 0) in  vec2 vTexCoords;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 1) uniform sampler2D albedo;

void main() {
    vec4 color = texture(albedo, vTexCoords);
    if (color.a < 0.01)
        discard;
    outColor = color;
}
