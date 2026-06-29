#version 450

// Passthrough copy of an HDR texture onto the current color target. The forward
// passes (PBR surface, transparency) can't blend over the bound attachment they
// also read from, so each one first copies the previous pass's HDR result into
// its own fresh target with this shader, then renders its contribution over it.
// Paired with fullscreen.vert; set 0, binding 0 = the source HDR color.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uSource;

void main() {
    outColor = texture(uSource, vUV);
}
