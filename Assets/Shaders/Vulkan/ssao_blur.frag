#version 450

// RETAINED, UNUSED — superseded by gtao_denoise.comp (gi-design.md slice 4.5).
// Still compiled so it cannot rot; nothing loads the .spv.
//
// SSAO blur pass (Vulkan port of Deferred/ssao_blur.frag). A 4×4 box blur that
// smooths the noisy raw occlusion — the noise rotation makes the raw result grainy,
// and a blur the size of the noise tile averages it out. Single-channel in/out.

layout(location = 0) in vec2 vUV;
layout(location = 0) out float FragColor;

layout(set = 0, binding = 0) uniform sampler2D ssaoInput;

void main() {
    vec2 texelSize = 1.0 / vec2(textureSize(ssaoInput, 0));
    float result = 0.0;
    for (int x = -2; x < 2; ++x)
        for (int y = -2; y < 2; ++y)
            result += texture(ssaoInput, vUV + vec2(x, y) * texelSize).r;
    FragColor = result / 16.0;
}
