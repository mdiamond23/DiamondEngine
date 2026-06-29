#version 450

// Separable Gaussian blur, one axis per invocation. Vulkan port of
// Engine/Shaders/Bloom/blur.frag. The render graph drives the ping-pong by
// alternating source/destination textures across passes; this shader only needs
// to know which axis to blur, supplied as a push constant (1 = horizontal).
// Paired with fullscreen.vert. set 0, binding 0 is the image to blur.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uImage;

layout(push_constant) uniform Push {
    int horizontal;
} pc;

const float weight[5] = float[](0.2270270270, 0.1945945946, 0.1216216216,
                                0.0540540541, 0.0162162162);

void main() {
    vec2 texOffset = 1.0 / vec2(textureSize(uImage, 0));
    vec3 result = texture(uImage, vUV).rgb * weight[0];

    if (pc.horizontal != 0) {
        for (int i = 1; i < 5; ++i) {
            result += texture(uImage, vUV + vec2(texOffset.x * i, 0.0)).rgb * weight[i];
            result += texture(uImage, vUV - vec2(texOffset.x * i, 0.0)).rgb * weight[i];
        }
    } else {
        for (int i = 1; i < 5; ++i) {
            result += texture(uImage, vUV + vec2(0.0, texOffset.y * i)).rgb * weight[i];
            result += texture(uImage, vUV - vec2(0.0, texOffset.y * i)).rgb * weight[i];
        }
    }

    outColor = vec4(result, 1.0);
}
