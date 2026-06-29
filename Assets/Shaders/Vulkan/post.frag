#version 450

// Samples the offscreen scene texture and applies an obvious post effect
// (vignette + warm tint) so M4.1 proves the render-to-texture → sample path, not
// just that something was drawn. set 0, binding 0 is the offscreen color target,
// now in SHADER_READ_ONLY after the command list's TransitionTexture.

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uScene;

void main() {
    vec3 scene = texture(uScene, vUV).rgb;

    // Vignette toward the edges.
    vec2  d   = vUV - 0.5;
    float vig = smoothstep(0.9, 0.25, dot(d, d) * 2.0);

    // Subtle warm grade, blended halfway so the effect reads clearly.
    vec3 graded = mix(scene, scene * vec3(1.15, 0.95, 0.8), 0.5);

    outColor = vec4(graded * vig, 1.0);
}
