#version 450

// FXAA (Fast Approximate Anti-Aliasing): edge-directed two-tap blend over an LDR
// color image. Vulkan port of Engine/Shaders/PostProcess/fxaa.frag. Paired with
// fullscreen.vert (vertex-less triangle). set 0, binding 0 is the tonemapped LDR
// image, in SHADER_READ_ONLY after the graph's read barrier.
//
// Differences from the GL source: texelSize is derived from textureSize() instead
// of a uniform, and the runtime 'enabled' branch is dropped — whether FXAA runs is
// decided by whether the render graph includes this pass.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uScreen;

float luma(vec3 rgb) {
    return dot(rgb, vec3(0.299, 0.587, 0.114));
}

void main() {
    vec2 texelSize = 1.0 / vec2(textureSize(uScreen, 0));

    vec3 center = texture(uScreen, vUV).rgb;

    // 4-corner + center luma samples
    vec3 nw = texture(uScreen, vUV + vec2(-1.0, -1.0) * texelSize).rgb;
    vec3 ne = texture(uScreen, vUV + vec2( 1.0, -1.0) * texelSize).rgb;
    vec3 sw = texture(uScreen, vUV + vec2(-1.0,  1.0) * texelSize).rgb;
    vec3 se = texture(uScreen, vUV + vec2( 1.0,  1.0) * texelSize).rgb;

    float lumaM  = luma(center);
    float lumaNW = luma(nw);
    float lumaNE = luma(ne);
    float lumaSW = luma(sw);
    float lumaSE = luma(se);

    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));
    float lumaRange = lumaMax - lumaMin;

    // Skip pixels that are clearly not on an edge
    if (lumaRange < max(0.0312, lumaMax * 0.125)) {
        outColor = vec4(center, 1.0);
        return;
    }

    // Compute blend direction from diagonal luma gradients
    vec2 dir;
    dir.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
    dir.y =  ((lumaNW + lumaSW) - (lumaNE + lumaSE));

    float dirReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * 0.03125, 0.0078125);
    float rcpDirMin = 1.0 / (min(abs(dir.x), abs(dir.y)) + dirReduce);
    dir = clamp(dir * rcpDirMin, vec2(-8.0), vec2(8.0)) * texelSize;

    // Two-tap blend — inner pair + outer pair, keep whichever stays in the local range
    vec3 rgbA = 0.5 * (
        texture(uScreen, vUV + dir * (1.0/3.0 - 0.5)).rgb +
        texture(uScreen, vUV + dir * (2.0/3.0 - 0.5)).rgb);
    vec3 rgbB = rgbA * 0.5 + 0.25 * (
        texture(uScreen, vUV + dir * -0.5).rgb +
        texture(uScreen, vUV + dir *  0.5).rgb);

    float lumaB = luma(rgbB);
    outColor = vec4(
        (lumaB < lumaMin || lumaB > lumaMax) ? rgbA : rgbB,
        1.0);
}
