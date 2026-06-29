#version 450

// Demo-only deferred visualizer. Tiles G-buffer + SSAO across a 3×2 grid on the
// swapchain so the ported passes can be eyeballed: depth, normals, albedo, raw vs
// blurred SSAO — and the sixth cell renders the scene with a REAL cascaded shadow
// (reconstructs each pixel from gViewPos, selects the cascade by view depth, samples
// that cascade's shadow map, shades lit/shadowed). That cell is the actual CSM
// lookup the deferred-lighting pass will use, so a correct shadow here proves the
// cascade depth maps. Not a ported engine pass; it exists to verify the chain.

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D gViewPos;
layout(set = 0, binding = 1) uniform sampler2D gViewNormal;
layout(set = 0, binding = 2) uniform sampler2D gAlbedo;
layout(set = 0, binding = 3) uniform sampler2D ssaoRaw;
layout(set = 0, binding = 4) uniform sampler2D ssaoBlurred;
layout(set = 0, binding = 5) uniform sampler2D shadowCascade0;
layout(set = 0, binding = 6) uniform sampler2D shadowCascade1;
layout(set = 0, binding = 7) uniform sampler2D shadowCascade2;
layout(set = 0, binding = 8) uniform sampler2D shadowCascade3;

layout(set = 0, binding = 9) uniform ShadowUBO {
    mat4 lightFromView[4];   // per-cascade lightMatrix * inverse(view): view space → light clip
    vec4 splits;             // cascade far view-depths (positive), x..w = cascade 0..3
} sh;

const float kBias = 0.0025;

// Cascaded shadow factor for a view-space position. 1.0 = lit, ~0.3 = shadowed.
float ShadowFactor(vec3 viewPos) {
    float viewDepth = -viewPos.z;   // view space looks down -z → positive distance

    int c = 3;
    if      (viewDepth < sh.splits.x) c = 0;
    else if (viewDepth < sh.splits.y) c = 1;
    else if (viewDepth < sh.splits.z) c = 2;

    vec4 lc = sh.lightFromView[c] * vec4(viewPos, 1.0);
    vec3 p  = lc.xyz / lc.w;             // ortho → w==1
    vec2 suv = p.xy * 0.5 + 0.5;         // light NDC → [0,1] (matches the flipped viewport)
    float current = p.z;                 // [0,1] depth (GLM_FORCE_DEPTH_ZERO_TO_ONE)

    // Outside this cascade's map → treat as lit.
    if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) return 1.0;

    float closest;
    if      (c == 0) closest = texture(shadowCascade0, suv).r;
    else if (c == 1) closest = texture(shadowCascade1, suv).r;
    else if (c == 2) closest = texture(shadowCascade2, suv).r;
    else             closest = texture(shadowCascade3, suv).r;

    return (current - kBias > closest) ? 0.3 : 1.0;
}

void main() {
    const vec2 grid = vec2(3.0, 2.0);
    ivec2 cell = ivec2(floor(vUV * grid));
    int   idx  = cell.y * 3 + cell.x;
    vec2  uv   = fract(vUV * grid);

    vec3 col;
    if (idx == 0) {
        col = vec3(-texture(gViewPos, uv).z) * 0.15;            // view-space depth
    } else if (idx == 1) {
        col = texture(gViewNormal, uv).xyz * 0.5 + 0.5;         // normals
    } else if (idx == 2) {
        col = pow(texture(gAlbedo, uv).rgb, vec3(1.0 / 2.2));   // albedo (linear → sRGB)
    } else if (idx == 3) {
        col = vec3(texture(ssaoRaw, uv).r);                     // raw occlusion (noisy)
    } else if (idx == 4) {
        col = vec3(texture(ssaoBlurred, uv).r);                 // blurred occlusion
    } else {
        // Scene shaded by the cascaded shadow — albedo modulated by lit/shadowed.
        vec3 vp = texture(gViewPos, uv).xyz;
        if (vp == vec3(0.0)) {
            col = vec3(0.08);                                   // background (no geometry)
        } else {
            vec3 albedo = pow(texture(gAlbedo, uv).rgb, vec3(1.0 / 2.2));
            col = albedo * ShadowFactor(vp);
        }
    }
    outColor = vec4(col, 1.0);
}
