#version 450

// Post-process tonemap: maps the HDR scene into display range and encodes it for
// the (UNORM, non-sRGB) swapchain. Paired with fullscreen.vert (vertex-less
// triangle). set 0, binding 0 is the HDR color target, in SHADER_READ_ONLY after
// the graph's read barrier.
//
// Two curves, selected per frame so they can be A/B'd on live content:
//   0 = ACES  — the Narkowicz fit the GL renderer uses. Cheap, but it skews hue
//               on saturated highlights (bright reds drift orange, blues purple)
//               and hard-clips at 1.0.
//   1 = AgX   — Troy Sobotka's curve. Saturated highlights desaturate toward
//               white the way film does instead of clipping to a hue shift, so
//               emissive/bloomed content holds together far better.
// The two encode differently: ACES is linear out and needs the manual gamma
// below, while AgX already emits display-encoded sRGB. Applying gamma to AgX
// would wash it out badly, so each branch returns display-ready values.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uHDR;
// 1x1 auto-exposure multiplier. 1.0 when auto-exposure is off, so the manual
// push constant below is always in charge of the final scale either way.
layout(set = 0, binding = 1) uniform sampler2D uExposure;

layout(push_constant) uniform Push {
    float exposure;     // manual exposure / EV compensation, multiplied with uExposure
    int   tonemapper;   // 0 = ACES, 1 = AgX
} pc;

// ── ACES (Narkowicz) ─────────────────────────────────────────────────────────
vec3 ACESFilmic(vec3 x)
{
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

// ── AgX ──────────────────────────────────────────────────────────────────────
// Works directly in sRGB primaries (no Rec.2020 round trip): inset matrix, log2
// encode across the working exposure range, sigmoid, then the inverse matrix.
const mat3 kAgXInset = mat3(
    0.842479062253094,  0.0423282422610123, 0.0423756549057051,
    0.0784335999999992, 0.878468636469772,  0.0784336,
    0.0792237451477643, 0.0791661274605434, 0.879142973793104);

const mat3 kAgXOutset = mat3(
     1.19687900512017,   -0.0528968517574562, -0.0529716355144438,
    -0.0980208811401368,  1.15190312990417,   -0.0980434501171241,
    -0.0990297440797205, -0.0989611768448433,  1.15107367264116);

// 6th-order fit of the AgX sigmoid over the normalized log range.
vec3 AgXContrast(vec3 x)
{
    vec3 x2 = x * x;
    vec3 x4 = x2 * x2;
    return  15.5   * x4 * x2
          - 40.14  * x4 * x
          + 31.96  * x4
          -  6.868 * x2 * x
          +  0.4298 * x2
          +  0.1191 * x
          -  0.00232;
}

vec3 AgX(vec3 color)
{
    const float minEv = -12.47393;
    const float maxEv =   4.026069;

    // SSR/TAA can leave small negatives behind; log2 of those is NaN.
    color = kAgXInset * max(color, vec3(0.0));
    color = clamp(log2(max(color, vec3(1e-10))), minEv, maxEv);
    color = (color - minEv) / (maxEv - minEv);
    color = AgXContrast(color);
    // Output is display-encoded sRGB — the caller must NOT apply gamma again.
    return clamp(kAgXOutset * color, 0.0, 1.0);
}

void main()
{
    float exposure = pc.exposure * texture(uExposure, vec2(0.5)).r;
    vec3  hdr      = texture(uHDR, vUV).rgb * exposure;

    vec3 display;
    if (pc.tonemapper == 1)
        display = AgX(hdr);
    else
        display = pow(ACESFilmic(hdr), vec3(1.0 / 2.2));

    outColor = vec4(display, 1.0);
}
