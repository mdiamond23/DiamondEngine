#version 450

// Resolves half-res SSGI into the lit scene at full resolution.
//
// SSGI REPLACES the far-field indirect diffuse where it has data, it does not
// stack on top of it. deferred_lighting.frag MRT-writes the far-field
// irradiance it used into gIndirect, so this pass subtracts an exact value:
//
//     out = scene + M * (ssgiIrradiance - farIrradiance) * confidence
//
// where M is the same diffuse multiplier the lighting pass applied — kD *
// albedo * gIndirect.a, the alpha channel being the occlusion factor that pass
// reports. Both irradiances are in the E/PI convention of
// irradiance_convolution.frag. At confidence 0 the expression collapses to
// the scene color, i.e. pure far-field — the same miss path ssr_composite
// takes. gIndirect carries DDGI probe irradiance under tier 2 and the IBL
// sample otherwise; nothing here branches on which.
//
// Upsampling is bilateral, not bilinear: plain filtering bleeds GI across
// silhouettes, which reads as a halo around every object.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 FragColor;

layout(set = 0, binding = 0) uniform sampler2D sceneColor;     // hdrLit
layout(set = 0, binding = 1) uniform sampler2D ssgiResolved;   // half res
layout(set = 0, binding = 2) uniform sampler2D gViewPos;
layout(set = 0, binding = 3) uniform sampler2D gViewNormal;
layout(set = 0, binding = 4) uniform sampler2D gAlbedo;
layout(set = 0, binding = 5) uniform sampler2D gMaterial;      // r=metallic g=roughness b=ao
layout(set = 0, binding = 6) uniform sampler2D aoTex;          // unused — see gIndirect.a
layout(set = 0, binding = 7) uniform sampler2D gIndirect;      // rgb far-field irradiance, a = its occlusion factor

layout(push_constant) uniform Push {
    float strength;   // 0 = passthrough, 1 = full
} pc;

const float NORMAL_POWER = 8.0;    // bilateral normal falloff
const float DEPTH_SCALE  = 0.05;   // depth tolerance as a fraction of view depth

// Must match deferred_lighting.frag's ambient term exactly.
vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    return F0 + (max(vec3(1.0 - roughness), F0) - F0)
              * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

// 4 half-res taps, weighted by depth + normal agreement with this full-res pixel.
//
// The weights are computed against the EXACT full-res texel each half-res
// sample was traced from (ssgi.comp's anchor, id * scale), not against a
// bilinear read at the half-res texel centre. Those are different values, and
// the difference is precisely the case this filter exists to catch: at a
// silhouette a filtered read returns a depth halfway between the two surfaces,
// so a tap belonging entirely to the far surface scores a middling weight
// instead of ~0 and its GI leaks across the edge. Reading the anchor texel
// makes the test exact.
vec4 BilateralUpsample(vec2 uv, vec3 P, vec3 N) {
    vec2  halfSize = vec2(textureSize(ssgiResolved, 0));
    ivec2 fullSize = textureSize(gViewPos, 0);
    ivec2 scale    = max(ivec2(round(vec2(fullSize) / halfSize)), ivec2(1));

    vec2  coord    = uv * halfSize - 0.5;
    vec2  base     = floor(coord);
    vec2  f        = coord - base;

    float bw[4] = float[4]((1.0 - f.x) * (1.0 - f.y),
                                  f.x  * (1.0 - f.y),
                           (1.0 - f.x) *        f.y,
                                  f.x  *        f.y);
    ivec2 offs[4] = ivec2[4](ivec2(0, 0), ivec2(1, 0), ivec2(0, 1), ivec2(1, 1));

    vec4  sum  = vec4(0.0);
    float wsum = 0.0;

    for (int i = 0; i < 4; ++i) {
        ivec2 t = clamp(ivec2(base) + offs[i], ivec2(0), ivec2(halfSize) - 1);

        // The G-buffer texel that half-res sample actually traced from.
        ivec2 anchor = min(t * scale, fullSize - 1);
        vec3  sp     = texelFetch(gViewPos,    anchor, 0).xyz;
        vec3  sn     = texelFetch(gViewNormal, anchor, 0).xyz;

        // An unwritten normal would make normalize() return NaN, and a single
        // NaN weight poisons the whole sum.
        float snLenSq = dot(sn, sn);

        float wd = exp(-abs(sp.z - P.z) / max(DEPTH_SCALE * abs(P.z), 1e-3));
        float wn = snLenSq > 1e-6
                 ? pow(max(dot(sn * inversesqrt(snLenSq), N), 0.0), NORMAL_POWER)
                 : 0.0;
        float w  = bw[i] * wd * wn + 1e-6;

        sum  += texelFetch(ssgiResolved, t, 0) * w;
        wsum += w;
    }

    return sum / wsum;
}

void main() {
    vec3 scene = texture(sceneColor, vUV).rgb;

    if (pc.strength <= 0.0) { FragColor = vec4(scene, 1.0); return; }

    vec3 P = texture(gViewPos, vUV).xyz;
    if (P.z == 0.0) { FragColor = vec4(scene, 1.0); return; }   // sky

    vec3  normal = texture(gViewNormal, vUV).xyz;
    float nLenSq = dot(normal, normal);
    if (nLenSq <= 1e-6) { FragColor = vec4(scene, 1.0); return; }
    vec3 N = normal * inversesqrt(nLenSq);

    vec4 ssgi = BilateralUpsample(vUV, P, N);
    if (ssgi.a <= 0.0) { FragColor = vec4(scene, 1.0); return; }

    vec3  albedo    = texture(gAlbedo, vUV).rgb;
    vec4  matData   = texture(gMaterial, vUV);
    float metallic  = clamp(matData.r, 0.0, 1.0);
    float roughness = clamp(matData.g, 0.0, 1.0);

    // Camera is at the view-space origin.
    vec3  V     = normalize(-P);
    float NdotV = max(dot(N, V), 0.0);

    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    vec3 kD = (1.0 - fresnelSchlickRoughness(NdotV, F0, roughness)) * (1.0 - metallic);

    // gIndirect carries the far-field irradiance in rgb and, in .a, the exact
    // occlusion factor lighting multiplied it by. Reading that back beats
    // re-deriving ao × ssao here: the lighting pass now fades SSAO by GI tier,
    // and a reconstruction would silently drift out of step with it.
    vec4 far           = texture(gIndirect, vUV);
    vec3 farIrradiance = far.rgb;
    vec3 M             = kD * albedo * far.a;
    vec3 delta = M * (ssgi.rgb - farIrradiance) * ssgi.a * pc.strength;

    FragColor = vec4(max(scene + delta, vec3(0.0)), 1.0);
}
