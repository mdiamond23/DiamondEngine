#version 450

// Resolves the raw SSR trace against the lit scene: every pixel outputs
//
//     mix(sceneColor, ssr.rgb, weight)
//
// where weight is confidence × fresnel × roughness fade. This is a full
// resolve into a NEW target (not an in-place blend onto hdrLit): the render
// graph points readers at a texture's last writer, so a pass chain that reads
// hdrLit and later writes it back is a dependency cycle. Pixels with weight 0
// (misses, sky, rough dielectrics) pass the scene color through unchanged —
// keeping their IBL specular from the lighting resolve.
//
// ssrColor.rgb = reflected scene radiance, ssrColor.a = ray-hit confidence.
// gMaterial packing (see gbuffer.frag): r = metallic, g = roughness, b = ao.

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 FragColor;

layout(set = 0, binding = 0) uniform sampler2D ssrColor;
layout(set = 0, binding = 1) uniform sampler2D gViewPos;
layout(set = 0, binding = 2) uniform sampler2D gViewNormal;
layout(set = 0, binding = 3) uniform sampler2D gMaterial;
layout(set = 0, binding = 4) uniform sampler2D sceneColor;

const float DIELECTRIC_F0       = 0.04;
const float METALLIC_F0         = 0.90;
const float ROUGHNESS_FADE_BEGIN = 0.30;
const float ROUGHNESS_FADE_END   = 0.85;

float ReadMetallic(vec4 material) {
    return material.r;
}

float ReadRoughness(vec4 material) {
    return material.g;
}

// Scalar Schlick Fresnel. Since this pass only calculates a blend weight, a
// scalar reflectance is sufficient. Reflection tint can remain part of the
// regular PBR lighting/material calculation.
float FresnelSchlick(float cosTheta, float f0) {
    float oneMinusCos = 1.0 - clamp(cosTheta, 0.0, 1.0);
    float factor = oneMinusCos * oneMinusCos;
    factor *= factor * oneMinusCos;

    return f0 + (1.0 - f0) * factor;
}

void main() {
    vec3 scene = texture(sceneColor, vUV).rgb;
    vec4 ssr   = texture(ssrColor,   vUV);

    // The tracing pass already rejected misses and invalid rays.
    if (ssr.a <= 0.0) {
        FragColor = vec4(scene, 1.0);
        return;
    }

    vec3 fragPos = texture(gViewPos, vUV).xyz;

    // Background pixel: no geometry was written into the G-buffer.
    if (fragPos.z == 0.0) {
        FragColor = vec4(scene, 1.0);
        return;
    }

    vec3 normal = texture(gViewNormal, vUV).xyz;
    float normalLengthSquared = dot(normal, normal);

    if (normalLengthSquared <= 1e-6) {
        FragColor = vec4(scene, 1.0);
        return;
    }

    normal *= inversesqrt(normalLengthSquared);

    vec4 material = texture(gMaterial, vUV);

    float metallic  = clamp(ReadMetallic(material), 0.0, 1.0);
    float roughness = clamp(ReadRoughness(material), 0.0, 1.0);

    // View-space camera is at the origin. fragPos points camera -> surface, so
    // its negation points surface -> camera.
    vec3 viewDir = normalize(-fragPos);

    float nDotV = max(dot(normal, viewDir), 0.0);

    // Without albedo in this pass, use a scalar approximation for metallic F0.
    // A version that samples albedo can calculate the full tinted:
    //
    //     vec3 F0 = mix(vec3(0.04), albedo, metallic);
    //
    float f0 = mix(DIELECTRIC_F0, METALLIC_F0, metallic);
    float fresnel = FresnelSchlick(nDotV, f0);

    // Sharp SSR becomes increasingly inaccurate as roughness rises. Fade to
    // the prefiltered IBL fallback instead of showing a sharp reflection on a
    // rough material.
    float roughnessWeight =
        1.0 - smoothstep(
            ROUGHNESS_FADE_BEGIN,
            ROUGHNESS_FADE_END,
            roughness
        );

    float weight = clamp(ssr.a * fresnel * roughnessWeight, 0.0, 1.0);

    FragColor = vec4(mix(scene, ssr.rgb, weight), 1.0);
}
