#version 450

// Deferred G-buffer fragment stage — the MRT slice. Writes five color attachments
// in one pass (the capability this milestone proves out): the dynamic-rendering
// scope binds all five and each `location` maps to one. Layout mirrors the GL
// Deferred/gbuffer.frag:
//   0 gViewPos   RGBA16F  view-space position
//   1 gViewNormal RGBA16F view-space normal
//   2 gAlbedo    RGBA8    base color (linear)
//   3 gMaterial  RGBA8    r=metallic g=roughness b=ao
//   4 gEmissive  RGBA16F  emissive (HDR)
// The slice fills albedo from one texture and writes sensible constants for the
// metallic/roughness/ao/emissive channels; the full PBR material binding (and
// tangent-space normal mapping) arrives alongside the deferred-lighting pass.

layout(location = 0) in vec3 vViewPos;
layout(location = 1) in vec3 vViewNormal;
layout(location = 2) in vec2 vUV;

layout(location = 0) out vec4 gViewPos;
layout(location = 1) out vec4 gViewNormal;
layout(location = 2) out vec4 gAlbedo;
layout(location = 3) out vec4 gMaterial;
layout(location = 4) out vec4 gEmissive;

layout(set = 0, binding = 1) uniform sampler2D albedoMap;

void main() {
    vec3 albedo = pow(texture(albedoMap, vUV).rgb, vec3(2.2));   // sRGB → linear (matches GL)

    gViewPos    = vec4(vViewPos, 1.0);
    gViewNormal = vec4(normalize(vViewNormal), 0.0);
    gAlbedo     = vec4(albedo, 1.0);
    gMaterial   = vec4(0.0, 0.5, 1.0, 1.0);   // metallic=0, roughness=0.5, ao=1 (slice defaults)
    gEmissive   = vec4(0.0, 0.0, 0.0, 1.0);
}
