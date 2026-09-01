#version 460
#extension GL_EXT_ray_tracing          : require
#extension GL_EXT_ray_query            : require
#extension GL_EXT_buffer_reference2    : require
#extension GL_EXT_nonuniform_qualifier : require
#extension GL_GOOGLE_include_directive : require

#include "ddgi_common.glsl"

// Reflection hit shading. Returns the outgoing radiance of the hit surface:
//
//   L = kD * albedo/pi * (E_direct + E_probe)  +  F * env(reflect)  +  emissive
//
// where kD = (1 - F) * (1 - metallic) and F is Schlick around
// F0 = mix(0.04, albedo, metallic).
//
// THE METALLIC SPLIT IS NOT COSMETIC. This shader used to run the diffuse term
// alone, on every hit, metal included — and a metal has no diffuse lobe at all.
// A chrome surface's base color is dark, so albedo * E came back very near
// black, and a mirror sphere reflected in a mirror floor turned into a dark
// blob wearing only its emissive dots. That is fine right up until the pixel
// next to it is one SSR kept, where the same sphere is the real, bright,
// specular thing sampled out of the lit scene. The two shading models meet at
// the SSR confidence boundary and the difference reads as a hard CUT through
// the reflection. Splitting the lobes puts a metal's energy where it belongs
// and the seam becomes a change in sharpness rather than a change in colour.
//
// Ported from ddgi_probe_trace.rchit, which this is ~90% identical to. Three
// differences, all deliberate:
//
//  1. The second term is a DDGI lookup at THIS hit point, not the probe's own
//     previous-frame irradiance. Same function, different sample position — it
//     is what keeps a reflection of a shadowed wall from going black, since
//     direct light alone has nothing to give there.
//  2. A backface hit FLIPS the normal instead of reporting a negative
//     distance. A probe treats backfaces as evidence it is buried; a reflection
//     just needs the surface it can see shaded sensibly.
//  3. A SPECULAR lobe, which the probe trace has no use for. A probe gathers
//     irradiance, so diffuse-only is right there; a reflection is looked at
//     directly, so a metal shading as Lambertian is visible immediately. The
//     lobe is one bounce off the environment — see the specular term below.
//
// Geometry is reached by DEVICE ADDRESS, not descriptors: gl_InstanceCustomIndexEXT
// indexes the same {vertexBuffer, indexBuffer, albedo, emissive, slots} table
// SceneRenderer builds for DDGI, and the base-color and metallic maps come from
// the shared bindless array. A ray hits whatever it hits, so no per-draw set could serve
// it, and the index is non-uniform across a subgroup by nature.

layout(buffer_reference, std430, buffer_reference_align = 4) readonly buffer Verts {
    float f[];
};
layout(buffer_reference, std430, buffer_reference_align = 4) readonly buffer Indices {
    uint i[];
};

// One record per TLAS instance. Mirrored by SceneRenderer's RTGeometry — the
// same table the probe trace reads.
struct RTGeometry {
    Verts   vb;
    Indices ib;
    vec4    albedo;     // rgb = base-color factor, a = UV scale
    vec4    emissive;   // rgb = emissive radiance, a = metallic factor
    uvec4   indices;    // x = albedo array slot, y = vertex stride in FLOATS,
                        // z = metallic array slot
};

layout(set = 0, binding = 1) uniform accelerationStructureEXT uTLAS;   // shadow queries

layout(set = 0, binding = 6, std140) uniform ReflectBlock {
    mat4  view;         // world -> view (lights are stored in view space)
    mat4  invView;
    vec4  ddgiOrigin;
    vec4  ddgiSpacing;
    ivec4 ddgiCounts;
    vec4  ddgiParams;   // x normalBias, y energy, z viewBias, w giMode
    vec4  params;       // x maxRayDistance, y skyIntensity
} r;

// Verbatim the layout of deferred_lighting.frag's LightingUBO — the whole block
// is declared even though only the diffuse terms are read, because std140
// offsets depend on every preceding member.
layout(set = 0, binding = 7, std140) uniform LightingUBO {
    mat4 lightFromView[4];
    mat4 invView;
    vec4 cascadeSplits;
    vec4 sunDirView;
    vec4 sunColor;
    vec4 pointPos[4];
    vec4 pointColor[4];
    vec4 pointPosWorld[4];
    mat4 spotFromView[4];
    vec4 spotPosView[4];
    vec4 spotDirView[4];
    vec4 spotColor[4];
    vec4 counts;
    vec4 ambient;
} u;

layout(set = 0, binding = 8, std430) readonly buffer GeometryBlock {
    RTGeometry g[];
} geom;

layout(set = 0, binding = 9)  uniform sampler2D uIrradianceAtlas;
layout(set = 0, binding = 10) uniform sampler2D uVisibilityAtlas;
layout(set = 0, binding = 11) uniform sampler2D uProbeData;

// The same baked radiance cubemap the miss shader returns. A reflected METAL
// has no diffuse term to speak of, so this is where its outgoing radiance comes
// from — and sampling the identical texture the miss path samples is what keeps
// a mirror ball's reflection of the sky agreeing with the sky beside it.
layout(set = 0, binding = 12) uniform samplerCube uEnvMap;

// Bindless material maps, indexed by the record's slots. Slot 0 is 1x1 white, so
// an untextured material resolves to its factor alone — matching the G-buffer
// pass. Base color and metallic both come from this one array.
layout(set = 0, binding = 13) uniform sampler2D uAlbedoMaps[];

layout(location = 0) rayPayloadInEXT vec3 payload;
hitAttributeEXT vec2 attribs;

// ── Shadowing (inline ray query against the scene TLAS) ──────────────────────
// Binary, one ray, no PCF. TerminateOnFirstHit + an opaque-only traversal make
// this a pure visibility test: it stops at the first blocker and never invokes a
// hit shader. Ray QUERIES traverse inline, so maxRecursionDepth stays at 1 even
// though this runs from inside a hit shader.
//
// Skinned meshes are absent from the TLAS by design, so a character casts no
// shadow inside a reflection. Same known exclusion as everywhere else in RT.
bool Occluded(vec3 origin, vec3 dir, float maxDist) {
    const float kBias = 0.02;   // tMin — start off the surface so it can't shadow itself
    if (maxDist <= kBias) return false;

    rayQueryEXT rq;
    rayQueryInitializeEXT(rq, uTLAS,
                          gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
                          0xFF, origin, kBias, dir, maxDist);
    rayQueryProceedEXT(rq);
    return rayQueryGetIntersectionTypeEXT(rq, true)
           != gl_RayQueryCommittedIntersectionNoneEXT;
}

float Falloff(float dist, float radius) {
    float w = clamp(1.0 - pow(dist / radius, 4.0), 0.0, 1.0);
    return (w * w) / max(dist * dist, 1e-4);
}

// Total incident irradiance E at the hit, diffuse only. Lights live in VIEW
// space (the UBO is shared with the deferred pass) while shadow rays need world
// space, so each direction is rotated out by mat3(invView) before tracing.
vec3 DirectIrradiance(vec3 viewPos, vec3 N, vec3 worldPos) {
    const mat3 viewToWorld = mat3(u.invView);
    vec3 E = vec3(0.0);

    {   // Sun. Directional, so the shadow ray runs to effectively infinity.
        vec3  L     = normalize(-u.sunDirView.xyz);
        float NdotL = max(dot(N, L), 0.0);
        if (NdotL > 0.0 && !Occluded(worldPos, normalize(viewToWorld * L), 1e4))
            E += u.sunColor.xyz * NdotL;
    }

    // CLAMPED to the array bounds. counts comes from a dynamic UBO, and a
    // garbage value is harmless arithmetic in a raster pass but a silent
    // VK_ERROR_DEVICE_LOST once each iteration costs a BVH traversal. See
    // gi-design.md's open trap — the root cause is still unfound.
    int np = clamp(int(u.counts.x), 0, 4);
    for (int i = 0; i < np; ++i) {
        vec3  d    = u.pointPos[i].xyz - viewPos;
        float dist = length(d);
        if (dist >= u.pointPos[i].w) continue;
        vec3  L     = d / dist;
        float NdotL = max(dot(N, L), 0.0);
        if (NdotL <= 0.0) continue;
        // Ray stops AT the light, so geometry behind it doesn't shadow.
        if (Occluded(worldPos, normalize(viewToWorld * L), dist)) continue;
        E += u.pointColor[i].xyz * Falloff(dist, u.pointPos[i].w) * NdotL;
    }

    int ns = clamp(int(u.counts.z), 0, 4);
    for (int i = 0; i < ns; ++i) {
        vec3  d    = u.spotPosView[i].xyz - viewPos;
        float dist = length(d);
        if (dist >= u.spotColor[i].w) continue;
        vec3  L        = d / dist;
        float cosTheta = dot(-L, u.spotDirView[i].xyz);
        float cosInner = u.spotDirView[i].w;
        float cosOuter = u.spotPosView[i].w;
        float ang = clamp((cosTheta - cosOuter) / max(cosInner - cosOuter, 1e-4), 0.0, 1.0);
        if (ang <= 0.0) continue;
        float NdotL = max(dot(N, L), 0.0);
        if (NdotL <= 0.0) continue;
        if (Occluded(worldPos, normalize(viewToWorld * L), dist)) continue;
        E += u.spotColor[i].xyz * Falloff(dist, u.spotColor[i].w) * ang * NdotL;
    }

    return E;
}

void main() {
    const float t = gl_HitTEXT;

    RTGeometry g = geom.g[gl_InstanceCustomIndexEXT];
    const uint stride = g.indices.y;

    const uint tri = uint(gl_PrimitiveID) * 3u;
    const uint i0 = g.ib.i[tri + 0u];
    const uint i1 = g.ib.i[tri + 1u];
    const uint i2 = g.ib.i[tri + 2u];

    // MeshVertex is pos(3) normal(3) uv(2) tangent(3) — see SceneRenderer.
    const uint nOff = 3u;
    const uint tOff = 6u;
    vec3 n0 = vec3(g.vb.f[i0 * stride + nOff], g.vb.f[i0 * stride + nOff + 1u],
                   g.vb.f[i0 * stride + nOff + 2u]);
    vec3 n1 = vec3(g.vb.f[i1 * stride + nOff], g.vb.f[i1 * stride + nOff + 1u],
                   g.vb.f[i1 * stride + nOff + 2u]);
    vec3 n2 = vec3(g.vb.f[i2 * stride + nOff], g.vb.f[i2 * stride + nOff + 1u],
                   g.vb.f[i2 * stride + nOff + 2u]);

    vec2 t0 = vec2(g.vb.f[i0 * stride + tOff], g.vb.f[i0 * stride + tOff + 1u]);
    vec2 t1 = vec2(g.vb.f[i1 * stride + tOff], g.vb.f[i1 * stride + tOff + 1u]);
    vec2 t2 = vec2(g.vb.f[i2 * stride + tOff], g.vb.f[i2 * stride + tOff + 1u]);

    const vec3 bary = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);
    vec3 objectN = normalize(n0 * bary.x + n1 * bary.y + n2 * bary.z);
    vec2 uv      = (t0 * bary.x + t1 * bary.y + t2 * bary.z) * g.albedo.a;
    // Normals transform by the inverse transpose; instance transforms here are
    // rigid + uniform scale (scene props), so the plain 3x3 is exact enough.
    vec3 worldN  = normalize(mat3(gl_ObjectToWorldEXT) * objectN);

    // Backface: FLIP toward the ray rather than reject. Winding is unreliable in
    // imported .obj/glTF content, so the test uses the authored vertex normal
    // (same reasoning as the probe trace) — but where a probe treats this as
    // evidence it is buried, a reflection just needs the visible side lit. The
    // alternative, returning black, puts holes in reflections of thin geometry.
    if (dot(gl_WorldRayDirectionEXT, worldN) > 0.0) worldN = -worldN;

    const vec3 worldPos = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * t;
    const vec3 viewPos  = (r.view * vec4(worldPos, 1.0)).xyz;
    const vec3 viewN    = normalize(mat3(r.view) * worldN);

    // Ray tracing has no derivatives, so the maps are sampled at LOD 0.
    const vec3 albedo = g.albedo.rgb
        * textureLod(uAlbedoMaps[nonuniformEXT(g.indices.x)], uv, 0.0).rgb;

    // Metallic, reconstructed exactly as gbuffer.frag reconstructs it: the CPU
    // sends 1.0 for a material that owns a metallic MAP and the factor for one
    // that does not, and slot 0 is 1x1 white — so this one multiply covers both
    // without a branch. Red channel, matching the G-buffer's convention.
    const float metallic = clamp(
        g.emissive.a * textureLod(uAlbedoMaps[nonuniformEXT(g.indices.z)], uv, 0.0).r,
        0.0, 1.0);

    // Schlick, against the direction the ray CAME from. A dielectric keeps its
    // 0.04 specular and nearly all its diffuse; a metal puts everything into F
    // and tints it with its own base color, which is what makes a reflected
    // gold surface gold instead of a dark brown Lambertian patch.
    const vec3  F0    = mix(vec3(0.04), albedo, metallic);
    const vec3  Vdir  = -gl_WorldRayDirectionEXT;
    const float NoV   = max(dot(worldN, Vdir), 0.0);
    const float fSch  = pow(1.0 - NoV, 5.0);
    const vec3  F     = F0 + (1.0 - F0) * fSch;

    // Energy left for the diffuse lobe. Metals have none.
    const vec3 kD = (1.0 - F) * (1.0 - metallic);

    // Direct, Lambert (probes and this both store/return irradiance, so the
    // albedo/pi * E form is the right one).
    vec3 L = kD * albedo * (DirectIrradiance(viewPos, viewN, worldPos) / DDGI_PI);

    // Indirect at the hit, from the probe field. WITHOUT THIS a reflection of
    // anything the sun cannot see is black — direct light has nothing to give
    // in shadow, and there is no ambient term out here. giMode 2 means the
    // atlases hold real probe data; otherwise the scene has no volume and
    // reflections lose their indirect term entirely (documented limitation).
    if (r.ddgiParams.w > 1.5) {
        DDGIVolume vol;
        vol.origin  = r.ddgiOrigin;
        vol.spacing = r.ddgiSpacing;
        vol.counts  = r.ddgiCounts;
        vol.params  = vec4(0.0, r.ddgiParams.x, 1.0, 0.0);
        vol.params2 = vec4(0.0, r.ddgiParams.z, 0.0, 0.0);
        // The atlas stores E/pi, so albedo/pi * E collapses to a plain multiply.
        // Energy is applied HERE, outside the lookup — see ddgi_common.glsl.
        L += kD * albedo * DDGISampleIrradiance(vol, uIrradianceAtlas, uVisibilityAtlas,
                                                uProbeData, worldPos, worldN,
                                                -gl_WorldRayDirectionEXT)
                         * r.ddgiParams.y;
    }

    // Specular: ONE bounce, off the environment. maxRecursionDepth is 1 and ray
    // queries can't launch a hit shader, so there is nothing here that could
    // shade a second surface — a reflected mirror shows the env and its own
    // emissive, not the room behind the camera. That is a real limitation, but
    // it is bounded and plausible, where the old diffuse-only version was
    // simply the wrong lobe.
    //
    // No roughness fade: the env cubemap has no prefiltered chain to fade INTO,
    // and the reflection pass only runs under its own roughness cutoff anyway.
    L += F * textureLod(uEnvMap, reflect(gl_WorldRayDirectionEXT, worldN), 0.0).rgb
           * r.params.y;

    L += g.emissive.rgb;

    payload = L;
}
