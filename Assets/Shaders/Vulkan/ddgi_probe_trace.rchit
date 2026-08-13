#version 460
#extension GL_EXT_ray_tracing                    : require
#extension GL_EXT_ray_query                      : require
#extension GL_EXT_buffer_reference2              : require
#extension GL_EXT_nonuniform_qualifier           : require
#extension GL_GOOGLE_include_directive           : require

#include "ddgi_common.glsl"

// Probe ray shading. Returns the outgoing radiance of the hit surface:
//
//   L = albedo/pi * E_direct  +  albedo * E_prev/pi  +  emissive
//
// The second term is the previous frame's probe irradiance — the recursive term
// that buys multi-bounce GI at ray depth 1. maxRecursionDepth stays at 1: the
// shadow rays below are ray QUERIES, which traverse inline and cannot recurse.
//
// Geometry is reached by DEVICE ADDRESS (GL_EXT_buffer_reference2), not through
// descriptors: each TLAS instance's gl_InstanceCustomIndexEXT indexes a table of
// {vertexBuffer, indexBuffer, albedo, emissive, textureSlot} that SceneRenderer
// fills from the same draw-list walk that builds the TLAS. The base-color MAP is
// sampled from a bindless array indexed by that slot — a ray hits whatever it
// hits, so no per-draw descriptor set can serve it. The index is non-uniform
// across a subgroup by nature, hence nonuniformEXT.
//
// Direct lighting is DIFFUSE ONLY (probes store irradiance, so the specular lobe
// would be wrong to include). Every light is shadowed by an inline RAY QUERY
// against the same TLAS. The math is deliberately a trimmed copy of
// deferred_lighting.frag rather than a shared include: that shader is a working
// Cook-Torrance resolve and this needs Lambert.
//
// Shadow rays replaced a CSM lookup here. CSM is fitted to the CAMERA frustum,
// but probes sit wherever the volume is, so any probe outside the current
// cascades read "unshadowed" and baked full sun into the atlas — including
// probes indoors whose sun is blocked by a roof. The result was a flat, uniform
// fill that moved as the camera turned. A ray query has no such coverage limit,
// costs no second miss shader / SBT record / recursion depth, and also closes
// the slice-3 gap where point and spot lights leaked through walls into probes.

layout(buffer_reference, std430, buffer_reference_align = 4) readonly buffer Verts {
    float f[];
};
layout(buffer_reference, std430, buffer_reference_align = 4) readonly buffer Indices {
    uint i[];
};

// One record per TLAS instance. Mirrored by SceneRenderer's RTGeometry.
struct DDGIGeometry {
    Verts   vb;
    Indices ib;
    vec4    albedo;     // rgb = base-color factor, a = UV scale
    vec4    emissive;   // rgb = emissive radiance
    uvec4   indices;    // x = albedo array slot, y = vertex stride in FLOATS
};

layout(set = 0, binding = 0) uniform accelerationStructureEXT uTLAS;   // shadow queries
layout(set = 0, binding = 2, std140) uniform DDGIBlock { DDGIVolume v; } ddgi;
layout(set = 0, binding = 3, std430) readonly buffer GeometryBlock {
    DDGIGeometry g[];
} geom;
layout(set = 0, binding = 4) uniform sampler2D uIrradianceAtlas;
layout(set = 0, binding = 5) uniform sampler2D uVisibilityAtlas;
layout(set = 0, binding = 6) uniform sampler2D uProbeData;

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

// Bindings 8-11 are the CSM cascades. No longer declared — shadowing is traced
// now. The descriptor set still carries them (harmless; unused descriptors are
// legal) so the C++ side did not have to change.

// Bindless base-color maps. Slot 0 is 1x1 white, so an untextured material
// resolves to its base-color factor alone — matching the G-buffer pass.
layout(set = 0, binding = 13) uniform sampler2D uAlbedoMaps[];

layout(location = 0) rayPayloadInEXT DDGIPayload payload;
hitAttributeEXT vec2 attribs;

// ── Shadowing (inline ray query against the scene TLAS) ──────────────────────
// Binary: one ray, no PCF. A probe integrates hundreds of rays, which is its own
// filter. TerminateOnFirstHit + SkipClosestHitShader make this an any-hit
// visibility test — the traversal stops at the first blocker and never invokes a
// hit shader, so it is far cheaper than a shading ray.
//
// Skinned meshes are absent from the TLAS by design, so a character casts no
// probe shadow. Same known exclusion as everywhere else in DDGI.
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

// Total incident irradiance E at a surface, diffuse only. Lights live in VIEW
// space (the UBO is shared with the deferred pass) while shadow rays need world
// space, so each direction is rotated out by mat3(invView) before tracing.
vec3 DirectIrradiance(vec3 viewPos, vec3 N, vec3 worldPos) {
    const mat3 viewToWorld = mat3(u.invView);
    vec3 E = vec3(0.0);

    {   // Sun. Directional, so the shadow ray runs to effectively infinity —
        // anything static between here and the sky blocks it.
        vec3  L     = normalize(-u.sunDirView.xyz);
        float NdotL = max(dot(N, L), 0.0);
        if (NdotL > 0.0 && !Occluded(worldPos, normalize(viewToWorld * L), 1e4))
            E += u.sunColor.xyz * NdotL;
    }

    // CLAMPED to the array bounds. counts comes from a dynamic UBO, and a
    // garbage value here used to just waste light math � now each iteration
    // costs a BVH traversal, so an out-of-range count is a GPU hang.
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

    DDGIGeometry g = geom.g[gl_InstanceCustomIndexEXT];
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

    // Backface test against the AUTHORED vertex normal, not gl_HitKindEXT.
    // Winding is unreliable in imported content — .obj and glTF meshes routinely
    // arrive with inconsistently wound triangles — so a winding-based test
    // reports backfaces on surfaces the probe is plainly outside of, and which
    // triangles it lies about changes with the per-frame ray rotation. That fed
    // noise straight into relocation's buried-probe classification. Vertex
    // normals are authored outward and don't have that problem.
    //
    // A backface hit means the ray started inside geometry: report the distance
    // NEGATED and no radiance. The irradiance blend drops these rays; relocation
    // counts them to decide the probe is buried.
    if (dot(gl_WorldRayDirectionEXT, worldN) > 0.0) {
        payload.radiance = vec3(0.0);
        payload.hitT     = -t;
        return;
    }

    const vec3 worldPos = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * t;
    const vec3 viewPos  = (ddgi.v.view * vec4(worldPos, 1.0)).xyz;
    const vec3 viewN    = normalize(mat3(ddgi.v.view) * worldN);

    // Ray tracing has no derivatives, so the base-color map is sampled at LOD 0.
    // Probes integrate hundreds of rays, which is its own filter — the aliasing
    // a mip would prevent averages out.
    const vec3 albedo = g.albedo.rgb
        * textureLod(uAlbedoMaps[nonuniformEXT(g.indices.x)], uv, 0.0).rgb;

    // Direct, plus last frame's probe irradiance for the recursive bounce. The
    // atlas already stores E/pi, so the Lambertian albedo/pi * E collapses to a
    // plain multiply on that term. Skipped entirely until the atlas holds
    // anything real.
    vec3 L = albedo * (DirectIrradiance(viewPos, viewN, worldPos) / DDGI_PI);
    if (ddgi.v.params3.x > 0.5) {
        vec3 prev = DDGISampleIrradiance(ddgi.v, uIrradianceAtlas, uVisibilityAtlas,
                                         uProbeData, worldPos, worldN,
                                         -gl_WorldRayDirectionEXT);
        L += albedo * prev;
    }
    L += g.emissive.rgb;

    payload.radiance = L;
    payload.hitT     = t;
}
