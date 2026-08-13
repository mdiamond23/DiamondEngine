#version 460
#extension GL_EXT_ray_tracing                    : require
#extension GL_EXT_buffer_reference2              : require
#extension GL_EXT_nonuniform_qualifier           : require
#extension GL_GOOGLE_include_directive           : require

#include "ddgi_common.glsl"

// Probe ray shading. Returns the outgoing radiance of the hit surface:
//
//   L = albedo/pi * E_direct  +  albedo * E_prev/pi  +  emissive
//
// The second term is the previous frame's probe irradiance — the recursive term
// that buys multi-bounce GI at ray depth 1, which is why maxRecursionDepth stays
// at 1 and no ray is ever traced from here.
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
// would be wrong to include) and shadowed only for the sun, via the same CSM maps
// the deferred pass uses. Point/spot lights contribute unshadowed. The math is
// deliberately a trimmed copy of deferred_lighting.frag rather than a shared
// include: that shader is a working Cook-Torrance resolve and this needs Lambert.

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

layout(set = 0, binding = 8)  uniform sampler2D shadowCascade0;
layout(set = 0, binding = 9)  uniform sampler2D shadowCascade1;
layout(set = 0, binding = 10) uniform sampler2D shadowCascade2;
layout(set = 0, binding = 11) uniform sampler2D shadowCascade3;

// Bindless base-color maps. Slot 0 is 1x1 white, so an untextured material
// resolves to its base-color factor alone — matching the G-buffer pass.
layout(set = 0, binding = 13) uniform sampler2D uAlbedoMaps[];

layout(location = 0) rayPayloadInEXT DDGIPayload payload;
hitAttributeEXT vec2 attribs;

// ── Shadowing (copied from deferred_lighting.frag, single-tap) ────────────────
// One tap rather than 3x3 PCF: a probe integrates hundreds of rays, so the
// filtering the raster pass needs to hide aliasing is wasted work here.
float SampleCascade(sampler2D smap, vec2 suv, float current, float bias) {
    if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) return 0.0;
    if (current > 1.0) return 0.0;
    return (current - bias > texture(smap, suv).r) ? 1.0 : 0.0;
}

float SunShadow(vec3 viewPos, vec3 N, vec3 L) {
    float depth = -viewPos.z;
    int c = 3;
    if      (depth < u.cascadeSplits.x) c = 0;
    else if (depth < u.cascadeSplits.y) c = 1;
    else if (depth < u.cascadeSplits.z) c = 2;

    vec4  lc      = u.lightFromView[c] * vec4(viewPos, 1.0);
    vec3  p       = lc.xyz / lc.w;
    // v flipped: the negative-height viewport wrote light NDC y=+1 to row 0.
    vec2  suv     = vec2(0.5 * p.x + 0.5, 0.5 - 0.5 * p.y);
    float bias    = max(0.004 * (1.0 - dot(N, L)), 0.001);

    if      (c == 0) return SampleCascade(shadowCascade0, suv, p.z, bias);
    else if (c == 1) return SampleCascade(shadowCascade1, suv, p.z, bias);
    else if (c == 2) return SampleCascade(shadowCascade2, suv, p.z, bias);
    else             return SampleCascade(shadowCascade3, suv, p.z, bias);
}

float Falloff(float dist, float radius) {
    float w = clamp(1.0 - pow(dist / radius, 4.0), 0.0, 1.0);
    return (w * w) / max(dist * dist, 1e-4);
}

// Total incident irradiance E at a surface, diffuse only.
vec3 DirectIrradiance(vec3 viewPos, vec3 N) {
    vec3 E = vec3(0.0);

    {   // Sun, CSM-shadowed. Outside the cascade coverage SampleCascade returns
        // 0 (unshadowed) — probes far behind the camera see an unoccluded sun.
        vec3  L     = normalize(-u.sunDirView.xyz);
        float NdotL = max(dot(N, L), 0.0);
        if (NdotL > 0.0)
            E += u.sunColor.xyz * NdotL * (1.0 - SunShadow(viewPos, N, L));
    }

    // Point + spot lights, UNSHADOWED (slice 3). Their cube/2D shadow maps are
    // bound to the deferred pass only; wiring them here is a slice-5 item, and
    // until then a local light can leak through a wall into a probe.
    int np = int(u.counts.x);
    for (int i = 0; i < np; ++i) {
        vec3  d    = u.pointPos[i].xyz - viewPos;
        float dist = length(d);
        if (dist >= u.pointPos[i].w) continue;
        float NdotL = max(dot(N, d / dist), 0.0);
        if (NdotL <= 0.0) continue;
        E += u.pointColor[i].xyz * Falloff(dist, u.pointPos[i].w) * NdotL;
    }

    int ns = int(u.counts.z);
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
    vec3 L = albedo * (DirectIrradiance(viewPos, viewN) / DDGI_PI);
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
