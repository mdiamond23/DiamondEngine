#version 450

// Deferred G-buffer geometry pass — full port of Engine/Shaders/Deferred/
// gbuffer.vert. Emits view-space position + a view-space TBN so the fragment
// stage can rotate the tangent-space normal map straight into view space (the
// space the whole deferred chain lights in). The normal matrix is derived from
// the model push constant in-shader (transpose-inverse — the pbr_surface.vert
// precedent) so non-uniform scale stays correct without a second push range.

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUV;
layout(location = 3) in vec3 inTangent;

layout(location = 0) out vec3 vViewPos;
layout(location = 1) out vec2 vUV;
layout(location = 2) out mat3 vTBN;   // consumes locations 2, 3, 4
// TAA motion: current/previous NDC xy (.xy = current, .zw = previous), already
// perspective-divided here and interpolated as NDC across the triangle — the
// standard TAA approximation (curved per-pixel motion vs. this linear
// interpolation differs by a sub-pixel amount in practice).
layout(location = 5) out vec4 vNdcCurrPrev;

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4 view;
    mat4 viewProj;               // JITTERED — feeds gl_Position only
    mat4 viewProjUnjittered;     // clean, this frame — velocity "current"
    mat4 prevViewProjUnjittered; // clean, last frame — velocity "previous"
} ubo;

layout(push_constant) uniform Push {
    mat4 model;
    mat4 prevModel;   // TAA velocity's per-object "previous" term
} pc;

void main() {
    vec4 worldPos = pc.model * vec4(inPos, 1.0);
    vViewPos = vec3(ubo.view * worldPos);
    vUV      = inUV;

    // Build the world-space TBN (Gram-Schmidt re-orthogonalized, bitangent from
    // the cross product — matches the GL shader), then rotate into view space.
    mat3 normalMatrix = transpose(inverse(mat3(pc.model)));
    vec3 T = normalize(normalMatrix * inTangent);
    vec3 N = normalize(normalMatrix * inNormal);
    T = normalize(T - dot(T, N) * N);
    vec3 B = cross(N, T);
    vTBN = mat3(ubo.view) * mat3(T, B, N);

    gl_Position = ubo.viewProj * worldPos;

    vec4 currClip = ubo.viewProjUnjittered * worldPos;
    vec4 prevClip = ubo.prevViewProjUnjittered * (pc.prevModel * vec4(inPos, 1.0));
    vNdcCurrPrev = vec4(currClip.xy / currClip.w, prevClip.xy / prevClip.w);
}
