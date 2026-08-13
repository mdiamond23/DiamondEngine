#version 450

// RETAINED, UNUSED — superseded by gtao.comp (Docs/gi-design.md slice 4.5). Still
// compiled so it cannot rot; nothing loads the .spv. See VulkanSSAOPass.h for why
// this is no longer a drop-in (the lighting pass wants a bent normal, not a scalar).
//
// SSAO raw-occlusion pass (Vulkan port of Deferred/ssao.frag). A fullscreen pass
// over the G-buffer: for each fragment it reorients a 64-sample hemisphere kernel
// to the surface normal (via a tiled 4×4 noise rotation), projects each sample,
// and compares the sample's view-space depth against the stored geometry depth to
// estimate ambient occlusion. Output is single-channel (R16F).
//
// Two deltas from the GL source: TexCoords → fullscreen.vert's vUV, and the noise
// is an RGBA8 texture decoded here (*2-1) instead of a float RGB16F holding signed
// values directly (keeps the slice on the existing RGBA8 upload path). The
// projection→UV math is unchanged: vUV == ndc*0.5+0.5 holds under the uniform
// negative-height viewport, so no Y-flip is needed in the lookup.

layout(location = 0) in vec2 vUV;
layout(location = 0) out float FragColor;

layout(set = 0, binding = 0) uniform sampler2D gViewPos;
layout(set = 0, binding = 1) uniform sampler2D gViewNormal;
layout(set = 0, binding = 2) uniform sampler2D texNoise;

layout(set = 0, binding = 3) uniform SSAOUBO {
    mat4 projection;
    vec4 samples[64];   // std140: vec3 kernel padded to vec4 (.xyz used)
    vec2 noiseScale;    // (screenW / 4, screenH / 4)
} ubo;

const float radius = 0.5;
const float bias   = 0.025;

void main() {
    vec3 fragPos = texture(gViewPos,    vUV).xyz;
    vec3 normal  = normalize(texture(gViewNormal, vUV).xyz);

    // Background pixel — no geometry written here (gViewPos cleared to 0).
    if (fragPos.z == 0.0) { FragColor = 1.0; return; }

    // Random rotation vector, RGBA8-decoded to [-1,1] then tiled over the screen.
    vec3 randVec = normalize(texture(texNoise, vUV * ubo.noiseScale).xyz * 2.0 - 1.0);

    // TBN reorienting the kernel hemisphere to the surface normal.
    vec3 tangent   = normalize(randVec - normal * dot(randVec, normal));
    vec3 bitangent = cross(normal, tangent);
    mat3 TBN       = mat3(tangent, bitangent, normal);

    float occlusion = 0.0;
    for (int i = 0; i < 64; ++i) {
        vec3 samplePos = fragPos + TBN * ubo.samples[i].xyz * radius;

        // Project the sample to find the UV where the real geometry depth lives.
        vec4 offset = ubo.projection * vec4(samplePos, 1.0);
        offset.xyz /= offset.w;
        // v flipped: the negative-height viewport wrote NDC y=+1 to texel row 0.
        offset.xy   = vec2(0.5 * offset.x + 0.5, 0.5 - 0.5 * offset.y);

        float sampleDepth = texture(gViewPos, offset.xy).z;

        // View-space z is negative; a larger (closer) stored depth in front of the
        // sample means it's occluded.
        float rangeCheck = smoothstep(0.0, 1.0, radius / abs(fragPos.z - sampleDepth));
        occlusion += (sampleDepth >= samplePos.z + bias ? 1.0 : 0.0) * rangeCheck;
    }

    FragColor = 1.0 - (occlusion / 64.0);
}
