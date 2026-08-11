#version 450

// Auto-exposure, stage 3: blend this frame's scene log-luminance toward the
// retained value (eye adaptation) and turn it into an exposure multiplier.
//
// 1x1, two outputs, mirroring the TAA resolve's trick — a pass cannot read the
// texture it writes, so the retained value is sampled RAW from a pass-owned
// single-buffered image while the new value goes to a graph target that gets
// copied back after the graph runs:
//   location 0 = exposure multiplier, consumed by bloom + tonemap this frame
//   location 1 = adapted log-luminance, copied into the retained image
//
// Blending happens in LOG space, so adaptation moves at a constant rate in
// stops per second regardless of absolute scene brightness — a 10x brightness
// jump takes the same time to settle as any other 10x jump.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out float outExposure;
layout(location = 1) out float outAdapted;

layout(set = 0, binding = 0) uniform sampler2D uSceneLogLuma;   // 1x1, this frame
layout(set = 0, binding = 1) uniform sampler2D uPrevAdapted;    // 1x1, retained

layout(push_constant) uniform Push {
    float alpha;        // temporal weight toward this frame (1.0 = snap, no history)
    float keyValue;     // luminance the adapted average is mapped to (middle grey)
    float minLogLuma;   // clamp on the measured average — the metering range
    float maxLogLuma;
    int   autoEnabled;  // 0 = pass 1.0 through, leaving manual exposure in charge
} pc;

void main() {
    float scene = clamp(texture(uSceneLogLuma, vec2(0.5)).r, pc.minLogLuma, pc.maxLogLuma);
    float prev  = texture(uPrevAdapted, vec2(0.5)).r;

    // alpha is 1.0 on the first frame (and after an invalidate), so the retained
    // value seeds from the scene instead of from whatever the image held.
    float adapted = mix(prev, scene, pc.alpha);
    outAdapted = adapted;

    // Map the adapted average onto the key value. exp2 of a clamped log is
    // always well above zero, so no divide guard is needed beyond the clamp.
    outExposure = (pc.autoEnabled != 0) ? pc.keyValue / exp2(adapted) : 1.0;
}
