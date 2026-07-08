#version 450

// Copies a spot light's cached static-caster depth map into the working depth
// target, so this frame's dynamic casters can be drawn over it (static shadow
// caching). Paired with fullscreen.vert.
//
// The working map is cleared to 1.0 and the copy pipeline runs LessEqual +
// depthWrite, so every fragment (sampled depth <= 1.0) passes the test and writes
// its depth — reproducing the cached static occluders. Dynamic casters then draw
// with the normal Less pipeline and win only where they are closer.
//
// set 0, binding 0 = the static map. It is a plain (non-comparison) depth sampler,
// so a straight texel fetch returns the stored [0,1] depth.

layout(location = 0) in vec2 vUV;

layout(set = 0, binding = 0) uniform sampler2D uStaticDepth;

void main() {
    gl_FragDepth = texture(uStaticDepth, vUV).r;
}
