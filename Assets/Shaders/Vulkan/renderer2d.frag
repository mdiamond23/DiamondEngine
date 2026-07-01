#version 450

// Port of OpenGLRenderer2D's inline fragment shader. mode 0: modulate RGBA (a 1x1
// white texture -> solid color, a sprite -> tinted texel). mode 1: text — the
// texture's R channel is glyph coverage, used as the alpha.
layout(location = 0) in vec2  vUV;
layout(location = 1) in vec4  vColor;
layout(location = 2) in float vMode;

layout(set = 0, binding = 0) uniform sampler2D uTex;

layout(location = 0) out vec4 FragColor;

void main() {
    vec4 s = texture(uTex, vUV);
    FragColor = (vMode > 0.5)
        ? vec4(vColor.rgb, vColor.a * s.r)
        : vColor * s;
}
