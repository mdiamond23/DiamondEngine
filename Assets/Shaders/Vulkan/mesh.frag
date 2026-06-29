#version 450

// Samples the bound texture and applies simple directional (Lambert) lighting,
// proving the combined image-sampler descriptor and the interpolated normal/UV
// both reach the fragment stage.

layout(location = 0) in vec3 vNormalWS;
layout(location = 1) in vec2 vUV;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4 viewProj;
    vec4 lightDir;
    float time;
} ubo;

layout(set = 0, binding = 1) uniform sampler2D uTexture;

void main() {
    vec3 N = normalize(vNormalWS);
    vec3 L = normalize(-ubo.lightDir.xyz);   // direction toward the light
    float diffuse = max(dot(N, L), 0.0);
    float ambient = 0.2;

    vec3 albedo = texture(uTexture, vUV).rgb;
    outColor = vec4(albedo * (ambient + diffuse), 1.0);
}
