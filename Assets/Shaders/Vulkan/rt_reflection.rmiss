#version 460
#extension GL_EXT_ray_tracing : require

// Reflection ray miss: the sky. Samples the same baked radiance cubemap the
// skybox draws, scaled by SkyLightComponent::intensity, so a reflection of the
// sky matches the sky itself. Modelled on ddgi_probe_trace.rmiss.

layout(set = 0, binding = 6, std140) uniform ReflectBlock {
    mat4  view;
    mat4  invView;
    vec4  ddgiOrigin;
    vec4  ddgiSpacing;
    ivec4 ddgiCounts;
    vec4  ddgiParams;
    vec4  params;       // x maxRayDistance, y skyIntensity
} r;

layout(set = 0, binding = 12) uniform samplerCube uEnvMap;

layout(location = 0) rayPayloadInEXT vec3 payload;

void main() {
    payload = textureLod(uEnvMap, gl_WorldRayDirectionEXT, 0.0).rgb * r.params.y;
}
