#version 450

// Diffuse irradiance convolution. Vulkan port of Engine/Shaders/IBL/irradiance.frag:
// integrates the environment cubemap over the hemisphere around each direction to
// produce the cosine-weighted irradiance map the PBR ambient term samples by world
// normal. Paired with cubemap.vert.

layout(location = 0) in  vec3 vWorldPos;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform samplerCube environmentMap;

const float PI = 3.14159265359;

void main() {
    vec3 normal = normalize(vWorldPos);

    vec3 irradiance = vec3(0.0);

    vec3 up    = vec3(0.0, 1.0, 0.0);
    vec3 right = normalize(cross(up, normal));
    up         = normalize(cross(normal, right));

    float sampleDelta = 0.025;
    float nrSamples   = 0.0;
    for (float phi = 0.0; phi < 2.0 * PI; phi += sampleDelta) {
        for (float theta = 0.0; theta < 0.5 * PI; theta += sampleDelta) {
            // spherical → cartesian (tangent space), then tangent → world
            vec3 tangentSample = vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
            vec3 sampleVec = tangentSample.x * right + tangentSample.y * up + tangentSample.z * normal;

            irradiance += texture(environmentMap, sampleVec).rgb * cos(theta) * sin(theta);
            nrSamples++;
        }
    }
    irradiance = PI * irradiance * (1.0 / nrSamples);

    outColor = vec4(irradiance, 1.0);
}
