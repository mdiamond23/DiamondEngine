#version 410 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D hdrBuffer;
uniform sampler2D bloomBlur;
uniform bool bloom;
uniform float exposure;

vec3 ACESFilmic(vec3 x)
{
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

void main()
{
    vec3 hdrColor = texture(hdrBuffer, TexCoords).rgb;
    if (bloom)
        hdrColor += texture(bloomBlur, TexCoords).rgb;
    vec3 mapped = ACESFilmic(hdrColor * exposure);
    FragColor = vec4(pow(mapped, vec3(1.0 / 2.2)), 1.0);
}
