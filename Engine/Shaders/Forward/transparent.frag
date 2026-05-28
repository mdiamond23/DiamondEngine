#version 410 core
layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec4 BrightColor;

in vec2 TexCoords;

uniform sampler2D albedo;

void main()
{
    vec4 color = texture(albedo, TexCoords);
    if (color.a < 0.01)
        discard;
    FragColor = color;
    BrightColor = vec4(0.0);
}
