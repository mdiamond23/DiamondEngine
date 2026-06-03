#version 410 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

out vec3 WorldPos;
out vec3 Normal;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform mat3 normalMatrix;

void main() {
    WorldPos    = vec3(model * vec4(aPos, 1.0));
    Normal      = normalize(normalMatrix * aNormal);
    gl_Position = projection * view * vec4(WorldPos, 1.0);
}
