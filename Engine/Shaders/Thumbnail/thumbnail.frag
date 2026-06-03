#version 410 core
in vec3 WorldPos;
in vec3 Normal;
out vec4 FragColor;

uniform vec3 cameraPos;

void main() {
    vec3 N = normalize(Normal);
    vec3 V = normalize(cameraPos - WorldPos);

    // Key light — warm, upper front-right
    vec3 keyDir  = normalize(vec3(1.0, 1.5, 1.0));
    float keyDiff = max(dot(N, keyDir), 0.0);
    vec3 key = vec3(1.0, 0.93, 0.82) * keyDiff * 0.75;

    // Fill light — cool, lower back-left
    vec3 fillDir  = normalize(vec3(-1.0, 0.4, -0.8));
    float fillDiff = max(dot(N, fillDir), 0.0);
    vec3 fill = vec3(0.35, 0.45, 0.65) * fillDiff * 0.3;

    // Rim — blue-white edge glow
    float rim = pow(1.0 - max(dot(N, V), 0.0), 3.0);
    vec3 rimCol = vec3(0.4, 0.5, 0.7) * rim * 0.45;

    vec3 ambient  = vec3(0.12);
    vec3 meshColor = vec3(0.72, 0.72, 0.78);

    vec3 color = meshColor * (key + fill + ambient) + rimCol;
    FragColor  = vec4(color, 1.0);
}
