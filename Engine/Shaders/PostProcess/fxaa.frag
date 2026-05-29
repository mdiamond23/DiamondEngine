#version 410 core
out vec4 FragColor;
in vec2 TexCoords;

uniform sampler2D screenTex;
uniform vec2      texelSize;   // 1.0 / resolution
uniform bool      enabled;

float luma(vec3 rgb) {
    return dot(rgb, vec3(0.299, 0.587, 0.114));
}

void main() {
    vec3 center = texture(screenTex, TexCoords).rgb;

    if (!enabled) {
        FragColor = vec4(center, 1.0);
        return;
    }

    // 4-corner + center luma samples
    vec3 nw = texture(screenTex, TexCoords + vec2(-1.0, -1.0) * texelSize).rgb;
    vec3 ne = texture(screenTex, TexCoords + vec2( 1.0, -1.0) * texelSize).rgb;
    vec3 sw = texture(screenTex, TexCoords + vec2(-1.0,  1.0) * texelSize).rgb;
    vec3 se = texture(screenTex, TexCoords + vec2( 1.0,  1.0) * texelSize).rgb;

    float lumaM  = luma(center);
    float lumaNW = luma(nw);
    float lumaNE = luma(ne);
    float lumaSW = luma(sw);
    float lumaSE = luma(se);

    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));
    float lumaRange = lumaMax - lumaMin;

    // Skip pixels that are clearly not on an edge
    if (lumaRange < max(0.0312, lumaMax * 0.125)) {
        FragColor = vec4(center, 1.0);
        return;
    }

    // Compute blend direction from diagonal luma gradients
    vec2 dir;
    dir.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
    dir.y =  ((lumaNW + lumaSW) - (lumaNE + lumaSE));

    float dirReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * 0.03125, 0.0078125);
    float rcpDirMin = 1.0 / (min(abs(dir.x), abs(dir.y)) + dirReduce);
    dir = clamp(dir * rcpDirMin, vec2(-8.0), vec2(8.0)) * texelSize;

    // Two-tap blend — inner pair + outer pair, keep whichever stays in the local range
    vec3 rgbA = 0.5 * (
        texture(screenTex, TexCoords + dir * (1.0/3.0 - 0.5)).rgb +
        texture(screenTex, TexCoords + dir * (2.0/3.0 - 0.5)).rgb);
    vec3 rgbB = rgbA * 0.5 + 0.25 * (
        texture(screenTex, TexCoords + dir * -0.5).rgb +
        texture(screenTex, TexCoords + dir *  0.5).rgb);

    float lumaB = luma(rgbB);
    FragColor = vec4(
        (lumaB < lumaMin || lumaB > lumaMax) ? rgbA : rgbB,
        1.0);
}
