#version 450

// Screen-space reflections. A fullscreen pass over the G-buffer: reflect the
// view ray about the stored view-space normal, then ray-march the reflection in
// view space — each step is projected to screen UV (same flip as ssao.frag's
// sample lookup) and its depth compared against gViewPos.z. On a hit, the lit
// scene color at the hit pixel becomes the reflection color; the alpha channel
// carries a confidence the composite uses to blend against the IBL fallback.
// Misses, screen-edge exits, and camera-facing rays fade to alpha 0.
//
// A coarse linear march finds the first depth crossing, then a short binary
// search refines the hit point (keeps step count low without banding).

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 FragColor;

layout(set = 0, binding = 0) uniform sampler2D gViewPos;
layout(set = 0, binding = 1) uniform sampler2D gViewNormal;
layout(set = 0, binding = 2) uniform sampler2D sceneColor;

layout(set = 0, binding = 3) uniform SSRUBO {
    mat4 projection;
    vec2 screenSize;
} ubo;

const int   MAX_STEPS    = 32;     // coarse linear march
const int   REFINE_STEPS = 6;      // binary-search refinement after a crossing
const float MAX_DISTANCE = 20.0;   // view-space units the ray may travel
const float THICKNESS    = 0.6;    // how "deep" a surface counts as solid
const float BIAS         = 0.05;   // self-intersection guard at the ray origin

// View space → screen UV. v flipped: the negative-height viewport wrote NDC
// y=+1 to texel row 0 (see ssao.frag's projected lookup).
vec2 ProjectToUV(vec3 p) {
    vec4 clip = ubo.projection * vec4(p, 1.0);
    clip.xy /= clip.w;
    return vec2(0.5 * clip.x + 0.5, 0.5 - 0.5 * clip.y);
}

// 1 in the screen interior, 0 at the borders — hides the hard cutoff where
// rays leave the frame.
float EdgeFade(vec2 uv) {
    vec2 f = smoothstep(0.0, 0.1, min(uv, 1.0 - uv));
    return f.x * f.y;
}

void main() {
    vec3 fragPos = texture(gViewPos,    vUV).xyz;
    vec3 normal  = normalize(texture(gViewNormal, vUV).xyz);

    // Background pixel — no geometry written here (gViewPos cleared to 0).
    if (fragPos.z == 0.0) { FragColor = vec4(0.0); return; }

    // Camera sits at the view-space origin, so the frag position doubles as
    // the incident ray direction.
    vec3 viewDir   = normalize(fragPos);
    vec3 reflected = reflect(viewDir, normal);

    // Rays bending back toward the camera (positive view z) march into space
    // the screen never captured — fade them out rather than show false hits.
    float dirFade = smoothstep(0.0, 0.15, -reflected.z);
    if (dirFade == 0.0) { FragColor = vec4(0.0); return; }

    // ── Coarse march: step until the ray falls behind stored geometry ───────────
    vec3 rayStep = reflected * (MAX_DISTANCE / float(MAX_STEPS));
    vec3 pos     = fragPos;
    vec3 prev    = pos;
    bool hit     = false;

    for (int i = 0; i < MAX_STEPS; ++i) {
        prev = pos;
        pos += rayStep;

        vec2 uv = ProjectToUV(pos);
        if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) break;

        float sceneZ = texture(gViewPos, uv).z;
        if (sceneZ == 0.0) continue;   // ray currently over a sky pixel

        // View z is negative; the ray is behind the surface once pos.z drops
        // below sceneZ. THICKNESS rejects crossings far behind thin geometry.
        if (sceneZ >= pos.z + BIAS && sceneZ - pos.z < THICKNESS) {
            hit = true;
            break;
        }
    }

    if (!hit) { FragColor = vec4(0.0); return; }

    // ── Binary refinement between the last in-front and first behind points ─────
    for (int i = 0; i < REFINE_STEPS; ++i) {
        vec3 mid     = 0.5 * (prev + pos);
        float sceneZ = texture(gViewPos, ProjectToUV(mid)).z;
        if (sceneZ != 0.0 && sceneZ >= mid.z + BIAS) pos = mid;
        else                                         prev = mid;
    }

    vec2 hitUV = ProjectToUV(pos);
    FragColor  = vec4(texture(sceneColor, hitUV).rgb, dirFade * EdgeFade(hitUV));
}
