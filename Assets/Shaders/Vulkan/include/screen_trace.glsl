#ifndef SCREEN_TRACE_GLSL
#define SCREEN_TRACE_GLSL

// Shared view-space screen march, factored out of ssr.frag so SSGI and SSR
// can't drift apart. Everything is VIEW space: camera at the origin, -z
// forward, and gViewPos.z == 0 marks a background pixel.
//
// Sampling is textureLod(..., 0.0) throughout, never texture() — implicit-LOD
// sampling is a fragment-only operation and this header is included by
// compute shaders.

const int SCREEN_TRACE_REFINE_STEPS = 6;

// View space -> screen UV. The v flip is mandatory: the negative-height
// viewport writes NDC y=+1 into texel row 0.
vec2 ProjectToUV(mat4 proj, vec3 p) {
    vec4 clip = proj * vec4(p, 1.0);
    clip.xy /= clip.w;
    return vec2(0.5 * clip.x + 0.5, 0.5 - 0.5 * clip.y);
}

// 1 in the screen interior, 0 at the borders — hides the hard cutoff where
// rays leave the frame.
float EdgeFade(vec2 uv) {
    vec2 f = smoothstep(0.0, 0.1, min(uv, 1.0 - uv));
    return f.x * f.y;
}

// Coarse linear march to the first depth crossing, then a short binary
// refinement. Returns true on hit and writes the hit's screen UV.
bool TraceScreenRay(sampler2D gViewPos, mat4 proj, vec3 origin, vec3 dir,
                    float maxDistance, int steps, float thickness, float bias,
                    out vec2 hitUV)
{
    hitUV = vec2(0.0);

    vec3 rayStep = dir * (maxDistance / float(steps));
    vec3 pos     = origin;
    vec3 prev    = pos;
    bool hit     = false;

    for (int i = 0; i < steps; ++i) {
        prev = pos;
        pos += rayStep;

        vec2 uv = ProjectToUV(proj, pos);
        if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) return false;

        float sceneZ = textureLod(gViewPos, uv, 0.0).z;
        if (sceneZ == 0.0) continue;   // ray is over a sky pixel

        // View z is negative; the ray is behind stored geometry once pos.z
        // drops below sceneZ. thickness rejects crossings far behind thin geo.
        if (sceneZ >= pos.z + bias && sceneZ - pos.z < thickness) {
            hit = true;
            break;
        }
    }

    if (!hit) return false;

    for (int i = 0; i < SCREEN_TRACE_REFINE_STEPS; ++i) {
        vec3  mid    = 0.5 * (prev + pos);
        float sceneZ = textureLod(gViewPos, ProjectToUV(proj, mid), 0.0).z;
        if (sceneZ != 0.0 && sceneZ >= mid.z + bias) pos = mid;
        else                                         prev = mid;
    }

    hitUV = ProjectToUV(proj, pos);
    return true;
}

#endif // SCREEN_TRACE_GLSL
