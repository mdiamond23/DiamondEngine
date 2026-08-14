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

// Screen-space DDA march to the first depth crossing, then a short binary
// refinement. Returns true on hit and writes the hit's screen UV.
//
// The projection happens TWICE — once per endpoint — instead of once per step.
// A straight line in view space stays straight under a projective transform, so
// screen position is affine in a screen-space parameter, and 1/w is affine in
// that same parameter. Interpolating both is exact, not an approximation, and
// it removes a mat4 multiply from the innermost loop (SSGI was doing ~180M of
// them per frame at 16 rays x 22 steps).
//
// It also samples BETTER: stepping uniformly in view space clusters taps near
// the origin in screen space and strides over distant geometry. Uniform screen
// steps put one tap every few pixels along the whole ray.
bool TraceScreenRay(sampler2D gViewPos, mat4 proj, vec3 origin, vec3 dir,
                    float maxDistance, int steps, float thickness, float bias,
                    out vec2 hitUV)
{
    hitUV = vec2(0.0);

    // Clip the segment to stay in front of the camera BEFORE projecting. With a
    // per-step projection a ray crossing the near plane produced garbage UVs
    // that the bounds test happened to reject; interpolated endpoints have no
    // such safety net, so a segment straddling w = 0 must be shortened here.
    const float kMinViewZ = -0.05;   // just in front of the camera
    float endT = maxDistance;
    if (origin.z + dir.z * endT > kMinViewZ) {
        if (dir.z <= 1e-6) return false;              // parallel or receding
        endT = (kMinViewZ - origin.z) / dir.z;
        if (endT <= 1e-4) return false;               // origin already at the plane
    }

    vec4 c0 = proj * vec4(origin, 1.0);
    vec4 c1 = proj * vec4(origin + dir * endT, 1.0);
    if (c0.w <= 0.0 || c1.w <= 0.0) return false;

    const vec2  uv0 = vec2(0.5 * c0.x / c0.w + 0.5, 0.5 - 0.5 * c0.y / c0.w);
    const vec2  uv1 = vec2(0.5 * c1.x / c1.w + 0.5, 0.5 - 0.5 * c1.y / c1.w);
    // w == -viewZ for a standard perspective matrix, so 1/w recovers view depth.
    const float iw0 = 1.0 / c0.w;
    const float iw1 = 1.0 / c1.w;

    float prevT = 0.0;
    float hitT  = -1.0;

    for (int i = 1; i <= steps; ++i) {
        float t  = float(i) / float(steps);
        vec2  uv = mix(uv0, uv1, t);
        if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) return false;

        float rayZ   = -1.0 / mix(iw0, iw1, t);
        float sceneZ = textureLod(gViewPos, uv, 0.0).z;
        if (sceneZ == 0.0) { prevT = t; continue; }   // ray is over a sky pixel

        // View z is negative; the ray is behind stored geometry once rayZ drops
        // below sceneZ. thickness rejects crossings far behind thin geometry.
        if (sceneZ >= rayZ + bias && sceneZ - rayZ < thickness) { hitT = t; break; }
        prevT = t;
    }

    if (hitT < 0.0) return false;

    // Bisect between the last miss and the hit, in the same interpolated space.
    float lo = prevT;
    float hi = hitT;
    for (int i = 0; i < SCREEN_TRACE_REFINE_STEPS; ++i) {
        float mid    = 0.5 * (lo + hi);
        float rayZ   = -1.0 / mix(iw0, iw1, mid);
        float sceneZ = textureLod(gViewPos, mix(uv0, uv1, mid), 0.0).z;
        if (sceneZ != 0.0 && sceneZ >= rayZ + bias) hi = mid;
        else                                        lo = mid;
    }

    hitUV = mix(uv0, uv1, hi);
    return true;
}

#endif // SCREEN_TRACE_GLSL
