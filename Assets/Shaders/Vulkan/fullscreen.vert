#version 450

// Vertex-less fullscreen triangle: three vertices generated from gl_VertexIndex
// that cover the whole target, with UVs spanning [0,1]. Bind no vertex buffer and
// draw 3. Used by post passes that sample an offscreen texture.

layout(location = 0) out vec2 vUV;

void main() {
    vec2 pos = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    // vUV.y runs opposite to NDC y: the backend's negative-height viewport puts
    // NDC y=+1 at texel row 0, and sampler v=0 reads row 0 — so v must decrease
    // as NDC y increases, or every fullscreen pass mirrors its input vertically.
    vUV = vec2(pos.x, 1.0 - pos.y);
    gl_Position = vec4(pos * 2.0 - 1.0, 0.0, 1.0);
}
