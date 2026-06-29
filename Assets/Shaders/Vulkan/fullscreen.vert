#version 450

// Vertex-less fullscreen triangle: three vertices generated from gl_VertexIndex
// that cover the whole target, with UVs spanning [0,1]. Bind no vertex buffer and
// draw 3. Used by post passes that sample an offscreen texture.

layout(location = 0) out vec2 vUV;

void main() {
    vUV = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    gl_Position = vec4(vUV * 2.0 - 1.0, 0.0, 1.0);
}
