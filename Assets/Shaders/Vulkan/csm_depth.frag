#version 450

// Depth-only — no color attachments. Depth is written by the fixed-function depth
// test; the fragment shader does nothing (matches Shadows/depth.frag).
void main() {}
