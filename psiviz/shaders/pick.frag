#version 440

layout(location = 0) out vec4 fragColor;

void main() {
    // Depth-only pick pass. Store normalized depth in R (some backends mishandle alpha readback).
    float depth01 = clamp(gl_FragCoord.z, 0.0, 1.0);
    fragColor = vec4(depth01, 0.0, 0.0, 1.0);
}
