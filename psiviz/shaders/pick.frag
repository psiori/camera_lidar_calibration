#version 440

layout(location = 0) in float vScalar;
layout(location = 1) in vec3 vRgb;

layout(location = 0) out vec4 fragColor;

layout(std140, binding = 0) uniform buf {
    mat4 mvp;
    float pointSize;
    float blendWeight;
    float scalarMin;
    float scalarMax;
};

void main() {
    fragColor = vec4(gl_FragCoord.z, 0.0, 0.0, 1.0);
}
