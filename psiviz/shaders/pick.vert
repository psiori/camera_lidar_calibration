#version 440

layout(location = 0) in vec3 position;
layout(location = 1) in float scalar;
layout(location = 2) in vec3 rgb;

layout(std140, binding = 0) uniform buf {
    mat4 mvp;
    float pointSize;
    float blendWeight;
    float scalarMin;
    float scalarMax;
};

layout(location = 0) flat out uint vPointId;

void main() {
    gl_Position = mvp * vec4(position, 1.0);
    gl_PointSize = pointSize;
    vPointId = uint(gl_VertexIndex) + 1u;
}
