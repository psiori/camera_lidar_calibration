#version 440

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;

layout(location = 0) out vec3 vColor;

layout(std140, binding = 0) uniform buf {
    mat4 mvp;
};

void main() {
    gl_Position = mvp * vec4(position, 1.0);
    vColor = color;
}
