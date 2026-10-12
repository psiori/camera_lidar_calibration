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

vec3 jet(float t) {
    t = clamp(t, 0.0, 1.0);
    float r = clamp(1.5 - abs(4.0 * t - 3.0), 0.0, 1.0);
    float g = clamp(1.5 - abs(4.0 * t - 2.0), 0.0, 1.0);
    float b = clamp(1.5 - abs(4.0 * t - 1.0), 0.0, 1.0);
    return vec3(r, g, b);
}

void main() {
    vec2 d = gl_PointCoord.xy - vec2(0.5);
    if (dot(d, d) > 0.25) {
        discard;
    }
    float denom = max(scalarMax - scalarMin, 1e-6);
    float t = (vScalar - scalarMin) / denom;
    vec3 cmap = jet(t);
    vec3 color = mix(cmap, vRgb, blendWeight);
    fragColor = vec4(color, 1.0);
}
