#version 440

layout(location = 0) flat in uint vPointId;

layout(location = 0) out vec4 fragPickId;

void main() {
    vec2 d = gl_PointCoord.xy - vec2(0.5);
    if (dot(d, d) > 0.25) {
        discard;
    }
    uint id = vPointId;
    fragPickId = vec4(float(id & 0xffu),
                      float((id >> 8u) & 0xffu),
                      float((id >> 16u) & 0xffu),
                      float((id >> 24u) & 0xffu)) / 255.0;
}
