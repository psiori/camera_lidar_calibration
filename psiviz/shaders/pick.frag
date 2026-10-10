#version 440

layout(location = 0) flat in uint vPickId;

layout(location = 0) out vec4 fragColor;

void main() {
    // Encode (point_index + 1) in RGB; alpha holds depth for fly-to / miss checks.
    uint enc = vPickId + 1u;
    float depth01 = clamp(gl_FragCoord.z, 0.0, 1.0);
    fragColor = vec4(float((enc >> 16u) & 255u) / 255.0,
                     float((enc >> 8u) & 255u) / 255.0,
                     float(enc & 255u) / 255.0,
                     depth01);
}
