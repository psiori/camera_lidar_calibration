// scene_shapes.cpp — psiviz rendering, input, or geometry implementation.

#include "psiviz/scene_shapes.hpp"

#include <cmath>

namespace psiviz {

namespace {

void appendTriangle(std::vector<ShapeVertex>& out, const clc::Vec3f& a, const clc::Vec3f& b, const clc::Vec3f& c,
                    float r, float g, float bl, float alpha) {
  out.push_back({a.x, a.y, a.z, r, g, bl, alpha});
  out.push_back({b.x, b.y, b.z, r, g, bl, alpha});
  out.push_back({c.x, c.y, c.z, r, g, bl, alpha});
}

}  // namespace

void appendSphereTriangles(std::vector<ShapeVertex>& out, const SphereMarker& sphere, int stacks, int slices) {
  if (sphere.radius <= 0.f || stacks < 2 || slices < 3) {
    return;
  }
  const float r = sphere.color.x;
  const float g = sphere.color.y;
  const float b = sphere.color.z;
  const float alpha = sphere.alpha;
  const clc::Vec3f c = sphere.center;
  const float rad = sphere.radius;

  for (int i = 0; i < stacks; ++i) {
    const float v0 = static_cast<float>(i) / static_cast<float>(stacks);
    const float v1 = static_cast<float>(i + 1) / static_cast<float>(stacks);
    const float phi0 = v0 * 3.14159265f;
    const float phi1 = v1 * 3.14159265f;
    for (int j = 0; j < slices; ++j) {
      const float u0 = static_cast<float>(j) / static_cast<float>(slices);
      const float u1 = static_cast<float>(j + 1) / static_cast<float>(slices);
      const float theta0 = u0 * 6.2831853f;
      const float theta1 = u1 * 6.2831853f;

      const auto sph = [](float phi, float theta) {
        const float sp = std::sin(phi);
        return clc::Vec3f{sp * std::cos(theta), std::cos(phi), sp * std::sin(theta)};
      };

      const clc::Vec3f p00 = c + sph(phi0, theta0) * rad;
      const clc::Vec3f p01 = c + sph(phi0, theta1) * rad;
      const clc::Vec3f p10 = c + sph(phi1, theta0) * rad;
      const clc::Vec3f p11 = c + sph(phi1, theta1) * rad;

      if (i > 0) {
        appendTriangle(out, p00, p10, p11, r, g, b, alpha);
      }
      if (i < stacks - 1) {
        appendTriangle(out, p00, p11, p01, r, g, b, alpha);
      }
    }
  }
}

void buildSphereMarkerMesh(std::vector<ShapeVertex>& out, const std::vector<SphereMarker>& spheres) {
  out.clear();
  for (const SphereMarker& sphere : spheres) {
    appendSphereTriangles(out, sphere);
  }
}

}  // namespace psiviz
