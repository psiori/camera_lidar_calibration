#pragma once

#include <vector>

#include <Eigen/Core>

namespace psiviz {

struct ShapeVertex {
  float px, py, pz;
  float r, g, b;
  float a;
};

struct SphereMarker {
  Eigen::Vector3f center{0.f, 0.f, 0.f};
  float radius{0.05f};
  /// Bright orange (sRGB-ish linear).
  Eigen::Vector3f color{1.f, 0.45f, 0.f};
  float alpha{0.5f};
};

/// Append a UV-sphere as triangle list (world-space vertices).
void appendSphereTriangles(std::vector<ShapeVertex>& out, const SphereMarker& sphere, int stacks = 12, int slices = 20);

void buildSphereMarkerMesh(std::vector<ShapeVertex>& out, const std::vector<SphereMarker>& spheres);

}  // namespace psiviz
