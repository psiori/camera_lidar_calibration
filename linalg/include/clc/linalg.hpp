#pragma once

/// @file linalg.hpp
/// Small fixed-size linear algebra types for CLC / psiviz public APIs.
///
/// Layout (Eigen / OpenGL / GLSL column-major compatible):
/// - Mat4f / Mat4d store 16 elements with index `m[col * 4 + row]` (i.e. `operator()(row, col)`).
/// - A 3x3 rotation occupies the upper-left; translation is column 3 rows 0..2; bottom row is (0,0,0,1) for rigid transforms.
/// - Vec3f / Vec3d are plain `{x,y,z}` with no padding requirement (not SIMD bundles).

#include <cmath>
#include <cstring>

namespace clc {

struct Vec2f {
  float x = 0.f;
  float y = 0.f;
  constexpr Vec2f() = default;
  constexpr Vec2f(float x_in, float y_in) : x(x_in), y(y_in) {}
};

struct Vec3f {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
};

struct Vec3d {
  double x = 0.;
  double y = 0.;
  double z = 0.;
};

struct Vec4f {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  float w = 1.f;
};

struct Vec4d {
  double x = 0.;
  double y = 0.;
  double z = 0.;
  double w = 1.;
};

struct Mat4f {
  float m[16]{};

  [[nodiscard]] float& operator()(int row, int col) { return m[col * 4 + row]; }
  [[nodiscard]] const float& operator()(int row, int col) const { return m[col * 4 + row]; }
  [[nodiscard]] const float* data() const { return m; }
  [[nodiscard]] float* data() { return m; }

  static Mat4f identity() {
    Mat4f out{};
    out(0, 0) = out(1, 1) = out(2, 2) = out(3, 3) = 1.f;
    return out;
  }

  static Mat4f zero() { return {}; }
};

struct Mat4d {
  double m[16]{};

  [[nodiscard]] double& operator()(int row, int col) { return m[col * 4 + row]; }
  [[nodiscard]] const double& operator()(int row, int col) const { return m[col * 4 + row]; }
  [[nodiscard]] const double* data() const { return m; }
  [[nodiscard]] double* data() { return m; }

  static Mat4d identity() {
    Mat4d out{};
    out(0, 0) = out(1, 1) = out(2, 2) = out(3, 3) = 1.;
    return out;
  }
};

inline Vec2f operator+(Vec2f a, Vec2f b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2f operator-(Vec2f a, Vec2f b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2f operator*(Vec2f a, float s) { return {a.x * s, a.y * s}; }
inline float dot(Vec2f a, Vec2f b) { return a.x * b.x + a.y * b.y; }
inline float lengthSquared(Vec2f v) { return dot(v, v); }
inline float length(Vec2f v) { return std::sqrt(lengthSquared(v)); }
inline Vec2f normalized(Vec2f v) {
  const float n = length(v);
  if (n < 1e-20f) {
    return {};
  }
  return v * (1.f / n);
}

inline Vec3f operator+(Vec3f a, Vec3f b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3f operator-(Vec3f a, Vec3f b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3f operator*(Vec3f a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3f operator*(float s, Vec3f a) { return a * s; }
inline Vec3f operator/(Vec3f a, float s) { return a * (1.f / s); }
inline float dot(Vec3f a, Vec3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3f cross(Vec3f a, Vec3f b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float lengthSquared(Vec3f v) { return dot(v, v); }
inline float length(Vec3f v) { return std::sqrt(lengthSquared(v)); }
inline Vec3f normalized(Vec3f v) {
  const float n = length(v);
  if (n < 1e-20f) {
    return {};
  }
  return v / n;
}

inline Vec3d operator+(Vec3d a, Vec3d b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3d operator-(Vec3d a, Vec3d b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline double dot(Vec3d a, Vec3d b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Mat4f multiply(Mat4f a, Mat4f b) {
  Mat4f out{};
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      float sum = 0.f;
      for (int k = 0; k < 4; ++k) {
        sum += a(row, k) * b(k, col);
      }
      out(row, col) = sum;
    }
  }
  return out;
}

inline Vec4f mulMat4Vec4(Mat4f a, Vec4f v) {
  const float vi[4] = {v.x, v.y, v.z, v.w};
  Vec4f out{};
  for (int row = 0; row < 4; ++row) {
    float sum = 0.f;
    for (int col = 0; col < 4; ++col) {
      sum += a(row, col) * vi[col];
    }
    (&out.x)[row] = sum;
  }
  return out;
}

inline Vec3f transformPoint(Mat4f m, Vec3f p) {
  const Vec4f h = mulMat4Vec4(m, {p.x, p.y, p.z, 1.f});
  return {h.x, h.y, h.z};
}

inline Vec3f transformVector(Mat4f m, Vec3f v) {
  return {m(0, 0) * v.x + m(0, 1) * v.y + m(0, 2) * v.z, m(1, 0) * v.x + m(1, 1) * v.y + m(1, 2) * v.z,
          m(2, 0) * v.x + m(2, 1) * v.y + m(2, 2) * v.z};
}

inline Vec3f translation(Mat4f m) { return {m(0, 3), m(1, 3), m(2, 3)}; }

inline void setTranslation(Mat4f& m, Vec3f t) {
  m(0, 3) = t.x;
  m(1, 3) = t.y;
  m(2, 3) = t.z;
}

inline Vec3f rotationColumn(Mat4f m, int col) { return {m(0, col), m(1, col), m(2, col)}; }

inline void setRotationColumn(Mat4f& m, int col, Vec3f v) {
  m(0, col) = v.x;
  m(1, col) = v.y;
  m(2, col) = v.z;
}

inline Mat4f rotationAxisAngle(Vec3f axis_unit, float angle_rad) {
  const float c = std::cos(angle_rad);
  const float s = std::sin(angle_rad);
  const float t = 1.f - c;
  const float x = axis_unit.x;
  const float y = axis_unit.y;
  const float z = axis_unit.z;
  Mat4f r = Mat4f::identity();
  r(0, 0) = t * x * x + c;
  r(0, 1) = t * x * y + s * z;
  r(0, 2) = t * x * z - s * y;
  r(1, 0) = t * x * y - s * z;
  r(1, 1) = t * y * y + c;
  r(1, 2) = t * y * z + s * x;
  r(2, 0) = t * x * z + s * y;
  r(2, 1) = t * y * z - s * x;
  r(2, 2) = t * z * z + c;
  return r;
}

inline Vec3f rotateAroundAxis(Vec3f point, Vec3f center, Vec3f axis_unit, float angle_rad) {
  const Mat4f rot = rotationAxisAngle(axis_unit, angle_rad);
  return center + transformVector(rot, point - center);
}

inline void rotateVectorInPlace(Vec3f& v, Vec3f axis_unit, float angle_rad) {
  v = transformVector(rotationAxisAngle(axis_unit, angle_rad), v);
}

inline Mat4f lookAt(Vec3f eye, Vec3f center, Vec3f up) {
  Vec3f f = center - eye;
  if (lengthSquared(f) < 1e-12f) {
    f = {0.f, 0.f, -1.f};
  } else {
    f = normalized(f);
  }
  Vec3f s = cross(f, up);
  if (lengthSquared(s) < 1e-12f) {
    s = cross(f, {0.f, 1.f, 0.f});
  }
  s = normalized(s);
  const Vec3f u = cross(s, f);
  Mat4f view = Mat4f::identity();
  // Row-wise upper 3×3 (legacy Eigen TrackballCamera layout). Matches GLSL `mvp * vec4` in psiviz;
  // a column-wise s/u/-f basis is the transpose and breaks orbit/spin feel.
  view(0, 0) = s.x;
  view(0, 1) = s.y;
  view(0, 2) = s.z;
  view(1, 0) = u.x;
  view(1, 1) = u.y;
  view(1, 2) = u.z;
  view(2, 0) = -f.x;
  view(2, 1) = -f.y;
  view(2, 2) = -f.z;
  view(0, 3) = -dot(s, eye);
  view(1, 3) = -dot(u, eye);
  view(2, 3) = dot(f, eye);
  return view;
}

inline Mat4f perspective(float fovy_rad, float aspect, float z_near, float z_far) {
  const float f = 1.f / std::tan(fovy_rad * 0.5f);
  Mat4f proj = Mat4f::zero();
  proj(0, 0) = f / aspect;
  proj(1, 1) = f;
  proj(2, 2) = (z_far + z_near) / (z_near - z_far);
  proj(2, 3) = (2.f * z_far * z_near) / (z_near - z_far);
  proj(3, 2) = -1.f;
  return proj;
}

inline Mat4d multiply(Mat4d a, Mat4d b) {
  Mat4d out{};
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      double sum = 0.;
      for (int k = 0; k < 4; ++k) {
        sum += a(row, k) * b(k, col);
      }
      out(row, col) = sum;
    }
  }
  return out;
}

inline Vec4d mulMat4Vec4(Mat4d a, Vec4d v) {
  return {a(0, 0) * v.x + a(0, 1) * v.y + a(0, 2) * v.z + a(0, 3) * v.w,
          a(1, 0) * v.x + a(1, 1) * v.y + a(1, 2) * v.z + a(1, 3) * v.w,
          a(2, 0) * v.x + a(2, 1) * v.y + a(2, 2) * v.z + a(2, 3) * v.w,
          a(3, 0) * v.x + a(3, 1) * v.y + a(3, 2) * v.z + a(3, 3) * v.w};
}

inline Vec3d transformPoint(Mat4d m, Vec3d p) {
  const Vec4d h = mulMat4Vec4(m, {p.x, p.y, p.z, 1.});
  return {h.x, h.y, h.z};
}

inline Vec3d translation(Mat4d m) { return {m(0, 3), m(1, 3), m(2, 3)}; }

inline Mat4d inverseRigid(Mat4d t) {
  Mat4d inv = Mat4d::identity();
  for (int c = 0; c < 3; ++c) {
    for (int r = 0; r < 3; ++r) {
      inv(r, c) = t(c, r);
    }
  }
  const Vec3d tr = translation(t);
  inv(0, 3) = -(inv(0, 0) * tr.x + inv(0, 1) * tr.y + inv(0, 2) * tr.z);
  inv(1, 3) = -(inv(1, 0) * tr.x + inv(1, 1) * tr.y + inv(1, 2) * tr.z);
  inv(2, 3) = -(inv(2, 0) * tr.x + inv(2, 1) * tr.y + inv(2, 2) * tr.z);
  return inv;
}

inline Mat4d mat4dFromQuaternionTranslation(double qx, double qy, double qz, double qw, double tx, double ty,
                                            double tz) {
  const double xx = qx * qx;
  const double yy = qy * qy;
  const double zz = qz * qz;
  const double xy = qx * qy;
  const double xz = qx * qz;
  const double yz = qy * qz;
  const double wx = qw * qx;
  const double wy = qw * qy;
  const double wz = qw * qz;
  Mat4d m = Mat4d::identity();
  m(0, 0) = 1. - 2. * (yy + zz);
  m(0, 1) = 2. * (xy - wz);
  m(0, 2) = 2. * (xz + wy);
  m(1, 0) = 2. * (xy + wz);
  m(1, 1) = 1. - 2. * (xx + zz);
  m(1, 2) = 2. * (yz - wx);
  m(2, 0) = 2. * (xz - wy);
  m(2, 1) = 2. * (yz + wx);
  m(2, 2) = 1. - 2. * (xx + yy);
  m(0, 3) = tx;
  m(1, 3) = ty;
  m(2, 3) = tz;
  return m;
}

inline void mat4dToQuaternionTranslation(const Mat4d& m, double& qx, double& qy, double& qz, double& qw, double& tx,
                                         double& ty, double& tz) {
  tx = m(0, 3);
  ty = m(1, 3);
  tz = m(2, 3);
  const double tr = m(0, 0) + m(1, 1) + m(2, 2);
  if (tr > 0.) {
    const double s = std::sqrt(tr + 1.) * 2.;
    qw = 0.25 * s;
    qx = (m(2, 1) - m(1, 2)) / s;
    qy = (m(0, 2) - m(2, 0)) / s;
    qz = (m(1, 0) - m(0, 1)) / s;
  } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
    const double s = std::sqrt(1. + m(0, 0) - m(1, 1) - m(2, 2)) * 2.;
    qw = (m(2, 1) - m(1, 2)) / s;
    qx = 0.25 * s;
    qy = (m(0, 1) + m(1, 0)) / s;
    qz = (m(0, 2) + m(2, 0)) / s;
  } else if (m(1, 1) > m(2, 2)) {
    const double s = std::sqrt(1. + m(1, 1) - m(0, 0) - m(2, 2)) * 2.;
    qw = (m(0, 2) - m(2, 0)) / s;
    qx = (m(0, 1) + m(1, 0)) / s;
    qy = 0.25 * s;
    qz = (m(1, 2) + m(2, 1)) / s;
  } else {
    const double s = std::sqrt(1. + m(2, 2) - m(0, 0) - m(1, 1)) * 2.;
    qw = (m(1, 0) - m(0, 1)) / s;
    qx = (m(0, 2) + m(2, 0)) / s;
    qy = (m(1, 2) + m(2, 1)) / s;
    qz = 0.25 * s;
  }
}

inline Mat4f mat4fFromMat4d(const Mat4d& d) {
  Mat4f out{};
  for (int i = 0; i < 16; ++i) {
    out.m[i] = static_cast<float>(d.m[i]);
  }
  return out;
}

inline Mat4d mat4dFromMat4f(const Mat4f& f) {
  Mat4d out{};
  for (int i = 0; i < 16; ++i) {
    out.m[i] = static_cast<double>(f.m[i]);
  }
  return out;
}

/// Interpret `m16` as column-major 4×4 (Eigen / OpenGL layout).
inline Mat4d mat4dFromColumnMajor(const double* m16) {
  Mat4d out{};
  for (int i = 0; i < 16; ++i) {
    out.m[i] = m16[i];
  }
  return out;
}

inline Mat4d mat4dFromColumnMajor(const float* m16) {
  Mat4d out{};
  for (int i = 0; i < 16; ++i) {
    out.m[i] = static_cast<double>(m16[i]);
  }
  return out;
}

}  // namespace clc
