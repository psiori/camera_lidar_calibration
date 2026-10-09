#pragma once

#include <memory>

#include <Eigen/Core>

#include "psiviz/point_cloud_buffers.hpp"
#include "psiviz/pose_gizmo.hpp"
#include "psiviz/trackball_camera.hpp"

class QRhi;
class QRhiCommandBuffer;
class QRhiRenderTarget;

namespace psiviz {

class PointCloudRenderer {
 public:
  explicit PointCloudRenderer(QRhi* rhi);
  ~PointCloudRenderer();

  void initialize(QRhiCommandBuffer* cb);
  void resize(int width, int height);
  void uploadPointCloud(const PointCloudData& data, QRhiRenderTarget* rt = nullptr);
  [[nodiscard]] int pointCount() const { return point_count_; }
  [[nodiscard]] bool pipelineReady() const { return pipeline_ready_; }
  void setBlendWeight(float w) { blend_weight_ = w; }
  void setScalarRange(float min_v, float max_v) {
    scalar_min_ = min_v;
    scalar_max_ = max_v;
  }
  void requestPick(int x, int y, float dpr);
  void render(QRhiCommandBuffer* cb, const TrackballCamera& camera, const PoseGizmo& gizmo, QRhiRenderTarget* rt);

  [[nodiscard]] float lastPickedDepth() const { return last_picked_depth_; }

 private:
  void ensurePipeline(QRhiRenderTarget* rt);
  void ensureGizmoPipeline(QRhiRenderTarget* rt);
  void ensurePickTarget(int width, int height);
  void uploadGizmo(const PoseGizmo& gizmo);
  void renderPickPass(QRhiCommandBuffer* cb, const TrackballCamera& camera, int fb_w, int fb_h);
  void releaseResources();

  QRhi* rhi_{nullptr};
  struct RhiResources;
  std::unique_ptr<RhiResources> rhi_resources_;
  int point_count_{0};
  int width_{0};
  int height_{0};
  float blend_weight_{0.f};
  float scalar_min_{0.f};
  float scalar_max_{1.f};
  bool initialized_{false};
  bool pipeline_ready_{false};
  bool pick_requested_{false};
  int pick_x_{0};
  int pick_y_{0};
  float pick_dpr_{1.f};
  float last_picked_depth_{1.f};
  int gizmo_vertex_count_{0};
};

}  // namespace psiviz
