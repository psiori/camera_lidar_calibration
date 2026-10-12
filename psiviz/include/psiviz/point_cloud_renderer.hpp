#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "psiviz/point_cloud_buffers.hpp"
#include "psiviz/pose_gizmo.hpp"
#include "psiviz/scene_shapes.hpp"
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
  /// Schedules a GPU pick; returns a request id to pass to tryConsumePickResult.
  [[nodiscard]] std::uint64_t requestPick(int x, int y, float dpr, float point_size);
  /// True when the GPU readback for `request_id` is ready. `point_id` is 1-based; 0 means miss.
  [[nodiscard]] bool tryConsumePickResult(std::uint64_t request_id, std::uint32_t& point_id);
  void setSphereMarkers(const std::vector<SphereMarker>& markers);
  void render(QRhiCommandBuffer* cb, const TrackballCamera& camera, const PoseGizmo& gizmo, QRhiRenderTarget* rt);

 private:
  struct PickRequest {
    std::uint64_t request_id{0};
    int x{0};
    int y{0};
    float dpr{1.f};
    float point_size{1.f};
  };

  void ensurePipeline(QRhiRenderTarget* rt);
  void ensureGizmoPipeline(QRhiRenderTarget* rt);
  void ensureShapePipeline(QRhiRenderTarget* rt);
  void ensurePickTarget(int width, int height);
  void uploadGizmo(const PoseGizmo& gizmo);
  void uploadSphereMarkers();
  void renderPickPass(QRhiCommandBuffer* cb, const TrackballCamera& camera, int fb_w, int fb_h,
                      const PickRequest& request);
  void pollPickReadback();
  void schedulePickPassIfIdle();
  void releaseResources();

  QRhi* rhi_{nullptr};
  struct RhiResources;
  std::unique_ptr<RhiResources> rhi_resources_;
  int point_count_{0};
  int width_{0};
  int height_{0};
  float blend_weight_{0.f};
  float scalar_min_{0.f};
  float scalar_max_{0.f};
  bool initialized_{false};
  bool pipeline_ready_{false};
  bool pick_pass_scheduled_{false};
  std::uint64_t next_request_id_{1};
  std::optional<PickRequest> active_request_;
  std::optional<PickRequest> queued_request_;
  std::vector<std::pair<std::uint64_t, std::uint32_t>> completed_picks_;
  int gizmo_vertex_count_{0};
  int shape_vertex_count_{0};
  std::vector<SphereMarker> sphere_markers_;
};

}  // namespace psiviz
