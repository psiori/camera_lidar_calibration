#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "psiviz/calibration_scene_overlays.hpp"
#include "psiviz/point_cloud_buffers.hpp"
#include "psiviz/pose_gizmo.hpp"
#include "psiviz/scene_shapes.hpp"
#include "psiviz/trackball_camera.hpp"

class QRhi;
class QRhiCommandBuffer;
class QRhiRenderTarget;

namespace psiviz {

/// Qt RHI backend: uploads point clouds, draws gizmo and markers, and runs GPU picking.
class PointCloudRenderer {
 public:
  explicit PointCloudRenderer(QRhi* rhi);
  ~PointCloudRenderer();

  /// Create shaders and buffers on the render thread (call once before first draw).
  void initialize(QRhiCommandBuffer* cb);
  /// Resize internal targets when the viewport changes.
  void resize(int width, int height);
  /// Upload new point positions, scalars, and colors to GPU memory.
  void uploadPointCloud(const PointCloudData& data, QRhiRenderTarget* rt = nullptr);
  [[nodiscard]] int pointCount() const { return point_count_; }
  [[nodiscard]] bool pipelineReady() const { return pipeline_ready_; }
  /// 0 = scalar colormap only, 1 = full camera RGB from per-point colors.
  void setBlendWeight(float w) { blend_weight_ = w; }
  /// Scalar range mapped to the JET LUT.
  void setScalarRange(float min_v, float max_v) {
    scalar_min_ = min_v;
    scalar_max_ = max_v;
  }
  /// Schedule a GPU color-id pick at framebuffer pixel (x,y); returns id for tryConsumePickResult.
  [[nodiscard]] std::uint64_t requestPick(int x, int y, float dpr, float point_size);
  /// When true, `point_id` is set (1-based index; 0 means no hit).
  [[nodiscard]] bool tryConsumePickResult(std::uint64_t request_id, std::uint32_t& point_id);
  void setSphereMarkers(const std::vector<SphereMarker>& markers);
  /// Lidar triad and camera frustum lines (camera pose comes from @p gizmo).
  void setCalibrationSceneOverlays(const CalibrationSceneOverlayState& state);
  /// Draw point cloud, pick pass if needed, scene overlays, and sphere markers.
  void render(QRhiCommandBuffer* cb, const TrackballCamera& camera, const PoseGizmo& gizmo, QRhiRenderTarget* rt);

 private:
  /// One in-flight or queued GPU pick at a screen location.
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
  void uploadSceneOverlays(const PoseGizmo& gizmo);
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
  CalibrationSceneOverlayState scene_overlays_{};
};

}  // namespace psiviz
