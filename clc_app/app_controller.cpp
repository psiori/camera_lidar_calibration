#include "app_controller.hpp"

#include <QMetaObject>
#include <QPointF>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <Eigen/Geometry>
#include <opencv2/imgcodecs.hpp>

#include <camera/create_camera.hpp>
#include <QFileDialog>
#include <QFileInfo>

#include <clc_core/calibration_providers.hpp>
#include <clc_core/calibration_service.hpp>
#include <clc_core/point_color_model.hpp>
#include <clc_core/camera_intrinsics_io.hpp>
#include <clc_core/colormap.hpp>
#include <clc_core/fusion_service.hpp>
#include <clc_core/pcd_io.hpp>
#include <clc_core/range_image.hpp>
#include <clc_core/scan_geometry.hpp>

#include <psiviz/point_cloud_item.hpp>

namespace {

glim_cloud_fusion::LidarScan to_fusion_scan(const clc::PointXYZITCloud& cloud) {
  glim_cloud_fusion::LidarScan scan;
  scan.width = cloud.width;
  scan.height = cloud.height;
  scan.points = cloud.scan.points;
  scan.times = cloud.scan.times;
  scan.intensities = cloud.scan.intensities;
  scan.keep = cloud.scan.keep;
  return scan;
}

vlcal::FrameCPU::Ptr to_frame_cpu(const glim_cloud_fusion::DensePointCloud& cloud) {
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;
  std::vector<double> intensities;
  points.reserve(cloud.points.size());
  intensities.reserve(cloud.intensities.size());
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    points.emplace_back(cloud.points[i].x(), cloud.points[i].y(), cloud.points[i].z(), 1.0);
    intensities.push_back(i < cloud.intensities.size() ? cloud.intensities[i] : 0.0);
  }
  auto frame = std::make_shared<vlcal::FrameCPU>(points);
  frame->add_intensities(intensities);
  return frame;
}

QVariantList isometryToPoseMatrix(const Eigen::Isometry3d& T) {
  QVariantList out;
  const Eigen::Matrix4d m = T.matrix();
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      out.append(m(r, c));
    }
  }
  return out;
}

QImage rangeImageToDisplay(const clc::RangeImage& image, clc::RangeImageChannel channel, float min_v, float max_v, double y_stretch) {
  if (image.width <= 0 || image.height <= 0) {
    return {};
  }
  const std::vector<float>& values = channel == clc::RangeImageChannel::Range ? image.range : image.intensity;
  if (values.empty()) {
    return {};
  }
  std::vector<std::uint8_t> rgb;
  clc::colormap_jet(values, min_v, max_v, rgb);
  QImage out(image.width, image.height, QImage::Format_RGB888);
  if (static_cast<int>(rgb.size()) == image.width * image.height * 3) {
    std::memcpy(out.bits(), rgb.data(), rgb.size());
  }
  if (y_stretch > 0.0 && !qFuzzyCompare(y_stretch, 1.0)) {
    const int out_h = std::max(1, static_cast<int>(std::lround(static_cast<double>(image.height) * y_stretch)));
    out = out.scaled(image.width, out_h, Qt::IgnoreAspectRatio, Qt::FastTransformation);
  }
  return out;
}

float autoMaxValue(const std::vector<float>& values) {
  float max_v = 0.f;
  for (float v : values) {
    if (std::isfinite(v)) {
      max_v = std::max(max_v, v);
    }
  }
  return max_v > 0.f ? max_v : 1.f;
}

QString normalizedDirectoryPath(const QString& path) {
  if (path.isEmpty()) {
    return path;
  }
  QFileInfo info(path);
  if (info.isDir()) {
    return info.absoluteFilePath();
  }
  return info.absolutePath();
}

}  // namespace

AppController::AppController(QObject* parent) : QObject(parent) {}

AppController::~AppController() {
  if (worker_ && worker_->joinable()) {
    worker_->join();
  }
}

double AppController::scanDuration() const {
  return session_.scan_time.scan_duration.value_or(0.0);
}

int AppController::timeOrigin() const {
  return static_cast<int>(session_.scan_time.time_origin);
}

int AppController::timeUnit() const {
  return static_cast<int>(session_.scan_time.time_unit);
}

void AppController::setScanDuration(double seconds) {
  session_.scan_time.scan_duration = seconds;
  session_.fusion_params.scan_time = session_.scan_time;
  emit scanTimeChanged();
}

void AppController::setTimeOrigin(int origin) {
  session_.scan_time.time_origin = static_cast<glim_cloud_fusion::TimeOrigin>(origin);
  session_.fusion_params.scan_time = session_.scan_time;
  emit scanTimeChanged();
}

void AppController::setTimeUnit(int unit) {
  session_.scan_time.time_unit = static_cast<glim_cloud_fusion::TimeUnit>(unit);
  session_.fusion_params.scan_time = session_.scan_time;
  emit scanTimeChanged();
}

void AppController::setStatus(const QString& message) {
  status_message_ = message;
  emit statusMessageChanged();
}

void AppController::setPointCount(int count) {
  point_count_ = count;
  emit pointCountChanged();
}

QString AppController::imagePath() const {
  return QString::fromStdString(session_.image_path);
}

QString AppController::pcdDirectory() const {
  return QString::fromStdString(session_.cloud_directory);
}

QString AppController::intrinsicsPath() const {
  return QString::fromStdString(session_.intrinsics_path);
}

void AppController::setSessionDirectory(const QString& directory) {
  const QString normalized = normalizedDirectoryPath(directory);
  if (session_directory_ == normalized) {
    return;
  }
  session_directory_ = normalized;
  emit sessionPathsChanged();
}

void AppController::setImagePath(const QString& path) {
  const QString normalized = QFileInfo(path).absoluteFilePath();
  if (QString::fromStdString(session_.image_path) == normalized) {
    return;
  }
  session_.image_path = normalized.toStdString();
  session_.syncPrimaryFromLegacy();
  projection_image_.release();
  emit sessionPathsChanged();
}

void AppController::setPcdDirectory(const QString& path) {
  const QString normalized = normalizedDirectoryPath(path);
  if (QString::fromStdString(session_.cloud_directory) == normalized) {
    return;
  }
  session_.cloud_directory = normalized.toStdString();
  refreshPcdCatalog();
  session_.syncPrimaryFromLegacy();
  emit sessionPathsChanged();
}

void AppController::setIntrinsicsPath(const QString& path) {
  const QString normalized = QFileInfo(path).absoluteFilePath();
  if (QString::fromStdString(session_.intrinsics_path) == normalized) {
    return;
  }
  session_.intrinsics_path = normalized.toStdString();
  emit sessionPathsChanged();
}

void AppController::refreshPcdCatalog() {
  fusion_blocked_ = false;
  if (session_.cloud_directory.empty()) {
    session_.pcd_paths.clear();
    emit pcdCatalogChanged();
    return;
  }

  const auto scan = clc::scan_pcd_directory(
    session_.cloud_directory, session_.reference_scan_width, session_.reference_scan_height);
  if (!scan.error.empty()) {
    session_.pcd_paths.clear();
    fusion_blocked_ = true;
    setStatus(QString::fromStdString(scan.error));
    emit pcdCatalogChanged();
    return;
  }

  session_.pcd_paths = scan.paths;
  session_.reference_scan_width = scan.reference_width;
  session_.reference_scan_height = scan.reference_height;
  fused_cloud_ = {};
  session_.syncPrimaryFromLegacy();
  setStatus(QStringLiteral("Found %1 PCD files (%2×%3)")
              .arg(static_cast<int>(session_.pcd_paths.size()))
              .arg(session_.reference_scan_width)
              .arg(session_.reference_scan_height));
  emit pcdCatalogChanged();
  refreshRangeImage();
  refreshPreviewInViewer();
}

void AppController::refreshPreviewInViewer() {
  if (fused_cloud_.points.empty()) {
    uploadPreviewScanToViewer();
  }
}

QByteArray AppController::maskPreview() const {
  const int w = session_.reference_scan_width;
  const int h = session_.reference_scan_height;
  if (w <= 0 || h <= 0) {
    return {};
  }
  const std::size_t n = static_cast<std::size_t>(w * h);
  std::vector<std::uint8_t> preview;
  if (global_mask_pixels_.size() == n) {
    preview = global_mask_pixels_;
  } else {
    preview.assign(n, 255);
  }
  if (mask_polygon_pending_ && pending_mask_polygon_.vertices.size() >= 3) {
    const std::vector<std::uint8_t> pending_interior =
        clc::rasterize_mask(w, h, {pending_mask_polygon_}, false);
    if (pending_interior.size() == n) {
      for (std::size_t i = 0; i < n; ++i) {
        if (pending_interior[i] == 0) {
          preview[i] = 0;
        }
      }
    }
  }
  return QByteArray(reinterpret_cast<const char*>(preview.data()), static_cast<int>(preview.size()));
}

void AppController::setRangeColormapMin(double v) {
  if (qFuzzyCompare(range_colormap_min_, v)) {
    return;
  }
  range_colormap_min_ = v;
  refreshRangeImage();
}

void AppController::setRangeColormapMax(double v) {
  if (qFuzzyCompare(range_colormap_max_, v)) {
    return;
  }
  range_colormap_max_ = v;
  refreshRangeImage();
}

void AppController::setRangeChannel(int channel) {
  if (range_channel_ == channel) {
    return;
  }
  range_channel_ = channel;
  refreshRangeImage();
}

void AppController::setRangeYStretch(double stretch) {
  stretch = std::clamp(stretch, 0.25, 8.0);
  if (qFuzzyCompare(range_y_stretch_, stretch)) {
    return;
  }
  range_y_stretch_ = stretch;
  refreshRangeImage();
  emit rangeYStretchChanged();
}

void AppController::setViewerColormapMin(double v) {
  if (qFuzzyCompare(viewer_colormap_min_, v)) {
    return;
  }
  viewer_colormap_min_ = v;
  emit viewerColormapChanged();
  if (!fused_cloud_.points.empty()) {
    uploadFusedCloudToViewer();
  } else {
    uploadPreviewScanToViewer();
  }
}

void AppController::setViewerColormapMax(double v) {
  if (qFuzzyCompare(viewer_colormap_max_, v)) {
    return;
  }
  viewer_colormap_max_ = v;
  emit viewerColormapChanged();
  if (!fused_cloud_.points.empty()) {
    uploadFusedCloudToViewer();
  } else {
    uploadPreviewScanToViewer();
  }
}

void AppController::setViewerScalarChannel(int channel) {
  if (viewer_scalar_channel_ == channel) {
    return;
  }
  viewer_scalar_channel_ = channel;
  if (!fused_cloud_.points.empty()) {
    refreshViewerColormapBounds();
  }
  emit viewerColormapChanged();
  if (!fused_cloud_.points.empty()) {
    uploadFusedCloudToViewer();
  } else {
    uploadPreviewScanToViewer();
  }
}

void AppController::setFusionVoxelResolution(double v) {
  if (qFuzzyCompare(session_.fusion_params.voxel_resolution, v)) {
    return;
  }
  session_.fusion_params.voxel_resolution = v;
  emit fusionParamsChanged();
}

void AppController::setFusionNumThreads(int n) {
  if (session_.fusion_params.num_threads == n) {
    return;
  }
  session_.fusion_params.num_threads = std::max(1, n);
  emit fusionParamsChanged();
}

void AppController::setFusionMaxCorrespondence(double v) {
  if (qFuzzyCompare(session_.fusion_params.max_correspondence_distance, v)) {
    return;
  }
  session_.fusion_params.max_correspondence_distance = v;
  emit fusionParamsChanged();
}

void AppController::refreshViewerColors() {
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
  if (!item) {
    return;
  }
  refreshViewerRgb(static_cast<float>(item->blendWeight()));
}

void AppController::syncColorModelFromSession() {
  if (session_.camera.intrinsics.empty() || session_.image_path.empty() || viewer_positions_.isEmpty()) {
    return;
  }
  if (projection_image_.empty()) {
    projection_image_ = cv::imread(session_.image_path, cv::IMREAD_COLOR);
  }
  if (projection_image_.empty()) {
    return;
  }
  const auto camera =
    camera::create_camera(session_.camera.model, session_.camera.intrinsics, session_.camera.distortion_coeffs);
  color_model_.setCamera(camera);
  color_model_.setImage(projection_image_);

  const int n = viewer_positions_.size() / 3;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;
  std::vector<double> intensities;
  points.reserve(n);
  intensities.reserve(n);
  for (int i = 0; i < n; ++i) {
    points.emplace_back(viewer_positions_[i * 3], viewer_positions_[i * 3 + 1], viewer_positions_[i * 3 + 2]);
    intensities.push_back(i < viewer_scalars_.size() ? static_cast<double>(viewer_scalars_[i]) : 0.0);
  }
  color_model_.setPoints(points, intensities);
}

void AppController::refreshViewerRgb(float blend_weight) {
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
  if (!item || viewer_positions_.isEmpty()) {
    return;
  }
  if (!color_model_.ready()) {
    syncColorModelFromSession();
  }
  if (!color_model_.ready()) {
    viewer_rgb_.fill(0.5f, viewer_positions_.size() * 3);
    item->setPointCloudData(viewer_positions_, viewer_scalars_, viewer_rgb_);
    return;
  }
  std::vector<float> rgb;
  color_model_.projectColors(session_.T_camera_lidar, blend_weight, static_cast<float>(viewer_colormap_min_),
                             static_cast<float>(viewer_colormap_max_), rgb);
  viewer_rgb_ = QVector<float>(static_cast<int>(rgb.size()));
  if (!rgb.empty()) {
    std::memcpy(viewer_rgb_.data(), rgb.data(), rgb.size() * sizeof(float));
  }
  item->setPointCloudData(viewer_positions_, viewer_scalars_, viewer_rgb_);
}

bool AppController::buildAlignmentInput(clc::AlignmentInput& input, std::string& error) {
  static_assert(clc::OptimizerTraits<clc::NidOptimizerProvider>::supports_multi_dataset);
  if (session_.camera.intrinsics.empty()) {
    error = "camera intrinsics required";
    return false;
  }
  if (fused_cloud_.points.empty()) {
    error = "fused cloud required";
    return false;
  }
  projection_image_ = cv::imread(session_.image_path, cv::IMREAD_COLOR);
  if (projection_image_.empty()) {
    error = "failed to load image";
    return false;
  }
  input.camera = camera::create_camera(session_.camera.model, session_.camera.intrinsics, session_.camera.distortion_coeffs);
  input.init_T_camera_lidar = session_.T_camera_lidar_initial;
  input.params = session_.calibration_params;

  session_.ensurePrimaryCalibration();
  if (session_.calibration_datasets.size() > 1 &&
      clc::InitialGuessTraits<clc::ManualGuessProvider>::supports_multi_dataset) {
    for (const auto& ds : session_.calibration_datasets) {
      if (ds.image_path.empty()) {
        continue;
      }
      cv::Mat img = cv::imread(ds.image_path, cv::IMREAD_COLOR);
      if (img.empty()) {
        error = "failed to load dataset image: " + ds.image_path;
        return false;
      }
      input.images.push_back(img);
      input.clouds.push_back(to_frame_cpu(fused_cloud_));
    }
  }
  if (input.images.empty()) {
    input.images = {projection_image_};
    input.clouds = {to_frame_cpu(fused_cloud_)};
  }
  return true;
}

QString AppController::exportFusedPcd() {
  if (fused_cloud_.points.empty()) {
    setStatus(QStringLiteral("No fused cloud to export"));
    return {};
  }
  const QString path =
    QFileDialog::getSaveFileName(nullptr, QStringLiteral("Export fused PCD"), QString(), QStringLiteral("PCD (*.pcd)"));
  if (path.isEmpty()) {
    return {};
  }
  refreshViewerColors();
  std::string err;
  std::vector<float> rgb(static_cast<size_t>(viewer_rgb_.size()));
  for (int i = 0; i < viewer_rgb_.size(); ++i) {
    rgb[i] = viewer_rgb_[i];
  }
  if (!clc::save_dense_pcd_rgb(path.toStdString(), fused_cloud_, rgb, err)) {
    setStatus(QString::fromStdString(err));
    return {};
  }
  setStatus(QStringLiteral("Exported %1").arg(path));
  return path;
}

void AppController::refreshViewerColormapBounds() {
  if (fused_cloud_.points.empty()) {
    return;
  }
  float min_v = std::numeric_limits<float>::infinity();
  float max_v = -std::numeric_limits<float>::infinity();
  const int n = static_cast<int>(fused_cloud_.points.size());
  for (int i = 0; i < n; ++i) {
    float s = 0.f;
    switch (viewer_scalar_channel_) {
      case 1:
        s = fused_cloud_.points[i].x();
        break;
      case 2:
        s = fused_cloud_.points[i].y();
        break;
      case 3:
        s = fused_cloud_.points[i].z();
        break;
      default:
        s = i < static_cast<int>(fused_cloud_.intensities.size()) ? static_cast<float>(fused_cloud_.intensities[i]) : 0.f;
        break;
    }
    if (!std::isfinite(s)) {
      continue;
    }
    min_v = std::min(min_v, s);
    max_v = std::max(max_v, s);
  }
  if (!std::isfinite(min_v) || !std::isfinite(max_v) || max_v <= min_v) {
    min_v = 0.f;
    max_v = 1.f;
  }
  viewer_colormap_min_ = min_v;
  viewer_colormap_max_ = max_v;
}

void AppController::setMaskPolygonPending(bool pending) {
  if (mask_polygon_pending_ == pending) {
    return;
  }
  mask_polygon_pending_ = pending;
  emit maskPolygonPendingChanged();
}

void AppController::rebuildMaskFromOps() {
  const int w = session_.reference_scan_width;
  const int h = session_.reference_scan_height;
  if (w <= 0 || h <= 0) {
    emit maskChanged();
    return;
  }
  if (mask_ops_.empty()) {
    global_mask_pixels_.clear();
    emit maskChanged();
    return;
  }
  global_mask_pixels_ = clc::build_mask_from_ops(w, h, mask_ops_);
  session_.global_mask.width = w;
  session_.global_mask.height = h;
  emit maskChanged();
  if (fused_cloud_.points.empty()) {
    uploadPreviewScanToViewer();
  }
}

void AppController::commitMaskPolygon(const QVariantList& nativeVertices) {
  if (nativeVertices.size() < 3) {
    return;
  }
  pending_mask_polygon_ = {};
  for (const QVariant& v : nativeVertices) {
    const QPointF p = v.toPointF();
    pending_mask_polygon_.vertices.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()));
  }
  setMaskPolygonPending(true);
  emit maskChanged();
  setStatus(QStringLiteral("Polygon closed — choose Keep or Remove to apply"));
}

void AppController::applyPendingMaskKeep() {
  if (!mask_polygon_pending_) {
    return;
  }
  mask_ops_.push_back({pending_mask_polygon_, true});
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  rebuildMaskFromOps();
  setStatus(QStringLiteral("Mask updated (keep region, %1 ops)").arg(static_cast<int>(mask_ops_.size())));
}

void AppController::applyPendingMaskRemove() {
  if (!mask_polygon_pending_) {
    return;
  }
  mask_ops_.push_back({pending_mask_polygon_, false});
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  rebuildMaskFromOps();
  setStatus(QStringLiteral("Mask updated (remove region, %1 ops)").arg(static_cast<int>(mask_ops_.size())));
}

void AppController::discardPendingMaskPolygon() {
  if (!mask_polygon_pending_) {
    return;
  }
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  emit maskChanged();
  setStatus(QStringLiteral("Pending polygon discarded"));
}

void AppController::clearMask() {
  mask_ops_.clear();
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  global_mask_pixels_.clear();
  session_.global_mask.path.clear();
  session_.global_mask.width = 0;
  session_.global_mask.height = 0;
  emit maskChanged();
  if (fused_cloud_.points.empty()) {
    uploadPreviewScanToViewer();
  }
  setStatus(QStringLiteral("Mask cleared"));
}

bool AppController::saveMask() {
  const int w = session_.reference_scan_width;
  const int h = session_.reference_scan_height;
  if (w <= 0 || h <= 0 || global_mask_pixels_.size() != static_cast<std::size_t>(w * h)) {
    setStatus(QStringLiteral("No mask to save (draw polygons or load a mask first)"));
    return false;
  }
  QString dir = session_directory_;
  if (dir.isEmpty()) {
    dir = QFileDialog::getExistingDirectory(nullptr, QStringLiteral("Save mask to session folder"));
    if (dir.isEmpty()) {
      return false;
    }
    setSessionDirectory(dir);
  }
  const QString path = dir + QStringLiteral("/global_mask.png");
  if (!clc::save_mask_png(path.toStdString(), w, h, global_mask_pixels_)) {
    setStatus(QStringLiteral("Failed to write mask"));
    return false;
  }
  session_.global_mask.path = path.toStdString();
  session_.global_mask.width = w;
  session_.global_mask.height = h;
  emit maskChanged();
  setStatus(QStringLiteral("Mask saved to %1").arg(path));
  return true;
}

bool AppController::loadMask() {
  const int w = session_.reference_scan_width;
  const int h = session_.reference_scan_height;
  if (w <= 0 || h <= 0) {
    setStatus(QStringLiteral("Load structured PCDs before loading a mask"));
    return false;
  }
  QString path = maskPath();
  if (path.isEmpty()) {
    path = browseMaskFile();
    if (path.isEmpty()) {
      return false;
    }
  }
  std::string error;
  if (!clc::load_mask_png(path.toStdString(), w, h, global_mask_pixels_, error)) {
    setStatus(QString::fromStdString(error));
    return false;
  }
  mask_ops_.clear();
  setMaskPolygonPending(false);
  session_.global_mask.path = path.toStdString();
  session_.global_mask.width = w;
  session_.global_mask.height = h;
  emit maskChanged();
  setStatus(QStringLiteral("Mask loaded"));
  return true;
}

QString AppController::browseMaskFile() {
  const QString file = QFileDialog::getOpenFileName(
    nullptr,
    QStringLiteral("Load mask PNG"),
    session_directory_,
    QStringLiteral("PNG mask (*.png)"));
  if (file.isEmpty()) {
    return QString();
  }
  const int w = session_.reference_scan_width;
  const int h = session_.reference_scan_height;
  if (w <= 0 || h <= 0) {
    setStatus(QStringLiteral("Load structured PCDs before loading a mask"));
    return QString();
  }
  std::string error;
  if (!clc::load_mask_png(file.toStdString(), w, h, global_mask_pixels_, error)) {
    setStatus(QString::fromStdString(error));
    return QString();
  }
  mask_ops_.clear();
  setMaskPolygonPending(false);
  session_.global_mask.path = file.toStdString();
  session_.global_mask.width = w;
  session_.global_mask.height = h;
  emit maskChanged();
  setStatus(QStringLiteral("Mask loaded"));
  return file;
}

bool AppController::loadFirstReadableScan(clc::PointXYZITCloud& cloud, std::string& err) {
  for (const auto& path : session_.pcd_paths) {
    if (clc::load_pcd_xyzit(path, cloud, err)) {
      return true;
    }
  }
  if (err.empty()) {
    err = "no readable PCD in catalog";
  }
  return false;
}

void AppController::refreshRangeImage() {
  range_image_ = {};
  range_image_display_ = {};
  if (session_.pcd_paths.empty() || fusion_blocked_) {
    emit rangeImageChanged();
    return;
  }
  clc::PointXYZITCloud cloud;
  std::string err;
  if (!loadFirstReadableScan(cloud, err)) {
    setStatus(QString::fromStdString(err));
    emit rangeImageChanged();
    return;
  }
  range_image_ = clc::build_range_image(cloud);
  const auto channel = range_channel_ == 0 ? clc::RangeImageChannel::Range : clc::RangeImageChannel::Intensity;
  const std::vector<float>& values = channel == clc::RangeImageChannel::Range ? range_image_.range : range_image_.intensity;
  if (range_colormap_max_ <= range_colormap_min_) {
    range_colormap_min_ = 0.0;
    range_colormap_max_ = autoMaxValue(values);
  }
  range_image_display_ = rangeImageToDisplay(range_image_, channel, static_cast<float>(range_colormap_min_),
                                             static_cast<float>(range_colormap_max_), range_y_stretch_);
  emit rangeImageChanged();
}

void AppController::applySessionToViewer() {
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
  if (!item) {
    return;
  }
  item->setPoseMatrix(isometryToPoseMatrix(session_.T_camera_lidar));
}

void AppController::emitSessionUiState() {
  emit scanTimeChanged();
  emit sessionPathsChanged();
  emit pcdCatalogChanged();
  emit sessionLoaded();
  applySessionToViewer();
  if (!session_.pcd_paths.empty() && !fusion_blocked_) {
    refreshRangeImage();
    if (fused_cloud_.points.empty()) {
      uploadPreviewScanToViewer();
    }
  }
}

bool AppController::loadSession(const QString& directory) {
  const QString dir = normalizedDirectoryPath(directory);
  if (dir.isEmpty()) {
    setStatus(QStringLiteral("Session directory is empty"));
    return false;
  }
  const QString path = dir + QStringLiteral("/session.json");
  if (!clc::load_session(path.toStdString(), session_)) {
    setStatus(QStringLiteral("Failed to load session from %1").arg(path));
    return false;
  }
  session_.fusion_params.scan_time = session_.scan_time;
  setSessionDirectory(dir);
  if (!session_.cloud_directory.empty()) {
    refreshPcdCatalog();
  } else if (!session_.pcd_paths.empty()) {
    emit pcdCatalogChanged();
  }
  if (!session_.intrinsics_path.empty()) {
    std::string error;
    clc::load_camera_intrinsics_file(session_.intrinsics_path, session_.camera, error);
  }
  mask_ops_.clear();
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  global_mask_pixels_.clear();
  if (!session_.global_mask.path.empty() && session_.global_mask.width > 0 && session_.global_mask.height > 0) {
    std::string error;
    clc::load_mask_png(session_.global_mask.path, session_.global_mask.width, session_.global_mask.height,
                       global_mask_pixels_, error);
  }
  setStatus(QStringLiteral("Session loaded"));
  emit maskChanged();
  emitSessionUiState();
  return true;
}

bool AppController::saveSession(const QString& directory) {
  const QString dir = normalizedDirectoryPath(directory.isEmpty() ? session_directory_ : directory);
  if (dir.isEmpty()) {
    setStatus(QStringLiteral("Session directory is empty"));
    return false;
  }
  setSessionDirectory(dir);
  session_.fusion_params.scan_time = session_.scan_time;
  if (!global_mask_pixels_.empty() && session_.reference_scan_width > 0 && session_.reference_scan_height > 0 &&
      global_mask_pixels_.size() == static_cast<std::size_t>(session_.reference_scan_width * session_.reference_scan_height)) {
    const QString mask_path = dir + QStringLiteral("/global_mask.png");
    clc::save_mask_png(mask_path.toStdString(), session_.reference_scan_width, session_.reference_scan_height,
                       global_mask_pixels_);
    session_.global_mask.path = mask_path.toStdString();
    session_.global_mask.width = session_.reference_scan_width;
    session_.global_mask.height = session_.reference_scan_height;
  }
  const QString path = dir + QStringLiteral("/session.json");
  if (!clc::save_session(path.toStdString(), session_)) {
    setStatus(QStringLiteral("Failed to save session to %1").arg(path));
    return false;
  }
  setStatus(QStringLiteral("Session saved"));
  return true;
}

QString AppController::browseSessionDirectory() {
  const QString dir = QFileDialog::getExistingDirectory(nullptr, QStringLiteral("Session directory"), session_directory_);
  if (!dir.isEmpty()) {
    setSessionDirectory(dir);
  }
  return session_directory_;
}

QString AppController::browsePcdDirectory() {
  const QString dir = "/Users/rasmus/Downloads/pbb-muc-255-1-fasssauna-high-freq-approach/sequence1";//FileDialog::getExistingDirectory(nullptr, QStringLiteral("PCD directory"), pcdDirectory());
  if (!dir.isEmpty()) {
    setPcdDirectory(dir);
  }
  return pcdDirectory();
}

QString AppController::browseImageFile() {
  const QString file = QFileDialog::getOpenFileName(
    nullptr,
    QStringLiteral("Camera image"),
    "/Users/rasmus/Downloads/pbb-muc-255-1-fasssauna-high-freq-approach/sequence1/2026-10-01T18_42_13_624Z.png",//imagePath(),
    QStringLiteral("Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)"));
  if (!file.isEmpty()) {
    setImagePath(file);
  }
  return imagePath();
}

QString AppController::browseIntrinsicsFile() {
  const QString file = QFileDialog::getOpenFileName(
    nullptr,
    QStringLiteral("Camera intrinsics"),
    "/Users/rasmus/Documents/Projects/autocrane-core/config/pbb/muc/shared/calibration/cam_bridge_narrow_rotated_upright.yml",//intrinsicsPath(),
    QStringLiteral("YAML (*.yaml *.yml);;OpenCV (*.xml)"));
  if (!file.isEmpty()) {
    loadIntrinsicsFile(file);
  }
  return intrinsicsPath();
}

bool AppController::loadIntrinsicsFile(const QString& path) {
  const QString normalized = QFileInfo(path).absoluteFilePath();
  if (normalized.isEmpty()) {
    setStatus(QStringLiteral("Intrinsics path is empty"));
    return false;
  }
  std::string error;
  if (!clc::load_camera_intrinsics_file(normalized.toStdString(), session_.camera, error)) {
    setStatus(QString::fromStdString(error));
    return false;
  }
  session_.intrinsics_path = normalized.toStdString();
  setStatus(QStringLiteral("Intrinsics loaded (%1)").arg(QString::fromStdString(session_.camera.model)));
  emit sessionPathsChanged();
  return true;
}

void AppController::setViewer(QObject* viewer) {
  if (viewer_ == viewer) {
    return;
  }
  if (viewer_) {
    disconnect(viewer_, nullptr, this, nullptr);
  }
  viewer_ = viewer;
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
  if (!item) {
    setStatus(QStringLiteral("3D viewer type mismatch; point cloud preview disabled"));
    return;
  }
  connect(item, &psiviz::PointCloudItem::pointPicked, this, &AppController::onPointPicked);
  connect(item, &psiviz::PointCloudItem::poseChanged, this, &AppController::onViewerPoseChanged);
  connect(item, &psiviz::PointCloudItem::blendWeightChanged, this, &AppController::refreshViewerColors);
  if (!fused_cloud_.points.empty()) {
    uploadFusedCloudToViewer();
  } else if (preview_upload_pending_ || !session_.pcd_paths.empty()) {
    uploadPreviewScanToViewer();
  }
}

void AppController::uploadLidarScanToViewer(const clc::PointXYZITCloud& cloud) {
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
  if (!item || cloud.scan.points.empty()) {
    return;
  }
  const int n = static_cast<int>(cloud.scan.points.size());
  QVector<float> positions;
  QVector<float> scalars;
  QVector<float> rgb;
  positions.reserve(n * 3);
  scalars.reserve(n);
  rgb.reserve(n * 3);
  float min_v = std::numeric_limits<float>::infinity();
  float max_v = -std::numeric_limits<float>::infinity();
  for (int i = 0; i < n; ++i) {
    const auto& p = cloud.scan.points[i];
    if (!cloud.scan.keep.empty() && i < static_cast<int>(cloud.scan.keep.size()) && !cloud.scan.keep[i]) {
      continue;
    }
    const float x = static_cast<float>(p.x());
    const float y = static_cast<float>(p.y());
    const float z = static_cast<float>(p.z());
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    positions << x << y << z;
    float scalar = 0.f;
    switch (viewer_scalar_channel_) {
      case 1:
        scalar = x;
        break;
      case 2:
        scalar = y;
        break;
      case 3:
        scalar = z;
        break;
      default:
        scalar = i < static_cast<int>(cloud.scan.intensities.size()) ? static_cast<float>(cloud.scan.intensities[i]) : 0.f;
        break;
    }
    scalars << scalar;
    rgb << 0.5f << 0.5f << 0.5f;
    min_v = std::min(min_v, scalar);
    max_v = std::max(max_v, scalar);
  }
  if (positions.isEmpty()) {
    setPointCount(0);
    setStatus(QStringLiteral("No finite points to show in 3D preview (check mask / PCD)"));
    return;
  }
  if (std::isfinite(min_v) && std::isfinite(max_v) && max_v > min_v) {
    viewer_colormap_min_ = min_v;
    viewer_colormap_max_ = max_v;
  } else if (viewer_scalar_channel_ == 0) {
    viewer_colormap_min_ = 0.0;
    viewer_colormap_max_ = 255.0;
  } else {
    viewer_colormap_min_ = 0.0;
    viewer_colormap_max_ = 1.0;
  }
  emit viewerColormapChanged();
  item->setScalarMin(viewer_colormap_min_);
  item->setScalarMax(viewer_colormap_max_);
  viewer_positions_ = positions;
  viewer_scalars_ = scalars;
  viewer_rgb_ = rgb;
  syncColorModelFromSession();
  refreshViewerRgb(static_cast<float>(item->blendWeight()));
  const int shown = static_cast<int>(viewer_positions_.size() / 3);
  setPointCount(shown);
  setStatus(QStringLiteral("3D preview: %1 points (intensity)").arg(shown));
}

void AppController::uploadPreviewScanToViewer() {
  if (fusion_blocked_ || session_.pcd_paths.empty()) {
    preview_upload_pending_ = false;
    return;
  }
  if (!viewer_) {
    preview_upload_pending_ = true;
    return;
  }
  preview_upload_pending_ = false;
  clc::PointXYZITCloud cloud;
  std::string err;
  if (!loadFirstReadableScan(cloud, err)) {
    setStatus(QString::fromStdString(err.empty() ? "failed to load preview scan" : err));
    setPointCount(0);
    return;
  }
  const std::size_t expected_mask = static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
  if (!global_mask_pixels_.empty() && global_mask_pixels_.size() == expected_mask) {
    auto scan = to_fusion_scan(cloud);
    clc::apply_mask_to_scan(scan, global_mask_pixels_);
    cloud.scan = std::move(scan);
  }
  uploadLidarScanToViewer(cloud);
}

void AppController::uploadFusedCloudToViewer() {
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
  if (!item || fused_cloud_.points.empty()) {
    return;
  }
  QVector<float> positions;
  QVector<float> scalars;
  QVector<float> rgb;
  const int n = static_cast<int>(fused_cloud_.points.size());
  positions.reserve(n * 3);
  scalars.reserve(n);
  rgb.reserve(n * 3);
  for (int i = 0; i < n; ++i) {
    const auto& p = fused_cloud_.points[i];
    positions << p.x() << p.y() << p.z();
    float scalar = 0.f;
    switch (viewer_scalar_channel_) {
      case 1:
        scalar = p.x();
        break;
      case 2:
        scalar = p.y();
        break;
      case 3:
        scalar = p.z();
        break;
      default:
        scalar = i < static_cast<int>(fused_cloud_.intensities.size()) ? static_cast<float>(fused_cloud_.intensities[i]) : 0.f;
        break;
    }
    scalars << scalar;
    rgb << 0.5f << 0.5f << 0.5f;
  }
  item->setScalarMin(viewer_colormap_min_);
  item->setScalarMax(viewer_colormap_max_);
  viewer_positions_ = positions;
  viewer_scalars_ = scalars;
  viewer_rgb_ = rgb;
  syncColorModelFromSession();
  refreshViewerRgb(static_cast<float>(item->blendWeight()));
}

void AppController::onPointPicked(double x, double y, double z) {
  setStatus(QStringLiteral("Picked (%1, %2, %3)").arg(x).arg(y).arg(z));
}

void AppController::onViewerPoseChanged() {
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
  if (!item) {
    return;
  }
  const QVariantList m = item->poseMatrix();
  if (m.size() != 16) {
    return;
  }
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  int k = 0;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      T(r, c) = m[k++].toDouble();
    }
  }
  session_.T_camera_lidar_initial = Eigen::Isometry3d(T);
  session_.T_camera_lidar = session_.T_camera_lidar_initial;
  refreshViewerColors();
}

void AppController::runFusion() {
  if (worker_running_) {
    return;
  }
  if (fusion_blocked_) {
    setStatus(QStringLiteral("Fusion blocked: fix PCD scan geometry"));
    return;
  }
  if (session_.pcd_paths.empty()) {
    setStatus(QStringLiteral("No PCD files loaded"));
    return;
  }
  worker_running_ = true;
  setStatus(QStringLiteral("Fusion running..."));
  if (auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_)) {
    item->setInteractionEnabled(false);
  }
  worker_ = std::make_unique<std::thread>([this]() {
    std::vector<glim_cloud_fusion::LidarScan> scans;
    for (const auto& path : session_.pcd_paths) {
      clc::PointXYZITCloud cloud;
      std::string err;
      if (!clc::load_pcd_xyzit(path, cloud, err)) {
        QMetaObject::invokeMethod(this, [this, err]() {
          worker_running_ = false;
          if (auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_)) {
            item->setInteractionEnabled(true);
          }
          setStatus(QString::fromStdString(err));
          emit fusionFinished(false, QString::fromStdString(err));
        }, Qt::QueuedConnection);
        return;
      }
      if (session_.reference_scan_width > 0 &&
          (cloud.width != session_.reference_scan_width || cloud.height != session_.reference_scan_height)) {
        const std::string dim_err = "scan dimension mismatch in " + path;
        QMetaObject::invokeMethod(this, [this, dim_err]() {
          worker_running_ = false;
          if (auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_)) {
            item->setInteractionEnabled(true);
          }
          setStatus(QString::fromStdString(dim_err));
          emit fusionFinished(false, QString::fromStdString(dim_err));
        }, Qt::QueuedConnection);
        return;
      }
      auto fused_scan = to_fusion_scan(cloud);
      if (!global_mask_pixels_.empty()) {
        clc::apply_mask_to_scan(fused_scan, global_mask_pixels_);
      }
      scans.push_back(std::move(fused_scan));
    }

    auto params = session_.fusion_params;
    params.scan_time = session_.scan_time;
    const auto result = clc::run_fusion(scans, params, [this](const glim_cloud_fusion::FusionProgress& p) {
      QMetaObject::invokeMethod(this, [this, p]() { setStatus(QString::fromStdString(p.stage)); }, Qt::QueuedConnection);
    });

    QMetaObject::invokeMethod(this, [this, result]() {
      worker_running_ = false;
      if (auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_)) {
        item->setInteractionEnabled(true);
      }
      if (result.error != glim_cloud_fusion::FusionError::Ok) {
        setStatus(QString::fromStdString(result.message));
        emit fusionFinished(false, QString::fromStdString(result.message));
        return;
      }
      fused_cloud_ = result.cloud;
      setPointCount(static_cast<int>(result.cloud.points.size()));
      refreshViewerColormapBounds();
      emit viewerColormapChanged();
      setStatus(QStringLiteral("Fusion complete"));
      uploadFusedCloudToViewer();
      emit fusionFinished(true, QStringLiteral("Fusion complete"));
    }, Qt::QueuedConnection);
  });
}

void AppController::runAlignment() {
  if (worker_running_) {
    return;
  }
  if (session_.image_path.empty() || fused_cloud_.points.empty()) {
    setStatus(QStringLiteral("Need image and fused cloud before alignment"));
    return;
  }
  worker_running_ = true;
  setStatus(QStringLiteral("Alignment running..."));
  if (auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_)) {
    item->setInteractionEnabled(false);
    item->setGizmoEnabled(false);
  }
  worker_ = std::make_unique<std::thread>([this]() {
    clc::AlignmentInput input;
    std::string build_err;
    if (!buildAlignmentInput(input, build_err)) {
      QMetaObject::invokeMethod(this, [this, build_err]() {
        worker_running_ = false;
        if (auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_)) {
          item->setInteractionEnabled(true);
          item->setGizmoEnabled(true);
        }
        setStatus(QString::fromStdString(build_err));
        emit alignmentFinished(false, QString::fromStdString(build_err));
      }, Qt::QueuedConnection);
      return;
    }

    const auto result = clc::run_alignment(input, [this](const vlcal::CalibrationProgress& p) {
      QMetaObject::invokeMethod(
        this,
        [this, p]() {
          session_.T_camera_lidar = p.T_camera_lidar;
          applySessionToViewer();
          refreshViewerColors();
          setStatus(QStringLiteral("cost=%1").arg(p.cost));
        },
        Qt::QueuedConnection);
    });

    QMetaObject::invokeMethod(this, [this, result]() {
      worker_running_ = false;
      if (auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_)) {
        item->setInteractionEnabled(true);
        item->setGizmoEnabled(true);
      }
      if (result.error != vlcal::CalibrationError::Ok) {
        setStatus(QString::fromStdString(result.message));
        emit alignmentFinished(false, QString::fromStdString(result.message));
        return;
      }
      session_.T_camera_lidar = result.T_camera_lidar;
      applySessionToViewer();
      refreshViewerColors();
      setStatus(QStringLiteral("Alignment complete"));
      emit alignmentFinished(true, QStringLiteral("Alignment complete"));
    }, Qt::QueuedConnection);
  });
}
