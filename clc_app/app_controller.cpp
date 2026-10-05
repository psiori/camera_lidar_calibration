#include "app_controller.hpp"

#include <QMetaObject>

#include <opencv2/imgcodecs.hpp>

#include <camera/create_camera.hpp>
#include <clc_core/calibration_service.hpp>
#include <clc_core/fusion_service.hpp>
#include <clc_core/pcd_io.hpp>

namespace {

glim_cloud_fusion::LidarScan to_fusion_scan(const clc::PointXYZITCloud& cloud) {
  glim_cloud_fusion::LidarScan scan;
  scan.width = cloud.width;
  scan.height = cloud.height;
  scan.points = cloud.scan.points;
  scan.times = cloud.scan.times;
  scan.intensities = cloud.scan.intensities;
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

bool AppController::loadSession(const QString& directory) {
  const QString path = directory + "/session.json";
  if (!clc::load_session(path.toStdString(), session_)) {
    setStatus(QStringLiteral("Failed to load session"));
    return false;
  }
  setStatus(QStringLiteral("Session loaded"));
  emit scanTimeChanged();
  return true;
}

bool AppController::saveSession(const QString& directory) {
  const QString path = directory + "/session.json";
  session_.fusion_params.scan_time = session_.scan_time;
  if (!clc::save_session(path.toStdString(), session_)) {
    setStatus(QStringLiteral("Failed to save session"));
    return false;
  }
  setStatus(QStringLiteral("Session saved"));
  return true;
}

void AppController::setImagePath(const QString& path) {
  session_.image_path = path.toStdString();
}

void AppController::setPcdDirectory(const QString& path) {
  session_.cloud_directory = path.toStdString();
  session_.pcd_paths = clc::list_pcd_files(path.toStdString());
  setStatus(QStringLiteral("Found %1 PCD files").arg(static_cast<int>(session_.pcd_paths.size())));
}

void AppController::runFusion() {
  if (worker_running_) {
    return;
  }
  if (session_.pcd_paths.empty()) {
    setStatus(QStringLiteral("No PCD files loaded"));
    return;
  }
  worker_running_ = true;
  setStatus(QStringLiteral("Fusion running..."));
  worker_ = std::make_unique<std::thread>([this]() {
    std::vector<glim_cloud_fusion::LidarScan> scans;
    for (const auto& path : session_.pcd_paths) {
      clc::PointXYZITCloud cloud;
      std::string err;
      if (!clc::load_pcd_xyzit(path, cloud, err)) {
        QMetaObject::invokeMethod(this, [this, err]() {
          worker_running_ = false;
          setStatus(QString::fromStdString(err));
          emit fusionFinished(false, QString::fromStdString(err));
        }, Qt::QueuedConnection);
        return;
      }
      scans.push_back(to_fusion_scan(cloud));
    }

    auto params = session_.fusion_params;
    params.scan_time = session_.scan_time;
    const auto result = clc::run_fusion(scans, params, [this](const glim_cloud_fusion::FusionProgress& p) {
      QMetaObject::invokeMethod(this, [this, p]() { setStatus(QString::fromStdString(p.stage)); }, Qt::QueuedConnection);
    });

    QMetaObject::invokeMethod(this, [this, result]() {
      worker_running_ = false;
      if (result.error != glim_cloud_fusion::FusionError::Ok) {
        setStatus(QString::fromStdString(result.message));
        emit fusionFinished(false, QString::fromStdString(result.message));
        return;
      }
      fused_cloud_ = result.cloud;
      setPointCount(static_cast<int>(result.cloud.points.size()));
      setStatus(QStringLiteral("Fusion complete"));
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
  worker_ = std::make_unique<std::thread>([this]() {
    cv::Mat image = cv::imread(session_.image_path, cv::IMREAD_UNCHANGED);
    if (image.empty()) {
      QMetaObject::invokeMethod(this, [this]() {
        worker_running_ = false;
        setStatus(QStringLiteral("Failed to load image"));
        emit alignmentFinished(false, QStringLiteral("Failed to load image"));
      }, Qt::QueuedConnection);
      return;
    }

    clc::AlignmentInput input;
    input.camera = camera::create_camera(session_.camera.model, session_.camera.intrinsics, session_.camera.distortion_coeffs);
    input.images = {image};
    input.clouds = {to_frame_cpu(fused_cloud_)};
    input.init_T_camera_lidar = session_.T_camera_lidar_initial;
    input.params = session_.calibration_params;

    const auto result = clc::run_alignment(input, [this](const vlcal::CalibrationProgress& p) {
      QMetaObject::invokeMethod(
        this,
        [this, p]() { setStatus(QStringLiteral("cost=%1").arg(p.cost)); },
        Qt::QueuedConnection);
    });

    QMetaObject::invokeMethod(this, [this, result]() {
      worker_running_ = false;
      if (result.error != vlcal::CalibrationError::Ok) {
        setStatus(QString::fromStdString(result.message));
        emit alignmentFinished(false, QString::fromStdString(result.message));
        return;
      }
      session_.T_camera_lidar = result.T_camera_lidar;
      setStatus(QStringLiteral("Alignment complete"));
      emit alignmentFinished(true, QStringLiteral("Alignment complete"));
    }, Qt::QueuedConnection);
  });
}
