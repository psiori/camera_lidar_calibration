#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include <QObject>
#include <QString>

#include <glim_cloud_fusion/fusion.hpp>

#include <clc_core/session.hpp>

class AppController : public QObject {
  Q_OBJECT
  Q_PROPERTY(QString statusMessage READ statusMessage NOTIFY statusMessageChanged)
  Q_PROPERTY(int pointCount READ pointCount NOTIFY pointCountChanged)
  Q_PROPERTY(double scanDuration READ scanDuration WRITE setScanDuration NOTIFY scanTimeChanged)
  Q_PROPERTY(int timeOrigin READ timeOrigin WRITE setTimeOrigin NOTIFY scanTimeChanged)
  Q_PROPERTY(int timeUnit READ timeUnit WRITE setTimeUnit NOTIFY scanTimeChanged)

public:
  explicit AppController(QObject* parent = nullptr);
  ~AppController() override;

  QString statusMessage() const { return status_message_; }
  int pointCount() const { return point_count_; }
  double scanDuration() const;
  int timeOrigin() const;
  int timeUnit() const;

  void setScanDuration(double seconds);
  void setTimeOrigin(int origin);
  void setTimeUnit(int unit);

  Q_INVOKABLE bool loadSession(const QString& directory);
  Q_INVOKABLE bool saveSession(const QString& directory);
  Q_INVOKABLE void runFusion();
  Q_INVOKABLE void runAlignment();
  Q_INVOKABLE void setImagePath(const QString& path);
  Q_INVOKABLE void setPcdDirectory(const QString& path);

signals:
  void statusMessageChanged();
  void pointCountChanged();
  void scanTimeChanged();
  void fusionFinished(bool ok, const QString& message);
  void alignmentFinished(bool ok, const QString& message);

private:
  void setStatus(const QString& message);
  void setPointCount(int count);

  clc::SessionData session_;
  glim_cloud_fusion::DensePointCloud fused_cloud_;
  QString status_message_ = QStringLiteral("Ready");
  int point_count_ = 0;
  std::atomic<bool> worker_running_{false};
  std::unique_ptr<std::thread> worker_;
};
