#include "app_logging.hpp"

#include <cstdio>

#include <QDateTime>
#include <QString>

Q_LOGGING_CATEGORY(lcClcDataset, "clc.dataset")
Q_LOGGING_CATEGORY(lcClcLabeling, "clc.labeling")
Q_LOGGING_CATEGORY(lcClcOptimization, "clc.optimization")

namespace {

const char* messageTypeLabel(QtMsgType type) {
  switch (type) {
    case QtDebugMsg:
      return "debug";
    case QtInfoMsg:
      return "info";
    case QtWarningMsg:
      return "warning";
    case QtCriticalMsg:
      return "critical";
    case QtFatalMsg:
      return "fatal";
  }
  return "unknown";
}

void clcStdoutMessageHandler(QtMsgType type, const QMessageLogContext& context,
                             const QString& msg) {
  const QString timestamp =
      QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd hh:mm:ss.zzz"));
  const char* category =
      (context.category != nullptr && context.category[0] != '\0') ? context.category
                                                                   : "default";
  const QByteArray line =
      QStringLiteral("[%1] [%2] %3: %4")
          .arg(timestamp, QString::fromUtf8(category),
               QString::fromUtf8(messageTypeLabel(type)), msg)
          .toUtf8();
  std::fputs(line.constData(), stdout);
  std::fputc('\n', stdout);
  std::fflush(stdout);
  if (type == QtFatalMsg) {
    std::abort();
  }
}

}  // namespace

void install_clc_stdout_logging() {
  static bool installed = false;
  if (installed) {
    return;
  }
  installed = true;
  qInstallMessageHandler(clcStdoutMessageHandler);
}
