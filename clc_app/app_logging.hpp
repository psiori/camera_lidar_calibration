#pragma once

#include <QLoggingCategory>

/// Logging for calibration dataset load/catalog changes.
Q_DECLARE_LOGGING_CATEGORY(lcClcDataset)
/// Logging for manual 2D/3D correspondence (labeling) actions.
Q_DECLARE_LOGGING_CATEGORY(lcClcLabeling)
/// Logging for NID / alignment input assembly and runs.
Q_DECLARE_LOGGING_CATEGORY(lcClcOptimization)

/// Route Qt log messages to stdout (default handler uses stderr on many platforms).
void install_clc_stdout_logging();
