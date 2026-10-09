# Building on macOS

**Hybrid dependency model:** Conan builds and caches the in-house libraries (`gtsam`, `gtsam_points`, `glim`, `vlcal_align`) plus smaller third-party packages (Eigen, Boost, fmt, spdlog, Ceres, PCL). **Homebrew** provides prebuilt OpenCV, Qt 6, and libomp for the app shell.

Packages are cached in `~/.calib-conan` (override with `CONAN_HOME`).

System requirements: Xcode Command Line Tools, `git`, Python 3 (for Conan), and Homebrew.

## Layout

Clone repositories as siblings under one directory (default `~/source_builds`):

```
~/source_builds/
  gtsam/
  gtsam_points/
  glim/
  direct_visual_lidar_calibration/
  camera_lidar_calibration/
```

All stack repos use branch **`feature/camera-lidar-calibration-libs`** on **psiori** (including `gtsam`, which adds Conan support on that branch).

### Clone (once)

```bash
CALIB_SRC=~/source_builds bash ~/source_builds/camera_lidar_calibration/scripts/clone-repos.sh
```

Or clone `camera_lidar_calibration` first, then run the script from that checkout. Set `GIT_URL_SCHEME=https` for HTTPS URLs.

## Step 0 — Homebrew system dependencies

```bash
bash ~/source_builds/camera_lidar_calibration/scripts/ensure-brew-deps.sh
```

Installs: `opencv@4`, `qt@6`, `libomp`, `ninja`. (Use the `opencv@4` formula, not Homebrew `opencv` 5.)

## Step 1 — Conan dependencies

Builds and caches in-house libraries from local checkouts:

```bash
CALIB_SRC=~/source_builds CONAN_HOME=~/.calib-conan \
  bash ~/source_builds/camera_lidar_calibration/scripts/install-deps.sh
```


Dependency order:

1. `gtsam`
2. `gtsam_points`
3. `glim` (includes `glim_cloud_fusion`)
4. `vlcal_align` (`direct_visual_lidar_calibration`)

The Conan profile (`conan/profiles/native-macos`) uses `compiler.version=13` for Conan Center binary compatibility. The numeric stack (`gtsam`, `gtsam_points`, `glim`, `vlcal_align`, and `clc_core`) is built from source with `-march=native` enabled by default (`CALIB_MARCH_NATIVE=True`). Set `CALIB_MARCH_NATIVE=False` to disable CPU-specific tuning.

Hybrid Homebrew integration (OpenMP, Qt hints for Conan; OpenCV 4 and Qt for the app) lives only in this repo under `conan/calib_deps.py` and `scripts/build-app.sh`. Conan library packages use Conan Center OpenCV 4.x; `build-app.sh` passes Homebrew `opencv@4` `OpenCV_DIR` / `Qt6_DIR` when configuring the app. If you upgrade `opencv@4`, rerun `install-deps.sh` to refresh `vlcal_align` in the Conan cache when needed.

## Step 2 — app shell

```bash
CALIB_SRC=~/source_builds CONAN_HOME=~/.calib-conan \
  bash ~/source_builds/camera_lidar_calibration/scripts/build-app.sh
```

Run:

```bash
~/source_builds/camera_lidar_calibration/build/clc_app/clc_app
```

## Repositories / branches

| Project | psiori repo | Branch |
|---------|-------------|--------|
| gtsam | psiori/gtsam | feature/camera-lidar-calibration-libs |
| gtsam_points | psiori/gtsam_points | feature/camera-lidar-calibration-libs |
| glim | psiori/glim | feature/camera-lidar-calibration-libs |
| direct_visual_lidar_calibration | psiori/direct_visual_lidar_calibration | feature/camera-lidar-calibration-libs |
| camera_lidar_calibration | psiori/camera_lidar_calibration | feature/camera-lidar-calibration-libs |
