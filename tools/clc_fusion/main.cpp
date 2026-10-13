#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#include <glim/util/config.hpp>

#include <clc_core/fusion_service.hpp>
#include <clc_core/pcd_io.hpp>
#include <clc_core/range_image.hpp>
#include <clc_core/session.hpp>

namespace {

using Clock = std::chrono::steady_clock;

std::string format_duration(double seconds) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << seconds << "s";
  return oss.str();
}

void print_usage(const char* prog) {
  std::cerr
    << "Usage: " << prog << " <pcd_directory> [options]\n"
    << "  Fuses sorted PCD scans with glim_cloud_fusion (default: CT integrator), same core as clc_app.\n\n"
    << "I/O:\n"
    << "  --output PATH               Write fused cloud as binary PCD (XYZ + intensity)\n"
    << "  --max-scans N               Only process first N files after timestamp sort\n"
    << "  --glim-config DIR           GLIM config directory (silences missing /config.json)\n"
    << "  --params-json PATH          Load fusion_params from session.json or fusion_params JSON\n"
    << "  --print-params              Print effective FusionParams and exit (with --params-json only)\n\n"
    << "Output merge:\n"
    << "  --voxel-resolution M        Accumulated output voxel size in meters (default: 0.01)\n\n"
    << "Scan time / deskew:\n"
    << "  --time-unit seconds|nanoseconds (default: nanoseconds)\n"
    << "  --time-origin scan_start|first_point (default: scan_start)\n"
    << "  --scan-duration S           Sweep duration in seconds (inferred from first scan if omitted)\n\n"
    << "Preprocess:\n"
    << "  --k-neighbors N             kNN count for per-point covariances (default: 20)\n"
    << "  --threads N                 OpenMP worker threads (default: 16 for integrator)\n\n"
    << "CT-GICP SLAM (--fusion-backend slam):\n"
    << "  --max-correspondence-dist M Max correspondence distance in meters\n"
    << "  --ivox-resolution M         Scan-to-model iVox leaf size in meters\n"
    << "  --ivox-min-points-dist M    Min distance between points in an iVox cell\n"
    << "  --ivox-lru-thresh N         iVox LRU horizon (scans)\n"
    << "  --location-consistency S    Prior weight tying scan start to previous scan end\n"
    << "  --constant-velocity S       Weight on zero motion between scan start/end poses\n"
    << "  --lm-max-iterations N       Levenberg-Marquardt iterations per scan\n"
    << "  --smoother-lag S            Fixed-lag smoother window in seconds\n"
    << "  --use-isam2-dogleg          Use ISAM2 dogleg optimizer\n"
    << "  --isam2-relinearize-skip N  ISAM2 relinearize skip\n"
    << "  --isam2-relinearize-thresh T ISAM2 relinearize threshold\n"
    << "  --compute-covs              Compute marginal covariances (debug)\n"
    << "  --enable-smoother           Enable iSAM2 fixed-lag smoother (default: off)\n"
    << "CT integrator (default --fusion-backend integrator):\n"
    << "  --fusion-backend integrator|slam   Fusion method (default: integrator)\n"
    << "  --max-correspondence-dist M Max correspondence (m, default: 1.0)\n"
    << "  --target-num-points N       Per-scan point cap after random-grid subsample (default: 10000)\n"
    << "  --min-distance M            Min range in scanner frame before transform (m, default: 1.0; not iVox spacing)\n"
    << "  --integrator-ivox-resolution M  Scan-to-model iVox leaf size (m, default: 1.0)\n"
    << "  --integrator-ivox-min-points-dist M  Min spacing in integrator iVox cells (m, default: 0.05)\n"
    << "  --integrator-debug          Per-scan deskew/LM/pose stats (stderr)\n"
    << "  -h, --help                  Show this help\n";
}

std::vector<std::string> sort_pcd_paths_by_timestamp(const std::vector<std::string>& paths) {
  std::vector<std::pair<double, std::string>> keyed;
  keyed.reserve(paths.size());
  for (const auto& path : paths) {
    double stamp = 0.0;
    const auto name = std::filesystem::path(path).filename().string();
    if (!clc::parse_pcd_filename_stamp(name, stamp)) {
      throw std::runtime_error("cannot parse timestamp from " + name);
    }
    keyed.emplace_back(stamp, path);
  }
  std::sort(keyed.begin(), keyed.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  std::vector<std::string> sorted;
  sorted.reserve(keyed.size());
  for (const auto& [_, path] : keyed) {
    sorted.push_back(path);
  }
  return sorted;
}

struct Options {
  std::string directory;
  std::optional<int> threads;
  std::optional<double> max_correspondence_distance;
  std::optional<double> voxel_resolution;
  std::optional<double> scan_duration;
  std::optional<int> max_scans;
  std::optional<int> k_neighbors;
  std::optional<double> ivox_resolution;
  std::optional<double> ivox_min_points_dist;
  std::optional<int> ivox_lru_thresh;
  std::optional<double> location_consistency_inf_scale;
  std::optional<double> constant_velocity_inf_scale;
  std::optional<int> lm_max_iterations;
  std::optional<double> smoother_lag;
  std::optional<bool> use_isam2_dogleg;
  std::optional<double> isam2_relinearize_skip;
  std::optional<double> isam2_relinearize_thresh;
  std::optional<bool> compute_covs;
  std::optional<bool> enable_fixed_lag_smoother;
  std::optional<std::string> fusion_backend;
  std::optional<int> target_num_points;
  std::optional<double> min_distance;
  std::optional<double> integrator_ivox_resolution;
  std::optional<double> integrator_ivox_min_points_dist;
  std::string output_path = "out.pcd";
  std::string glim_config_dir;
  std::string params_json_path;
  bool print_params_only = false;
  bool integrator_debug = false;
  glim_cloud_fusion::TimeUnit time_unit = glim_cloud_fusion::TimeUnit::Nanoseconds;
  glim_cloud_fusion::TimeOrigin time_origin = glim_cloud_fusion::TimeOrigin::ScanStart;
};

void apply_cli_overrides(const Options& opt, glim_cloud_fusion::FusionParams& params) {
  params.scan_time.time_unit = opt.time_unit;
  params.scan_time.time_origin = opt.time_origin;
  if (opt.threads.has_value()) {
    params.num_threads = *opt.threads;
  }
  if (opt.max_correspondence_distance.has_value()) {
    params.max_correspondence_distance = *opt.max_correspondence_distance;
  }
  if (opt.voxel_resolution.has_value()) {
    params.voxel_resolution = *opt.voxel_resolution;
  }
  if (opt.scan_duration.has_value()) {
    params.scan_time.scan_duration = *opt.scan_duration;
  }
  if (opt.k_neighbors.has_value()) {
    params.k_neighbors = *opt.k_neighbors;
  }
  if (opt.ivox_resolution.has_value()) {
    params.ivox_resolution = *opt.ivox_resolution;
  }
  if (opt.ivox_min_points_dist.has_value()) {
    params.ivox_min_points_dist = *opt.ivox_min_points_dist;
  }
  if (opt.ivox_lru_thresh.has_value()) {
    params.ivox_lru_thresh = *opt.ivox_lru_thresh;
  }
  if (opt.location_consistency_inf_scale.has_value()) {
    params.location_consistency_inf_scale = *opt.location_consistency_inf_scale;
  }
  if (opt.constant_velocity_inf_scale.has_value()) {
    params.constant_velocity_inf_scale = *opt.constant_velocity_inf_scale;
  }
  if (opt.lm_max_iterations.has_value()) {
    params.lm_max_iterations = *opt.lm_max_iterations;
  }
  if (opt.smoother_lag.has_value()) {
    params.smoother_lag = *opt.smoother_lag;
  }
  if (opt.use_isam2_dogleg.has_value()) {
    params.use_isam2_dogleg = *opt.use_isam2_dogleg;
  }
  if (opt.isam2_relinearize_skip.has_value()) {
    params.isam2_relinearize_skip = *opt.isam2_relinearize_skip;
  }
  if (opt.isam2_relinearize_thresh.has_value()) {
    params.isam2_relinearize_thresh = *opt.isam2_relinearize_thresh;
  }
  if (opt.compute_covs.has_value()) {
    params.compute_covs = *opt.compute_covs;
  }
  if (opt.enable_fixed_lag_smoother.has_value()) {
    params.enable_fixed_lag_smoother = *opt.enable_fixed_lag_smoother;
  }
  if (opt.fusion_backend.has_value()) {
    if (*opt.fusion_backend == "slam" || *opt.fusion_backend == "glim_slam") {
      params.backend = glim_cloud_fusion::FusionBackend::GlimSlam;
    } else if (*opt.fusion_backend == "integrator" || *opt.fusion_backend == "ct_integrator") {
      params.backend = glim_cloud_fusion::FusionBackend::CtIntegrator;
    }
  }
  if (opt.target_num_points.has_value()) {
    params.target_num_points = *opt.target_num_points;
  }
  if (opt.min_distance.has_value()) {
    params.min_distance = *opt.min_distance;
  }
  if (opt.integrator_ivox_resolution.has_value()) {
    params.integrator_ivox_resolution = *opt.integrator_ivox_resolution;
  }
  if (opt.integrator_ivox_min_points_dist.has_value()) {
    params.integrator_ivox_min_points_dist = *opt.integrator_ivox_min_points_dist;
  }
  if (opt.integrator_debug) {
    params.integrator_debug = true;
  }
}

std::optional<Options> parse_args(int argc, char** argv) {
  if (argc < 2) {
    return std::nullopt;
  }
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      return std::nullopt;
    }
    if (arg == "--print-params") {
      opt.print_params_only = true;
      continue;
    }
    if (arg == "--use-isam2-dogleg") {
      opt.use_isam2_dogleg = true;
      continue;
    }
    if (arg == "--compute-covs") {
      opt.compute_covs = true;
      continue;
    }
    if (arg == "--enable-smoother") {
      opt.enable_fixed_lag_smoother = true;
      continue;
    }
    if (arg == "--fusion-backend" && i + 1 < argc) {
      opt.fusion_backend = argv[++i];
      continue;
    }
    if (arg == "--target-num-points" && i + 1 < argc) {
      opt.target_num_points = std::stoi(argv[++i]);
      continue;
    }
    if (arg == "--min-distance" && i + 1 < argc) {
      opt.min_distance = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--integrator-ivox-resolution" && i + 1 < argc) {
      opt.integrator_ivox_resolution = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--integrator-ivox-min-points-dist" && i + 1 < argc) {
      opt.integrator_ivox_min_points_dist = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--integrator-debug") {
      opt.integrator_debug = true;
      continue;
    }
    if (arg == "--threads" && i + 1 < argc) {
      opt.threads = std::stoi(argv[++i]);
      continue;
    }
    if (arg == "--max-correspondence-dist" && i + 1 < argc) {
      opt.max_correspondence_distance = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--voxel-resolution" && i + 1 < argc) {
      opt.voxel_resolution = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--scan-duration" && i + 1 < argc) {
      opt.scan_duration = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--max-scans" && i + 1 < argc) {
      opt.max_scans = std::stoi(argv[++i]);
      continue;
    }
    if (arg == "--k-neighbors" && i + 1 < argc) {
      opt.k_neighbors = std::stoi(argv[++i]);
      continue;
    }
    if (arg == "--ivox-resolution" && i + 1 < argc) {
      opt.ivox_resolution = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--ivox-min-points-dist" && i + 1 < argc) {
      opt.ivox_min_points_dist = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--ivox-lru-thresh" && i + 1 < argc) {
      opt.ivox_lru_thresh = std::stoi(argv[++i]);
      continue;
    }
    if (arg == "--location-consistency" && i + 1 < argc) {
      opt.location_consistency_inf_scale = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--constant-velocity" && i + 1 < argc) {
      opt.constant_velocity_inf_scale = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--lm-max-iterations" && i + 1 < argc) {
      opt.lm_max_iterations = std::stoi(argv[++i]);
      continue;
    }
    if (arg == "--smoother-lag" && i + 1 < argc) {
      opt.smoother_lag = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--isam2-relinearize-skip" && i + 1 < argc) {
      opt.isam2_relinearize_skip = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--isam2-relinearize-thresh" && i + 1 < argc) {
      opt.isam2_relinearize_thresh = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--output" && i + 1 < argc) {
      opt.output_path = argv[++i];
      continue;
    }
    if (arg == "--glim-config" && i + 1 < argc) {
      opt.glim_config_dir = argv[++i];
      continue;
    }
    if (arg == "--params-json" && i + 1 < argc) {
      opt.params_json_path = argv[++i];
      continue;
    }
    if (arg == "--time-unit" && i + 1 < argc) {
      const std::string v = argv[++i];
      if (v == "seconds") {
        opt.time_unit = glim_cloud_fusion::TimeUnit::Seconds;
      } else if (v == "nanoseconds") {
        opt.time_unit = glim_cloud_fusion::TimeUnit::Nanoseconds;
      } else {
        std::cerr << "Unknown --time-unit: " << v << "\n";
        return std::nullopt;
      }
      continue;
    }
    if (arg == "--time-origin" && i + 1 < argc) {
      const std::string v = argv[++i];
      if (v == "scan_start") {
        opt.time_origin = glim_cloud_fusion::TimeOrigin::ScanStart;
      } else if (v == "first_point") {
        opt.time_origin = glim_cloud_fusion::TimeOrigin::FirstPoint;
      } else {
        std::cerr << "Unknown --time-origin: " << v << "\n";
        return std::nullopt;
      }
      continue;
    }
    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "Unknown option: " << arg << "\n";
      return std::nullopt;
    }
    if (opt.directory.empty()) {
      opt.directory = arg;
    } else {
      std::cerr << "Unexpected extra argument: " << arg << "\n";
      return std::nullopt;
    }
  }
  if (opt.directory.empty() && !opt.print_params_only) {
    return std::nullopt;
  }
  return opt;
}

glim_cloud_fusion::FusionParams build_fusion_params(const Options& opt) {
  glim_cloud_fusion::FusionParams params;
  if (!opt.params_json_path.empty()) {
    std::string err;
    if (!clc::fusion_params_from_json_file(opt.params_json_path, params, err)) {
      throw std::runtime_error(err);
    }
  } else {
    clc::apply_default_fusion_params(params);
    clc::apply_default_scan_time_params(params.scan_time);
  }
  apply_cli_overrides(opt, params);
  params.scan_time.time_unit = opt.time_unit;
  params.scan_time.time_origin = opt.time_origin;
  if (opt.scan_duration.has_value()) {
    params.scan_time.scan_duration = *opt.scan_duration;
  }
  return params;
}

}  // namespace

int main(int argc, char** argv) {
  const auto parsed = parse_args(argc, argv);
  if (!parsed) {
    if (argc < 2) {
      print_usage(argv[0]);
    }
    return argc < 2 ? 1 : 2;
  }
  const Options& opt = *parsed;

  if (!opt.glim_config_dir.empty()) {
    std::filesystem::path config_path(opt.glim_config_dir);
    if (!config_path.is_absolute()) {
      config_path = std::filesystem::absolute(config_path);
    }
    glim::GlobalConfig::instance(config_path.string(), true);
    std::cout << "glim config: " << config_path << "\n";
  }

  try {
    auto params = build_fusion_params(opt);
#ifdef _OPENMP
    if (params.num_threads > 0) {
      omp_set_num_threads(params.num_threads);
    }
#endif
    std::vector<std::uint8_t> mask_pixels;
    if (!opt.params_json_path.empty()) {
      clc::SessionData session;
      if (clc::load_session(opt.params_json_path, session) && !session.global_mask.path.empty() &&
          session.global_mask.width > 0 && session.global_mask.height > 0) {
        std::string mask_err;
        clc::load_mask_png(
          session.global_mask.path,
          session.global_mask.width,
          session.global_mask.height,
          mask_pixels,
          mask_err);
        if (!mask_err.empty()) {
          std::cerr << "Warning: could not load global mask: " << mask_err << "\n";
          mask_pixels.clear();
        }
      }
    }
    if (opt.print_params_only) {
      std::cout << "FusionParams:\n";
      clc::print_fusion_params(std::cout, params);
      return 0;
    }

    auto paths = clc::list_pcd_files(opt.directory);
    paths = sort_pcd_paths_by_timestamp(paths);
    if (paths.empty()) {
      std::cerr << "No PCD files under: " << opt.directory << "\n";
      return 1;
    }
    if (opt.max_scans.has_value() && *opt.max_scans > 0 &&
        static_cast<size_t>(*opt.max_scans) < paths.size()) {
      paths.resize(static_cast<size_t>(*opt.max_scans));
    }

    auto params_mut = params;
    if (!clc::scan_time_duration_valid(params_mut.scan_time)) {
      std::cerr << "scan_duration is required (set --scan-duration or fusion_params.scan_time in JSON)\n";
      return 1;
    }
    std::cout << "PCD directory: " << opt.directory << "\n";
    std::cout << "Scans: " << paths.size() << "\nFusionParams:\n";
    clc::print_fusion_params(std::cout, params_mut);

    std::vector<glim_cloud_fusion::LidarScan> scans;
    scans.reserve(paths.size());

    const auto load_start = Clock::now();
    for (size_t i = 0; i < paths.size(); ++i) {
      clc::PointXYZITCloud cloud;
      std::string err;
      if (!clc::load_pcd_xyzit(paths[i], cloud, err)) {
        std::cerr << "Failed to load " << paths[i] << ": " << err << "\n";
        return 1;
      }
      scans.push_back(clc::to_lidar_scan_masked(cloud, mask_pixels, 0.0));

      if ((i + 1) % 10 == 0 || i + 1 == paths.size()) {
        const double elapsed =
          std::chrono::duration<double>(Clock::now() - load_start).count();
        std::cout << "[load] " << (i + 1) << "/" << paths.size() << " (" << format_duration(elapsed)
                  << ")\n";
      }
    }
    const double load_seconds = std::chrono::duration<double>(Clock::now() - load_start).count();
    std::cout << "Load complete: " << format_duration(load_seconds) << "\n";

    const auto fuse_start = Clock::now();
    auto last_progress = fuse_start;

    const auto result = clc::run_fusion(scans, params_mut, [&](const glim_cloud_fusion::FusionProgress& p) {
      const auto now = Clock::now();
      const double total_elapsed = std::chrono::duration<double>(now - fuse_start).count();
      const double since_last = std::chrono::duration<double>(now - last_progress).count();
      last_progress = now;

      const double rate = p.scan_index > 0 ? static_cast<double>(p.scan_index) / total_elapsed : 0.0;
      const double remaining =
        rate > 0.0 ? static_cast<double>(p.total_scans - p.scan_index) / rate : 0.0;

      std::cout << "[fusion] " << p.scan_index << "/" << p.total_scans << "  stage=" << p.stage
                << "  elapsed=" << format_duration(total_elapsed)
                << "  dt=" << format_duration(since_last);
      if (rate > 0.0) {
        std::cout << "  " << std::fixed << std::setprecision(2) << rate << " scans/s"
                  << "  eta=" << format_duration(remaining);
      }
      std::cout << "\n";
      if (p.integrator.has_value()) {
        glim_cloud_fusion::print_fusion_integrator_diagnostics(std::cout, p.scan_index, *p.integrator);
      }
    });

    const double fuse_seconds = std::chrono::duration<double>(Clock::now() - fuse_start).count();

    if (result.error != glim_cloud_fusion::FusionError::Ok) {
      std::cerr << "Fusion failed: " << result.message << "\n";
      return 1;
    }

    std::cout << "Fusion complete: " << format_duration(fuse_seconds) << "  points="
              << result.cloud.points.size() << "\n";
    std::cout << "Total (load+fuse): " << format_duration(load_seconds + fuse_seconds) << "\n";

    if (!opt.output_path.empty()) {
      std::string err;
      if (!clc::save_dense_pcd(opt.output_path, result.cloud, err)) {
        std::cerr << "Failed to write output: " << err << "\n";
        return 1;
      }
      std::cout << "Wrote " << opt.output_path << "\n";
    }

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
