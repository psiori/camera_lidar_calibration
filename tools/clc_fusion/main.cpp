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
    << "  Fuses sorted PCD scans with glim_cloud_fusion (CT-GICP), same core as clc_app.\n\n"
    << "I/O:\n"
    << "  --output PATH               Write fused cloud as binary PCD (XYZ + intensity)\n"
    << "  --max-scans N               Only process first N files after sorting\n"
    << "  --sort lex|timestamp        File order (default: lex)\n"
    << "  --glim-config DIR           GLIM config directory (silences missing /config.json)\n"
    << "  --params-json PATH          Load fusion_params from session.json or fusion_params JSON\n"
    << "  --print-params              Print effective FusionParams and exit (with --params-json only)\n\n"
    << "Output merge:\n"
    << "  --voxel-resolution M        Post-SLAM voxel average size in meters (default: app default)\n\n"
    << "Scan time / deskew:\n"
    << "  --time-unit seconds|nanoseconds (default: nanoseconds)\n"
    << "  --time-origin scan_start|first_point (default: scan_start)\n"
    << "  --scan-duration S           Sweep duration in seconds (inferred from first scan if omitted)\n\n"
    << "Preprocess:\n"
    << "  --k-neighbors N             kNN count for per-point covariances (default: app default)\n\n"
    << "CT-GICP odometry:\n"
    << "  --threads N                 Worker threads (default: app default / hardware count)\n"
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
    << "  -h, --help                  Show this help\n";
}

bool parse_autocrane_timestamp(const std::string& filename, double& stamp_seconds) {
  int y = 0;
  unsigned m = 0, d = 0, h = 0, mi = 0, s = 0, ms = 0;
  if (7 != std::sscanf(filename.c_str(), "%d-%u-%uT%u_%u_%u_%3uZ.pcd", &y, &m, &d, &h, &mi, &s, &ms)) {
    return false;
  }
  std::tm calendar_time{};
  calendar_time.tm_sec = s;
  calendar_time.tm_min = mi;
  calendar_time.tm_hour = h;
  calendar_time.tm_mday = d;
  calendar_time.tm_mon = m - 1;
  calendar_time.tm_year = y - 1900;
  calendar_time.tm_isdst = 0;
#if defined(_WIN32)
  const std::time_t utc_time = _mkgmtime(&calendar_time);
#else
  const std::time_t utc_time = timegm(&calendar_time);
#endif
  const auto ms_tp = std::chrono::milliseconds(ms) + std::chrono::system_clock::from_time_t(utc_time);
  stamp_seconds =
    std::chrono::duration_cast<std::chrono::duration<double>>(ms_tp.time_since_epoch()).count();
  return true;
}

std::vector<std::string> sort_pcd_paths(const std::vector<std::string>& paths, const std::string& mode) {
  if (mode != "timestamp") {
    return paths;
  }
  std::vector<std::pair<double, std::string>> keyed;
  keyed.reserve(paths.size());
  for (const auto& path : paths) {
    double stamp = 0.0;
    const auto name = std::filesystem::path(path).filename().string();
    if (!parse_autocrane_timestamp(name, stamp)) {
      std::cerr << "Warning: cannot parse timestamp from " << name << ", using lex order key\n";
      stamp = static_cast<double>(keyed.size());
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
  std::string output_path;
  std::string sort_mode = "lex";
  std::string glim_config_dir;
  std::string params_json_path;
  bool print_params_only = false;
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
    if (arg == "--sort" && i + 1 < argc) {
      opt.sort_mode = argv[++i];
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
    const auto params = build_fusion_params(opt);
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
    paths = sort_pcd_paths(paths, opt.sort_mode);
    if (paths.empty()) {
      std::cerr << "No PCD files under: " << opt.directory << "\n";
      return 1;
    }
    if (opt.max_scans.has_value() && *opt.max_scans > 0 &&
        static_cast<size_t>(*opt.max_scans) < paths.size()) {
      paths.resize(static_cast<size_t>(*opt.max_scans));
    }

    auto params_mut = params;
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
      if (scans.empty() && (!params_mut.scan_time.scan_duration.has_value() ||
                            *params_mut.scan_time.scan_duration <= 0.0)) {
        if (const auto duration = clc::infer_scan_duration_seconds(cloud.scan, params_mut.scan_time)) {
          params_mut.scan_time.scan_duration = *duration;
          std::cout << "Inferred scan_duration: " << *duration << " s\n";
        }
      }

      double stamp = static_cast<double>(i) * 0.1;
      if (opt.sort_mode == "timestamp") {
        parse_autocrane_timestamp(std::filesystem::path(paths[i]).filename().string(), stamp);
      }
      scans.push_back(clc::to_lidar_scan_masked(cloud, mask_pixels, stamp));

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
