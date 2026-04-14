#include "pure_lidar_gyro_odometer/gyro_odometer_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <tuple>
#include <utility>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pclomp/gicp_omp.h>
#include <small_gicp/pcl/pcl_registration.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Eigenvalues>

namespace pure_gyro_odometer
{

static uint8_t diagLevelFromString(const std::string & level)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (level == "OK" || level == "ok") return DiagnosticStatus::OK;
  if (level == "WARN" || level == "warn") return DiagnosticStatus::WARN;
  if (level == "ERROR" || level == "error") return DiagnosticStatus::ERROR;
  if (level == "STALE" || level == "stale") return DiagnosticStatus::STALE;
  return DiagnosticStatus::WARN;
}

namespace
{

std::string boolString(bool v)
{
  return v ? "true" : "false";
}

std::string formatDouble(double v, int precision = 6)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << v;
  return oss.str();
}

constexpr uint8_t kLidarPoseModeNormal = 0;
constexpr uint8_t kLidarPoseModeCritical = 1;
constexpr uint8_t kLidarPoseModeScanReject = 2;
constexpr int kCriticalKeepSpeedMismatchStreak = 3;

std::string lidarPoseModeToString(uint8_t mode)
{
  switch (mode) {
    case kLidarPoseModeCritical:
      return "CRITICAL";
    case kLidarPoseModeScanReject:
      return "SCAN_REJECT";
    case kLidarPoseModeNormal:
    default:
      return "NORMAL";
  }
}

double clamp01(double v)
{
  return std::max(0.0, std::min(1.0, v));
}

double wrapYaw(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3d = Eigen::Matrix3d;
using Vector3d = Eigen::Vector3d;

Matrix6d adjointRotationFirst(const Eigen::Matrix4d & T)
{
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T.block<3, 1>(0, 3);

  Eigen::Matrix3d tx = Eigen::Matrix3d::Zero();
  tx(0, 1) = -t.z();
  tx(0, 2) = t.y();
  tx(1, 0) = t.z();
  tx(1, 2) = -t.x();
  tx(2, 0) = -t.y();
  tx(2, 1) = t.x();

  Matrix6d A = Matrix6d::Zero();
  A.block<3, 3>(0, 0) = R;
  A.block<3, 3>(3, 0) = tx * R;
  A.block<3, 3>(3, 3) = R;
  return A;
}

Matrix3d symmetrize(const Matrix3d & M)
{
  return 0.5 * (M + M.transpose());
}

bool reduceHessianToSe2Info(const Matrix6d & H_base, Matrix3d & out_info)
{
  constexpr std::array<int, 3> q_idx{{3, 4, 2}};  // tx, ty, rz
  constexpr std::array<int, 3> n_idx{{0, 1, 5}};  // rx, ry, tz

  Matrix3d A = Matrix3d::Zero();
  Matrix3d B = Matrix3d::Zero();
  Matrix3d C = Matrix3d::Zero();

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      A(r, c) = H_base(q_idx[r], q_idx[c]);
      B(r, c) = H_base(q_idx[r], n_idx[c]);
      C(r, c) = H_base(n_idx[r], n_idx[c]);
    }
  }

  Matrix3d schur = symmetrize(A);
  Eigen::LDLT<Matrix3d> ldlt(symmetrize(C) + 1e-9 * Matrix3d::Identity());
  if (ldlt.info() == Eigen::Success) {
    schur = symmetrize(A - B * ldlt.solve(B.transpose()));
  }

  Eigen::SelfAdjointEigenSolver<Matrix3d> eig(schur);
  if (eig.info() != Eigen::Success) {
    return false;
  }

  const Eigen::Vector3d evals = eig.eigenvalues().cwiseMax(0.0);
  out_info = eig.eigenvectors() * evals.asDiagonal() * eig.eigenvectors().transpose();
  return out_info.allFinite();
}

std::tuple<Matrix3d, Eigen::Vector3d, int, double> computeWeakProjector(
  const Matrix3d & info_se2,
  double yaw_metric_m,
  double rel_thr,
  double abs_thr,
  double prior_blend)
{
  Matrix3d projector_param = Matrix3d::Zero();
  Eigen::Vector3d eigvals_norm = Eigen::Vector3d::Zero();
  int weak_count = 0;
  double score = 0.0;

  const double L = std::max(1e-3, yaw_metric_m);
  const Matrix3d D = (Matrix3d() << 1.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0,
                                    0.0, 0.0, L).finished();
  const Matrix3d Dinv = (Matrix3d() << 1.0, 0.0, 0.0,
                                       0.0, 1.0, 0.0,
                                       0.0, 0.0, 1.0 / L).finished();
  const Matrix3d info_norm = symmetrize(Dinv.transpose() * info_se2 * Dinv);

  Eigen::SelfAdjointEigenSolver<Matrix3d> eig(info_norm);
  if (eig.info() != Eigen::Success) {
    return {projector_param, eigvals_norm, weak_count, score};
  }

  eigvals_norm = eig.eigenvalues().cwiseMax(0.0);
  const double lambda_ref = std::max(1e-12, eigvals_norm(2));
  Matrix3d Pz = Matrix3d::Zero();
  double score_sum = 0.0;
  const double blend = std::max(0.0, std::min(1.0, prior_blend));

  for (int i = 0; i < 3; ++i) {
    const double lambda = eigvals_norm(i);
    const double rel = lambda / lambda_ref;

    double alpha_rel = 0.0;
    if (rel_thr > 0.0) {
      alpha_rel = std::max(0.0, std::min(1.0, (rel_thr - rel) / rel_thr));
    }

    double alpha_abs = 0.0;
    if (abs_thr > 0.0) {
      alpha_abs = std::max(0.0, std::min(1.0, (abs_thr - lambda) / abs_thr));
    }

    const double alpha_raw = std::max(alpha_rel, alpha_abs);
    if (alpha_raw > 1e-6) {
      ++weak_count;
      const Eigen::Vector3d v = eig.eigenvectors().col(i);
      Pz.noalias() += (blend * alpha_raw) * (v * v.transpose());
      score_sum += alpha_raw;
    }
  }

  projector_param = Dinv * Pz * D;
  score = std::max(0.0, std::min(1.0, score_sum / 3.0));
  return {projector_param, eigvals_norm, weak_count, score};
}

Eigen::Vector3d arcMotionPrior(double distance, double dyaw)
{
  if (!std::isfinite(distance) || !std::isfinite(dyaw)) {
    return Eigen::Vector3d::Zero();
  }

  if (std::fabs(dyaw) < 1e-6) {
    return Eigen::Vector3d(distance, 0.0, dyaw);
  }

  const double dx = distance * std::sin(dyaw) / dyaw;
  const double dy = distance * (1.0 - std::cos(dyaw)) / dyaw;
  return Eigen::Vector3d(dx, dy, dyaw);
}

Eigen::Matrix4f se2DeltaToScanGuess(
  const Eigen::Vector3d & delta_base,
  const Eigen::Matrix4f & T_base_scan,
  const Eigen::Matrix4f & T_scan_base)
{
  Eigen::Matrix4f T_prev_curr_base = Eigen::Matrix4f::Identity();
  T_prev_curr_base(0, 3) = static_cast<float>(delta_base.x());
  T_prev_curr_base(1, 3) = static_cast<float>(delta_base.y());
  T_prev_curr_base.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(static_cast<float>(delta_base.z()), Eigen::Vector3f::UnitZ()).toRotationMatrix();

  Eigen::Matrix4f T_prev_curr_scan = T_scan_base * T_prev_curr_base * T_base_scan;
  if (!T_prev_curr_scan.allFinite()) {
    return Eigen::Matrix4f::Identity();
  }
  return T_prev_curr_scan;
}

Eigen::Vector3d fuseWeakDirections(
  const Eigen::Vector3d & scan_delta,
  const Eigen::Vector3d & prior_delta,
  const Matrix3d & weak_projector_param)
{
  return scan_delta + weak_projector_param * (prior_delta - scan_delta);
}

double se2MetricNorm(const Eigen::Vector3d & delta, double yaw_metric_m)
{
  const double L = std::max(1e-3, yaw_metric_m);
  return std::sqrt(
    delta.x() * delta.x() + delta.y() * delta.y() +
    (L * delta.z()) * (L * delta.z()));
}

Eigen::Vector3d se2LocalDelta(
  double prev_x, double prev_y, double prev_yaw,
  double curr_x, double curr_y, double curr_yaw)
{
  const double dx_world = curr_x - prev_x;
  const double dy_world = curr_y - prev_y;
  const double c = std::cos(prev_yaw);
  const double s = std::sin(prev_yaw);
  const double dx_local = c * dx_world + s * dy_world;
  const double dy_local = -s * dx_world + c * dy_world;
  const double dyaw = wrapYaw(curr_yaw - prev_yaw);
  return Eigen::Vector3d(dx_local, dy_local, dyaw);
}

void integrateSe2Delta(double & x, double & y, double & yaw, const Eigen::Vector3d & delta_local)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  x += c * delta_local.x() - s * delta_local.y();
  y += s * delta_local.x() + c * delta_local.y();
  yaw = wrapYaw(yaw + delta_local.z());
}

}  // namespace

double GyroOdometerNode::normalizeYaw(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double GyroOdometerNode::yawFromRot(const Eigen::Matrix3d & R)
{
  return normalizeYaw(std::atan2(R(1, 0), R(0, 0)));
}

Eigen::Quaterniond GyroOdometerNode::quatFromYaw(double yaw)
{
  Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  q.normalize();
  return q;
}

GyroOdometerNode::GyroOdometerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("gyro_odometer", options)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Frames
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");

  // Topics
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
  wheel_speed_topic_ = declare_parameter<std::string>("wheel_speed_topic", "");
  reference_pose_topic_ = declare_parameter<std::string>("reference_pose_topic", "");
  points_topic_ = declare_parameter<std::string>("points_topic", "/localization/points_undistorted");

  out_odom_topic_ = declare_parameter<std::string>("out_odom_topic", "/localization/gyro_lidar_odom");
  out_filtered_odom_topic_ = declare_parameter<std::string>("out_filtered_odom_topic", "/localization/gyro_lidar_odom_filtered");
  out_stopped_topic_ = declare_parameter<std::string>("out_stopped_topic", "/localization/is_stopped");
  out_imu_topic_ = declare_parameter<std::string>("out_imu_topic", "/localization/imu_corrected");

  imu_corrected_enable_ = declare_parameter<bool>("imu_corrected.enable", true);
  imu_corrected_apply_tf_ = declare_parameter<bool>("imu_corrected.apply_tf", true);
  imu_corrected_transform_orientation_ =
    declare_parameter<bool>("imu_corrected.transform_orientation", false);

  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
  out_filtered_odom_enable_ =
    declare_parameter<bool>("out_filtered_odom.enable", true);
  filtered_odom_zero_when_stopped_ =
    declare_parameter<bool>("out_filtered_odom.zero_when_stopped", true);
  filtered_odom_lowpass_alpha_ =
    declare_parameter<double>("out_filtered_odom.lowpass_alpha", 0.85);
  filtered_odom_linear_rate_limit_mps2_ =
    declare_parameter<double>("out_filtered_odom.linear_rate_limit_mps2", 4.0);
  filtered_odom_lateral_rate_limit_mps2_ =
    declare_parameter<double>("out_filtered_odom.lateral_rate_limit_mps2", 2.0);
  filtered_odom_yaw_rate_limit_radps2_ =
    declare_parameter<double>("out_filtered_odom.yaw_rate_limit_radps2", 1.5);
  filtered_odom_reset_gap_sec_ =
    declare_parameter<double>("out_filtered_odom.reset_gap_sec", 1.0);

  // Stop detection
  stop_enable_ = declare_parameter<bool>("stop.enable", true);
  stop_speed_thr_mps_ = declare_parameter<double>("stop.speed_thr_mps", 0.15);
  stop_gyro_abs_thr_rad_s_ = declare_parameter<double>("stop.gyro_abs_thr_rad_s", 0.05);
  stop_acc_var_thr_ = declare_parameter<double>("stop.acc_var_thr", 0.0225);
  stop_hold_sec_ = declare_parameter<double>("stop.hold_sec", 0.5);

  // Gyro bias
  gyro_bias_enable_ = declare_parameter<bool>("gyro_bias.enable", true);
  gyro_bias_tau_sec_ = declare_parameter<double>("gyro_bias.tau_sec", 10.0);
  gyro_bias_max_abs_rad_s_ = declare_parameter<double>("gyro_bias.max_abs_rad_s", 0.5);
  bg_est_ = declare_parameter<double>("gyro_bias.initial_bg_rad_s", 0.0);

  // Wheel speed
  use_wheel_speed_ = declare_parameter<bool>("wheel_speed.use", false);
  wheel_speed_scale_ = declare_parameter<double>("wheel_speed.scale_factor", 1.0);
  wheel_speed_timeout_sec_ = declare_parameter<double>("wheel_speed.timeout_sec", 0.2);

  wheel_low_speed_enable_ = declare_parameter<bool>("wheel_speed.low_speed.enable", true);
  wheel_low_speed_deadband_mps_ = declare_parameter<double>("wheel_speed.low_speed.deadband_mps", 0.12);
  wheel_low_speed_acc_thr_mps2_ = declare_parameter<double>("wheel_speed.low_speed.acc_thr_mps2", 0.2);
  wheel_low_speed_blend_ = declare_parameter<double>("wheel_speed.low_speed.blend", 0.5);
  wheel_low_speed_max_corr_mps_ = declare_parameter<double>("wheel_speed.low_speed.max_corr_mps", 0.3);

  wheel_scale_est_enable_ = declare_parameter<bool>("wheel_speed.scale_estimation.enable", false);
  wheel_scale_est_tau_sec_ = declare_parameter<double>("wheel_speed.scale_estimation.tau_sec", 30.0);
  wheel_scale_est_min_ref_dist_m_ =
    declare_parameter<double>("wheel_speed.scale_estimation.min_ref_dist_m", 2.0);
  wheel_scale_est_min_wheel_dist_m_ =
    declare_parameter<double>("wheel_speed.scale_estimation.min_wheel_dist_m", 1.0);
  wheel_scale_min_ = declare_parameter<double>("wheel_speed.scale_estimation.min_scale", 0.5);
  wheel_scale_max_ = declare_parameter<double>("wheel_speed.scale_estimation.max_scale", 2.0);

  wheel_degeneracy_enable_ = declare_parameter<bool>("wheel_speed.degeneracy.enable", true);
  wheel_degeneracy_yaw_metric_m_ = declare_parameter<double>("wheel_speed.degeneracy.yaw_metric_m", 2.0);
  wheel_degeneracy_rel_eigenvalue_thr_ =
    declare_parameter<double>("wheel_speed.degeneracy.rel_eigenvalue_thr", 0.10);
  wheel_degeneracy_abs_eigenvalue_thr_ =
    declare_parameter<double>("wheel_speed.degeneracy.abs_eigenvalue_thr", 0.0);
  wheel_degeneracy_min_wheel_dist_m_ =
    declare_parameter<double>("wheel_speed.degeneracy.min_wheel_dist_m", 0.05);
  wheel_degeneracy_prior_blend_ = declare_parameter<double>("wheel_speed.degeneracy.prior_blend", 1.0);
  wheel_degeneracy_full_guess_use_current_prior_ =
    declare_parameter<bool>("wheel_speed.degeneracy.full_guess_use_current_prior", true);
  wheel_degeneracy_latch_hold_sec_ =
    declare_parameter<double>("wheel_speed.degeneracy.latch_hold_sec", 0.5);
  wheel_degeneracy_latch_off_streak_thr_ =
    declare_parameter<int>("wheel_speed.degeneracy.latch_off_streak_thr", 6);
  wheel_degeneracy_score_thr_ = declare_parameter<double>("wheel_speed.degeneracy.score_thr", 0.25);
  wheel_degeneracy_prior_conflict_trans_thr_m_ =
    declare_parameter<double>("wheel_speed.degeneracy.prior_conflict.trans_thr_m", 0.05);
  wheel_degeneracy_prior_conflict_yaw_thr_rad_ =
    declare_parameter<double>("wheel_speed.degeneracy.prior_conflict.yaw_thr_rad", 0.01);
  wheel_degeneracy_prior_conflict_metric_thr_m_ =
    declare_parameter<double>("wheel_speed.degeneracy.prior_conflict.metric_thr_m", 0.08);
  wheel_degeneracy_bad_fit_fitness_thr_ =
    declare_parameter<double>("wheel_speed.degeneracy.bad_fit_fitness_thr", 1.5);
  wheel_degeneracy_scan_wheel_speed_diff_thr_mps_ =
    declare_parameter<double>("wheel_speed.degeneracy.scan_wheel_speed_diff_thr_mps", 1.0);
  wheel_degeneracy_stationary_trans_thr_m_ =
    declare_parameter<double>("wheel_speed.degeneracy.stationary.trans_thr_m", 0.03);
  wheel_degeneracy_stationary_yaw_thr_rad_ =
    declare_parameter<double>("wheel_speed.degeneracy.stationary.yaw_thr_rad", 0.01);
  wheel_degeneracy_debug_pub_enable_ = declare_parameter<bool>("wheel_speed.degeneracy.debug_pub.enable", false);
  wheel_degeneracy_debug_topic_ = declare_parameter<std::string>(
    "wheel_speed.degeneracy.debug_pub.topic", "/localization/lidar_degeneracy_debug");
  out_degeneracy_enable_ = declare_parameter<bool>("out_degeneracy.enable", true);
  out_degeneracy_topic_ =
    declare_parameter<std::string>("out_degeneracy_topic", "/localization/lidar_degenerate");
  out_pose_mode_enable_ = declare_parameter<bool>("out_pose_mode.enable", true);
  out_pose_mode_topic_ =
    declare_parameter<std::string>("out_pose_mode_topic", "/localization/lidar_pose_mode");

  odom_cov_base_xy_step_ = declare_parameter<double>("odom_covariance.base_xy_step", odom_cov_base_xy_step_);
  odom_cov_base_yaw_step_ = declare_parameter<double>("odom_covariance.base_yaw_step", odom_cov_base_yaw_step_);
  odom_cov_xy_per_meter_ = declare_parameter<double>("odom_covariance.xy_per_meter", odom_cov_xy_per_meter_);
  odom_cov_yaw_per_rad_ = declare_parameter<double>("odom_covariance.yaw_per_rad", odom_cov_yaw_per_rad_);
  odom_cov_fitness_xy_scale_ =
    declare_parameter<double>("odom_covariance.fitness_xy_scale", odom_cov_fitness_xy_scale_);
  odom_cov_fitness_yaw_scale_ =
    declare_parameter<double>("odom_covariance.fitness_yaw_scale", odom_cov_fitness_yaw_scale_);
  odom_cov_degenerate_scale_ =
    declare_parameter<double>("odom_covariance.degenerate_scale", odom_cov_degenerate_scale_);
  odom_cov_wheel_assist_scale_ =
    declare_parameter<double>("odom_covariance.wheel_assist_scale", odom_cov_wheel_assist_scale_);
  odom_cov_invalid_xy_step_ =
    declare_parameter<double>("odom_covariance.invalid_xy_step", odom_cov_invalid_xy_step_);
  odom_cov_invalid_yaw_step_ =
    declare_parameter<double>("odom_covariance.invalid_yaw_step", odom_cov_invalid_yaw_step_);
  odom_cov_deadreckon_xy_per_sec_ = declare_parameter<double>(
    "odom_covariance.deadreckon_xy_per_sec", odom_cov_deadreckon_xy_per_sec_);
  odom_cov_deadreckon_yaw_per_sec_ = declare_parameter<double>(
    "odom_covariance.deadreckon_yaw_per_sec", odom_cov_deadreckon_yaw_per_sec_);

  // LiDAR odometry (scan-to-scan, backend selectable)
  lidar_odom_enable_ = declare_parameter<bool>("lidar_odom.enable", true);
  lidar_backend_ = declare_parameter<std::string>("lidar_odom.backend", lidar_backend_);
  lidar_registration_type_ = declare_parameter<std::string>("lidar_odom.registration_type", lidar_registration_type_);
  lidar_num_threads_ = declare_parameter<int>("lidar_odom.num_threads", lidar_num_threads_);
  lidar_timeout_sec_ = declare_parameter<double>("lidar_odom.timeout_sec", 0.5);
  lidar_min_range_m_ = declare_parameter<double>("lidar_odom.min_range_m", 2.0);
  lidar_max_range_m_ = declare_parameter<double>("lidar_odom.max_range_m", 80.0);
  lidar_voxel_leaf_m_ = declare_parameter<double>("lidar_odom.voxel_leaf_m", 0.4);

  gicp_max_corr_dist_m_ = declare_parameter<double>("lidar_odom.gicp.max_corr_dist_m", 2.5);
  gicp_max_iterations_ = declare_parameter<int>("lidar_odom.gicp.max_iterations", 30);
  gicp_trans_eps_ = declare_parameter<double>("lidar_odom.gicp.trans_eps", 1e-3);
  gicp_rot_eps_ = declare_parameter<double>("lidar_odom.gicp.rot_eps", 2e-3);
  gicp_corr_randomness_ = declare_parameter<int>("lidar_odom.gicp.corr_randomness", 20);
  gicp_fitness_max_ = declare_parameter<double>("lidar_odom.gicp.max_fitness", 5.0);
  gicp_voxel_resolution_ = declare_parameter<double>("lidar_odom.gicp.voxel_resolution", 1.0);
  lidar_pose_se2_enable_ = declare_parameter<bool>("lidar_odom.pose_se2.enable", true);
  lidar_yaw_blend_imu_ = declare_parameter<double>("lidar_odom.pose_se2.yaw_blend_imu", 0.0);
  lidar_guess_use_imu_yaw_only_ = declare_parameter<bool>("lidar_odom.pose_se2.guess_use_imu_yaw_only", true);

  lidar_smoother_enable_ = declare_parameter<bool>("lidar_odom.smoother.enable", true);
  lidar_smoother_window_size_ = declare_parameter<int>("lidar_odom.smoother.window_size", 20);
  lidar_smoother_w_imu_ = declare_parameter<double>("lidar_odom.smoother.w_imu", 3.0);
  lidar_smoother_w_scan_ = declare_parameter<double>("lidar_odom.smoother.w_scan", 1.0);
  lidar_smoother_lambda_ = declare_parameter<double>("lidar_odom.smoother.lambda", 0.5);
  lidar_smoother_fitness_sigma_ = declare_parameter<double>("lidar_odom.smoother.fitness_sigma", 1.0);
  lidar_smoother_min_scan_weight_ = declare_parameter<double>("lidar_odom.smoother.min_scan_weight", 0.05);
  lidar_smoother_max_scan_weight_ = declare_parameter<double>("lidar_odom.smoother.max_scan_weight", 5.0);
  lidar_smoother_zupt_enable_ = declare_parameter<bool>("lidar_odom.smoother.zupt.enable", false);
  lidar_smoother_zupt_w_trans_ = declare_parameter<double>("lidar_odom.smoother.zupt.w_trans", 25.0);
  lidar_smoother_zupt_w_yaw_ = declare_parameter<double>("lidar_odom.smoother.zupt.w_yaw", 25.0);
  lidar_smoother_nhc_enable_ = declare_parameter<bool>("lidar_odom.smoother.nhc.enable", false);
  lidar_smoother_nhc_w_lateral_ = declare_parameter<double>("lidar_odom.smoother.nhc.w_lateral", 2.0);
  lidar_smoother_nhc_huber_delta_m_ = declare_parameter<double>("lidar_odom.smoother.nhc.huber_delta_m", 0.10);

  if (use_wheel_speed_ && wheel_degeneracy_enable_) {
    const bool backend_has_hessian =
      (lidar_backend_ == "SMALL_GICP" || lidar_backend_ == "small_gicp" ||
       lidar_backend_ == "VGICP" || lidar_backend_ == "GICP");
    if (!backend_has_hessian) {
      RCLCPP_WARN(
        get_logger(),
        "wheel_speed.degeneracy is enabled, but lidar_odom.backend=%s does not expose small_gicp Hessian. Wheel assist will stay inactive.",
        lidar_backend_.c_str());
    }
  }
  if (wheel_degeneracy_enable_ && !use_wheel_speed_) {
    RCLCPP_INFO(
      get_logger(),
      "wheel_speed.degeneracy is enabled but wheel_speed.use=false. Degeneracy judgement/assist will stay disabled until wheel speed is enabled.");
  }
  if (wheel_degeneracy_debug_pub_enable_ && wheel_degeneracy_debug_topic_.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "wheel_speed.degeneracy.debug_pub.enable is true but topic is empty. Disabling degeneracy debug publisher.");
    wheel_degeneracy_debug_pub_enable_ = false;
  }

  // Publishers
  pub_odom_raw_ = create_publisher<nav_msgs::msg::Odometry>(out_odom_topic_, 10);
  if (out_filtered_odom_enable_ && !out_filtered_odom_topic_.empty() && out_filtered_odom_topic_ != out_odom_topic_) {
    pub_odom_filtered_ = create_publisher<nav_msgs::msg::Odometry>(out_filtered_odom_topic_, 10);
  } else if (out_filtered_odom_enable_ && !out_filtered_odom_topic_.empty() && out_filtered_odom_topic_ == out_odom_topic_) {
    RCLCPP_WARN(get_logger(), "out_filtered_odom_topic matches out_odom_topic; filtered odom publisher is disabled.");
  }
  pub_stopped_ = create_publisher<std_msgs::msg::Bool>(out_stopped_topic_, 10);
  if (out_degeneracy_enable_ && !out_degeneracy_topic_.empty()) {
    pub_degenerate_ = create_publisher<std_msgs::msg::Bool>(out_degeneracy_topic_, 10);
  }
  if (out_pose_mode_enable_ && !out_pose_mode_topic_.empty()) {
    pub_pose_mode_ = create_publisher<std_msgs::msg::UInt8>(out_pose_mode_topic_, 10);
  }
  if (imu_corrected_enable_) {
    pub_imu_corrected_ = create_publisher<sensor_msgs::msg::Imu>(out_imu_topic_, rclcpp::SensorDataQoS());
  }
  pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  if (wheel_degeneracy_debug_pub_enable_) {
    pub_degeneracy_debug_ = create_publisher<std_msgs::msg::String>(wheel_degeneracy_debug_topic_, 10);
  }

  // Subscribers
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    std::bind(&GyroOdometerNode::onImu, this, std::placeholders::_1));

  if (use_wheel_speed_) {
    if (wheel_speed_topic_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "wheel_speed.use is true but wheel_speed_topic is empty. Wheel assist will stay inactive; LiDAR-only localization continues.");
    } else {
      sub_wheel_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        wheel_speed_topic_, rclcpp::SensorDataQoS(),
        std::bind(&GyroOdometerNode::onWheelTwist, this, std::placeholders::_1));
    }
  }

  if (wheel_scale_est_enable_) {
    if (reference_pose_topic_.empty()) {
      RCLCPP_WARN(get_logger(), "wheel_speed.scale_estimation.enable is true but reference_pose_topic is empty. Disabling.");
      wheel_scale_est_enable_ = false;
    } else {
      sub_ref_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        reference_pose_topic_, 10,
        std::bind(&GyroOdometerNode::onReferencePose, this, std::placeholders::_1));
    }
  }

  if (lidar_odom_enable_) {
    sub_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GyroOdometerNode::onPoints, this, std::placeholders::_1));
    lidar_active_ = true;
  } else {
    lidar_active_ = false;
  }

  // Timer
  const double period = (publish_rate_hz_ > 1e-3) ? (1.0 / publish_rate_hz_) : 0.02;
  timer_ = create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&GyroOdometerNode::onPublishTimer, this));

  publishDiagnostics(now(), "OK", "pure_gyro_odometer started");
}

void GyroOdometerNode::onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Resolve IMU extrinsic (base <- imu_frame). We keep it cached.
  const std::string imu_frame = msg->header.frame_id;
  Eigen::Quaterniond q_base_imu = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d R_base_imu = Eigen::Matrix3d::Identity();
  bool tf_ok = true;
  if (imu_corrected_apply_tf_ && !imu_frame.empty() && imu_frame != base_frame_) {
    bool cached = false;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      cached = has_imu_extrinsic_ && (imu_frame_id_ == imu_frame);
      if (cached) {
        q_base_imu = q_base_imu_;
        R_base_imu = R_base_imu_;
      }
    }

    if (!cached) {
      try {
        const auto tf = tf_buffer_->lookupTransform(base_frame_, imu_frame, tf2::TimePointZero);
        q_base_imu = Eigen::Quaterniond(
          static_cast<double>(tf.transform.rotation.w),
          static_cast<double>(tf.transform.rotation.x),
          static_cast<double>(tf.transform.rotation.y),
          static_cast<double>(tf.transform.rotation.z));
        q_base_imu.normalize();
        R_base_imu = q_base_imu.toRotationMatrix();

        std::lock_guard<std::mutex> lk(mtx_);
        has_imu_extrinsic_ = true;
        imu_frame_id_ = imu_frame;
        q_base_imu_ = q_base_imu;
        R_base_imu_ = R_base_imu;
      } catch (const tf2::TransformException & ex) {
        tf_ok = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[pure_gyro_odometer] TF lookup failed for %s <- %s: %s. Using identity IMU extrinsic.",
          base_frame_.c_str(), imu_frame.c_str(), ex.what());
        q_base_imu = Eigen::Quaterniond::Identity();
        R_base_imu = Eigen::Matrix3d::Identity();
      }
    }
  }

  // Rotate gyro/acc into base frame.
  Eigen::Vector3d gyro_imu(
    std::isfinite(msg->angular_velocity.x) ? msg->angular_velocity.x : 0.0,
    std::isfinite(msg->angular_velocity.y) ? msg->angular_velocity.y : 0.0,
    std::isfinite(msg->angular_velocity.z) ? msg->angular_velocity.z : 0.0);
  Eigen::Vector3d acc_imu(
    std::isfinite(msg->linear_acceleration.x) ? msg->linear_acceleration.x : 0.0,
    std::isfinite(msg->linear_acceleration.y) ? msg->linear_acceleration.y : 0.0,
    std::isfinite(msg->linear_acceleration.z) ? msg->linear_acceleration.z : 0.0);

  const Eigen::Vector3d gyro_base = R_base_imu * gyro_imu;
  const Eigen::Vector3d acc_base = R_base_imu * acc_imu;

  ImuSample s;
  s.stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());

  s.gyro_z = gyro_base.z();
  s.acc = acc_base;

  std::lock_guard<std::mutex> lk(mtx_);

  // buffer
  imu_buf_.push_back(s);
  // keep up to ~2 sec of IMU
  const double keep_sec = 2.0;
  while (!imu_buf_.empty()) {
    if ((s.stamp - imu_buf_.front().stamp).seconds() > keep_sec) imu_buf_.pop_front();
    else break;
  }

  // integrate yaw and acceleration-derived speed (for low-speed correction)
  if (!has_last_imu_) {
    last_imu_stamp_ = s.stamp;
    has_last_imu_ = true;
    // initialize v_acc_est_ with 0
    v_acc_est_ = 0.0;
    has_v_acc_ = true;

    // Publish corrected IMU as soon as the first sample arrives (bias correction will apply from the next step).
    if (imu_corrected_enable_ && pub_imu_corrected_) {
      sensor_msgs::msg::Imu out;
      out.header = msg->header;
      out.header.frame_id = base_frame_;
      out.angular_velocity.x = gyro_base.x();
      out.angular_velocity.y = gyro_base.y();
      out.angular_velocity.z = gyro_base.z();
      out.linear_acceleration.x = acc_base.x();
      out.linear_acceleration.y = acc_base.y();
      out.linear_acceleration.z = acc_base.z();

      // Orientation: if we cannot (or do not want to) transform it, publish as "unknown".
      if (imu_frame.empty() || imu_frame == base_frame_) {
        out.orientation = msg->orientation;
        out.orientation_covariance = msg->orientation_covariance;
      } else if (imu_corrected_transform_orientation_ && tf_ok) {
        Eigen::Quaterniond q_w_i(
          static_cast<double>(msg->orientation.w),
          static_cast<double>(msg->orientation.x),
          static_cast<double>(msg->orientation.y),
          static_cast<double>(msg->orientation.z));
        if (std::isfinite(q_w_i.w()) && std::isfinite(q_w_i.x()) && std::isfinite(q_w_i.y()) && std::isfinite(q_w_i.z()) &&
          q_w_i.norm() > 1e-6)
        {
          q_w_i.normalize();
          const Eigen::Quaterniond q_i_b = q_base_imu.inverse();
          Eigen::Quaterniond q_w_b = q_w_i * q_i_b;
          q_w_b.normalize();
          out.orientation.w = q_w_b.w();
          out.orientation.x = q_w_b.x();
          out.orientation.y = q_w_b.y();
          out.orientation.z = q_w_b.z();
          out.orientation_covariance = msg->orientation_covariance;
        } else {
          out.orientation.w = 1.0;
          out.orientation.x = 0.0;
          out.orientation.y = 0.0;
          out.orientation.z = 0.0;
          out.orientation_covariance[0] = -1.0;
        }
      } else {
        out.orientation.w = 1.0;
        out.orientation.x = 0.0;
        out.orientation.y = 0.0;
        out.orientation.z = 0.0;
        out.orientation_covariance[0] = -1.0;
      }

      // Covariances: rotate if they are valid and TF was applied.
      out.angular_velocity_covariance = msg->angular_velocity_covariance;
      out.linear_acceleration_covariance = msg->linear_acceleration_covariance;
      if (!imu_frame.empty() && imu_frame != base_frame_ && tf_ok) {
        auto rotate_cov = [&](const std::array<double, 9> & cov_in) -> std::array<double, 9> {
          // -1 means unknown
          if (cov_in[0] < 0.0) return cov_in;
          Eigen::Matrix3d C;
          C << cov_in[0], cov_in[1], cov_in[2],
               cov_in[3], cov_in[4], cov_in[5],
               cov_in[6], cov_in[7], cov_in[8];
          const Eigen::Matrix3d Cb = R_base_imu * C * R_base_imu.transpose();
          std::array<double, 9> out;
          out[0] = Cb(0, 0); out[1] = Cb(0, 1); out[2] = Cb(0, 2);
          out[3] = Cb(1, 0); out[4] = Cb(1, 1); out[5] = Cb(1, 2);
          out[6] = Cb(2, 0); out[7] = Cb(2, 1); out[8] = Cb(2, 2);
          return out;
        };
        out.angular_velocity_covariance = rotate_cov(out.angular_velocity_covariance);
        out.linear_acceleration_covariance = rotate_cov(out.linear_acceleration_covariance);
      }

      pub_imu_corrected_->publish(out);
    }
    return;
  }
  const double dt = (s.stamp - last_imu_stamp_).seconds();
  if (std::isfinite(dt) && dt > 0.0 && dt < 1.0) {
    // Update stop state using the latest timestamp (needed for bias update gating)
    updateStopState(s.stamp);

    // Gyro bias estimation when stopped
    if (gyro_bias_enable_ && is_stopped_) {
      const double alpha = dt / (gyro_bias_tau_sec_ + dt);
      bg_est_ = (1.0 - alpha) * bg_est_ + alpha * s.gyro_z;
      bg_est_ = std::max(-gyro_bias_max_abs_rad_s_, std::min(gyro_bias_max_abs_rad_s_, bg_est_));
    }

    // IMU yaw integration (bias corrected)
    yaw_imu_ = normalizeYaw(yaw_imu_ + (s.gyro_z - bg_est_) * dt);

    // Acceleration integration for low-speed assist (base frame x assumed)
    if (has_v_acc_) {
      // Conservative clip to avoid runaway due to gravity/pitch
      const double ax = std::max(-5.0, std::min(5.0, s.acc.x()));
      v_acc_est_ += ax * dt;
      // keep in reasonable bounds
      v_acc_est_ = std::max(-50.0, std::min(50.0, v_acc_est_));
    }
  }

  last_imu_stamp_ = s.stamp;

  // Publish corrected IMU (TF-applied + z gyro bias corrected) so that downstream nodes that do not apply TF
  // can still consume base_frame-consistent IMU data.
  if (imu_corrected_enable_ && pub_imu_corrected_) {
    const double bg_now = bg_est_;

    sensor_msgs::msg::Imu out;
    out.header = msg->header;
    out.header.frame_id = base_frame_;

    out.angular_velocity.x = gyro_base.x();
    out.angular_velocity.y = gyro_base.y();
    out.angular_velocity.z = gyro_base.z() - bg_now;

    out.linear_acceleration.x = acc_base.x();
    out.linear_acceleration.y = acc_base.y();
    out.linear_acceleration.z = acc_base.z();

    // Orientation: if we cannot (or do not want to) transform it, publish as "unknown".
    if (imu_frame.empty() || imu_frame == base_frame_) {
      out.orientation = msg->orientation;
      out.orientation_covariance = msg->orientation_covariance;
    } else if (imu_corrected_transform_orientation_ && tf_ok) {
      Eigen::Quaterniond q_w_i(
        static_cast<double>(msg->orientation.w),
        static_cast<double>(msg->orientation.x),
        static_cast<double>(msg->orientation.y),
        static_cast<double>(msg->orientation.z));
      if (std::isfinite(q_w_i.w()) && std::isfinite(q_w_i.x()) && std::isfinite(q_w_i.y()) && std::isfinite(q_w_i.z()) &&
        q_w_i.norm() > 1e-6)
      {
        q_w_i.normalize();
        const Eigen::Quaterniond q_i_b = q_base_imu.inverse();
        Eigen::Quaterniond q_w_b = q_w_i * q_i_b;
        q_w_b.normalize();
        out.orientation.w = q_w_b.w();
        out.orientation.x = q_w_b.x();
        out.orientation.y = q_w_b.y();
        out.orientation.z = q_w_b.z();
        out.orientation_covariance = msg->orientation_covariance;
      } else {
        out.orientation.w = 1.0;
        out.orientation.x = 0.0;
        out.orientation.y = 0.0;
        out.orientation.z = 0.0;
        out.orientation_covariance[0] = -1.0;
      }
    } else {
      out.orientation.w = 1.0;
      out.orientation.x = 0.0;
      out.orientation.y = 0.0;
      out.orientation.z = 0.0;
      out.orientation_covariance[0] = -1.0;
    }

    // Covariances: rotate if they are valid and TF was applied.
    out.angular_velocity_covariance = msg->angular_velocity_covariance;
    out.linear_acceleration_covariance = msg->linear_acceleration_covariance;
    if (!imu_frame.empty() && imu_frame != base_frame_ && tf_ok) {
      auto rotate_cov = [&](const std::array<double, 9> & cov_in) -> std::array<double, 9> {
        // -1 means unknown
        if (cov_in[0] < 0.0) return cov_in;
        Eigen::Matrix3d C;
        C << cov_in[0], cov_in[1], cov_in[2],
             cov_in[3], cov_in[4], cov_in[5],
             cov_in[6], cov_in[7], cov_in[8];
        const Eigen::Matrix3d Cb = R_base_imu * C * R_base_imu.transpose();
        std::array<double, 9> out;
        out[0] = Cb(0, 0); out[1] = Cb(0, 1); out[2] = Cb(0, 2);
        out[3] = Cb(1, 0); out[4] = Cb(1, 1); out[5] = Cb(1, 2);
        out[6] = Cb(2, 0); out[7] = Cb(2, 1); out[8] = Cb(2, 2);
        return out;
      };
      out.angular_velocity_covariance = rotate_cov(out.angular_velocity_covariance);
      out.linear_acceleration_covariance = rotate_cov(out.linear_acceleration_covariance);
    }

    pub_imu_corrected_->publish(out);
  }
}

void GyroOdometerNode::onWheelTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  double v_raw = msg->twist.linear.x;
  if (!std::isfinite(v_raw)) v_raw = 0.0;

  std::lock_guard<std::mutex> lk(mtx_);

  // accumulate distance for optional scale estimation
  if (wheel_scale_est_enable_ && has_wheel_) {
    const double dt = (stamp - last_wheel_.stamp).seconds();
    if (std::isfinite(dt) && dt > 0.0 && dt < 1.0) {
      wheel_dist_since_ref_ += std::fabs(v_raw) * dt;
    }
  }

  WheelSample sample;
  sample.stamp = stamp;
  sample.v_raw = v_raw;
  wheel_buf_.push_back(sample);
  while (!wheel_buf_.empty()) {
    if ((stamp - wheel_buf_.front().stamp).seconds() > 5.0) {
      wheel_buf_.pop_front();
    } else {
      break;
    }
  }

  last_wheel_ = sample;
  has_wheel_ = true;

  // anchor accel-integrated speed estimate to wheel speed when not in deadband
  if (wheel_low_speed_enable_) {
    if (std::fabs(v_raw) > wheel_low_speed_deadband_mps_) {
      v_acc_est_ = v_raw * wheel_speed_scale_;
      has_v_acc_ = true;
    }
  }
}

void GyroOdometerNode::onReferencePose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!wheel_scale_est_enable_) return;

  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;

  std::lock_guard<std::mutex> lk(mtx_);

  if (!has_ref_pose_) {
    has_ref_pose_ = true;
    last_ref_stamp_ = stamp;
    last_ref_x_ = x;
    last_ref_y_ = y;
    wheel_dist_since_ref_ = 0.0;
    return;
  }

  const double dt = (stamp - last_ref_stamp_).seconds();
  if (!std::isfinite(dt) || dt <= 0.0) {
    last_ref_stamp_ = stamp;
    last_ref_x_ = x;
    last_ref_y_ = y;
    wheel_dist_since_ref_ = 0.0;
    return;
  }

  const double ref_dist = std::hypot(x - last_ref_x_, y - last_ref_y_);

  if (ref_dist >= wheel_scale_est_min_ref_dist_m_ && wheel_dist_since_ref_ >= wheel_scale_est_min_wheel_dist_m_) {
    const double k_meas = ref_dist / std::max(1e-6, wheel_dist_since_ref_);
    const double alpha = dt / (wheel_scale_est_tau_sec_ + dt);
    const double k_new = (1.0 - alpha) * wheel_speed_scale_ + alpha * k_meas;
    wheel_speed_scale_ = std::max(wheel_scale_min_, std::min(wheel_scale_max_, k_new));
  }

  last_ref_stamp_ = stamp;
  last_ref_x_ = x;
  last_ref_y_ = y;
  wheel_dist_since_ref_ = 0.0;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr filterAndDownsample(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & in,
  double min_range, double max_range,
  double voxel_leaf)
{
  auto filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  filtered->reserve(in->size());
  const double min_r2 = min_range * min_range;
  const double max_r2 = max_range * max_range;
  for (const auto & p : in->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const double r2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y + static_cast<double>(p.z) * p.z;
    if (r2 < min_r2 || r2 > max_r2) continue;
    filtered->push_back(p);
  }

  if (voxel_leaf > 1e-6) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(static_cast<float>(voxel_leaf), static_cast<float>(voxel_leaf), static_cast<float>(voxel_leaf));
    vg.setInputCloud(filtered);
    auto ds = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    vg.filter(*ds);
    return ds;
  }
  return filtered;
}

bool GyroOdometerNode::computeImuDeltaYaw(const rclcpp::Time & t0, const rclcpp::Time & t1, double & out_dyaw) const
{
  out_dyaw = 0.0;
  if (t1 <= t0) return false;
  std::lock_guard<std::mutex> lk(mtx_);
  if (imu_buf_.size() < 2) return false;

  const double t0s = t0.seconds();
  const double t1s = t1.seconds();
  const double min_t = imu_buf_.front().stamp.seconds();
  const double max_t = imu_buf_.back().stamp.seconds();
  if (t0s < min_t || t1s > max_t) return false;

  std::vector<std::pair<double,double>> samples;
  samples.reserve(imu_buf_.size() + 2);
  for (const auto & s : imu_buf_) samples.emplace_back(s.stamp.seconds(), s.gyro_z - bg_est_);

  auto interp_rate = [&](double ts) -> double {
    if (ts <= samples.front().first) return samples.front().second;
    if (ts >= samples.back().first) return samples.back().second;
    auto it = std::lower_bound(samples.begin(), samples.end(), ts, [](const auto & a, double v){ return a.first < v; });
    if (it == samples.begin()) return it->second;
    auto it0 = std::prev(it);
    const double dt = it->first - it0->first;
    if (dt <= 1e-9) return it->second;
    const double r = (ts - it0->first) / dt;
    return (1.0 - r) * it0->second + r * it->second;
  };

  double last_t = t0s;
  double last_w = interp_rate(t0s);
  for (const auto & s : samples) {
    if (s.first <= t0s || s.first >= t1s) continue;
    out_dyaw += 0.5 * (last_w + s.second) * (s.first - last_t);
    last_t = s.first;
    last_w = s.second;
  }
  const double end_w = interp_rate(t1s);
  out_dyaw += 0.5 * (last_w + end_w) * (t1s - last_t);
  out_dyaw = normalizeYaw(out_dyaw);
  return std::isfinite(out_dyaw);
}

bool GyroOdometerNode::computeWheelDistance(const rclcpp::Time & t0, const rclcpp::Time & t1, double & out_dist) const
{
  out_dist = 0.0;
  if (t1 <= t0) return false;

  std::lock_guard<std::mutex> lk(mtx_);
  if (wheel_buf_.empty()) return false;

  const double t0s = t0.seconds();
  const double t1s = t1.seconds();
  const double min_t = wheel_buf_.front().stamp.seconds();
  const double max_t = wheel_buf_.back().stamp.seconds();
  if (t0s < min_t) return false;
  if (t1s > max_t + std::max(0.0, wheel_speed_timeout_sec_)) return false;

  std::vector<std::pair<double, double>> samples;
  samples.reserve(wheel_buf_.size());
  for (const auto & s : wheel_buf_) {
    samples.emplace_back(s.stamp.seconds(), s.v_raw * wheel_speed_scale_);
  }
  if (samples.empty()) return false;

  auto interp_speed = [&](double ts) -> double {
    if (ts <= samples.front().first) return samples.front().second;
    if (ts >= samples.back().first) return samples.back().second;
    auto it = std::lower_bound(
      samples.begin(), samples.end(), ts,
      [](const auto & a, double v) { return a.first < v; });
    if (it == samples.begin()) return it->second;
    auto it0 = std::prev(it);
    const double dt = it->first - it0->first;
    if (dt <= 1e-9) return it->second;
    const double r = (ts - it0->first) / dt;
    return (1.0 - r) * it0->second + r * it->second;
  };

  double last_t = t0s;
  double last_v = interp_speed(t0s);
  for (const auto & s : samples) {
    if (s.first <= t0s || s.first >= t1s) continue;
    out_dist += 0.5 * (last_v + s.second) * (s.first - last_t);
    last_t = s.first;
    last_v = s.second;
  }
  const double end_v = interp_speed(t1s);
  out_dist += 0.5 * (last_v + end_v) * (t1s - last_t);
  return std::isfinite(out_dist);
}

double GyroOdometerNode::scanYawWeight(double fitness, bool converged) const
{
  // Yaw from scan-to-scan tends to be the first thing that breaks during turns.
  // Use IMU as the primary predictor and let scan yaw act as a correction only
  // when the registration quality is actually good.
  double w = lidar_smoother_w_scan_;
  const double sigma = std::max(1e-3, lidar_smoother_fitness_sigma_);
  if (!std::isfinite(fitness)) {
    return std::max(0.0, lidar_smoother_min_scan_weight_);
  }
  w *= std::exp(-std::max(0.0, fitness) / sigma);
  if (!converged) {
    // Strongly down-weight non-converged scan yaw so that IMU propagation dominates.
    w *= 0.1;
  }
  w = std::max(lidar_smoother_min_scan_weight_, std::min(lidar_smoother_max_scan_weight_, w));
  return w;
}

bool GyroOdometerNode::updateMiniSmootherLocked(const ScanFactor & factor)
{
  if (!lidar_smoother_enable_) return false;
  if (!has_odom_pose_) {
    odom_yaw_ = yaw_imu_;
    has_odom_pose_ = true;
  }
  if (!has_smoother_base_) {
    smoother_base_x_ = odom_x_;
    smoother_base_y_ = odom_y_;
    smoother_base_yaw_ = odom_yaw_;
    has_smoother_base_ = true;
  }

  auto scan_weight = [&](const ScanFactor & f) {
    double w = scanYawWeight(f.fitness, f.converged);
    w = std::max(lidar_smoother_min_scan_weight_, std::min(lidar_smoother_max_scan_weight_, w));
    return w;
  };

  auto fused_delta = [&](const ScanFactor & f) {
    const double ws = scan_weight(f);
    const double wi = std::max(1e-6, lidar_smoother_w_imu_);
    return normalizeYaw((ws * f.dyaw_scan + wi * f.dyaw_imu) / (ws + wi));
  };

  auto advance_base_with_factor = [&](const ScanFactor & f) {
    const double dtheta = fused_delta(f);
    const double yaw_mid = smoother_base_yaw_ + 0.5 * dtheta;
    const double c = std::cos(yaw_mid);
    const double s = std::sin(yaw_mid);
    smoother_base_x_ += c * f.dx - s * f.dy;
    smoother_base_y_ += s * f.dx + c * f.dy;
    smoother_base_yaw_ = normalizeYaw(smoother_base_yaw_ + dtheta);
  };

  // Keep a fixed-lag window by advancing the base with the oldest optimized state.
  while (static_cast<int>(scan_factor_buf_.size()) >= std::max(1, lidar_smoother_window_size_)) {
    const int expected_size = 3 * (static_cast<int>(scan_factor_buf_.size()) + 1);
    if (has_last_smoother_solution_ && last_smoother_solution_.size() == expected_size && expected_size >= 6) {
      smoother_base_x_ = last_smoother_solution_(3);
      smoother_base_y_ = last_smoother_solution_(4);
      smoother_base_yaw_ = normalizeYaw(last_smoother_solution_(5));
      last_smoother_solution_ = last_smoother_solution_.segment(3, last_smoother_solution_.size() - 3).eval();
    } else {
      advance_base_with_factor(scan_factor_buf_.front());
      last_smoother_solution_.resize(0);
      has_last_smoother_solution_ = false;
    }
    scan_factor_buf_.pop_front();
  }

  scan_factor_buf_.push_back(factor);
  const int N = static_cast<int>(scan_factor_buf_.size());
  if (N <= 0) return false;
  const int S = N + 1;  // state count
  const int D = 3 * S;  // x,y,yaw for each state

  Eigen::VectorXd x = Eigen::VectorXd::Zero(D);
  x(0) = smoother_base_x_;
  x(1) = smoother_base_y_;
  x(2) = smoother_base_yaw_;

  // Initialize states by propagating the base pose using IMU yaw prior and scan translation.
  for (int i = 0; i < N; ++i) {
    const auto & f = scan_factor_buf_[i];
    const int idx_i = 3 * i;
    const int idx_j = 3 * (i + 1);
    const double dtheta = normalizeYaw(f.dyaw_imu);
    const double yaw_mid = x(idx_i + 2) + 0.5 * dtheta;
    const double c = std::cos(yaw_mid);
    const double s = std::sin(yaw_mid);
    x(idx_j + 0) = x(idx_i + 0) + c * f.dx - s * f.dy;
    x(idx_j + 1) = x(idx_i + 1) + s * f.dx + c * f.dy;
    x(idx_j + 2) = normalizeYaw(x(idx_i + 2) + dtheta);
  }

  auto add_scalar_residual = [&](Eigen::MatrixXd & H, Eigen::VectorXd & g,
                                 int idx_a, const Eigen::Vector3d & Ja,
                                 int idx_b, const Eigen::Vector3d & Jb,
                                 double r, double w) {
    if (w <= 0.0 || !std::isfinite(r)) return;
    H.block<3, 3>(idx_a, idx_a).noalias() += w * (Ja * Ja.transpose());
    g.segment<3>(idx_a).noalias() += w * Ja * r;
    if (idx_b >= 0) {
      H.block<3, 3>(idx_a, idx_b).noalias() += w * (Ja * Jb.transpose());
      H.block<3, 3>(idx_b, idx_a).noalias() += w * (Jb * Ja.transpose());
      H.block<3, 3>(idx_b, idx_b).noalias() += w * (Jb * Jb.transpose());
      g.segment<3>(idx_b).noalias() += w * Jb * r;
    }
  };

  auto huber_scale = [&](double r, double delta) {
    if (!(delta > 0.0) || !std::isfinite(r)) return 1.0;
    const double a = std::fabs(r);
    if (a <= delta) return 1.0;
    return delta / a;
  };

  const double lambda = std::max(0.0, lidar_smoother_lambda_);
  const double zupt_w_trans = std::max(0.0, lidar_smoother_zupt_w_trans_);
  const double zupt_w_yaw = std::max(0.0, lidar_smoother_zupt_w_yaw_);
  const double nhc_w = std::max(0.0, lidar_smoother_nhc_w_lateral_);
  const double nhc_huber_delta = std::max(0.0, lidar_smoother_nhc_huber_delta_m_);
  const int max_iter = 5;
  for (int iter = 0; iter < max_iter; ++iter) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(D, D);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(D);

    // Strong prior on the base state.
    const double prior_w_pos = 1e6;
    const double prior_w_yaw = 1e6;
    for (int c = 0; c < 2; ++c) {
      const double r = x(c) - (c == 0 ? smoother_base_x_ : smoother_base_y_);
      H(c, c) += prior_w_pos;
      g(c) += prior_w_pos * r;
    }
    {
      const double r = normalizeYaw(x(2) - smoother_base_yaw_);
      H(2, 2) += prior_w_yaw;
      g(2) += prior_w_yaw * r;
    }

    // Tiny priors on all states to keep the linear system well-conditioned when scan weight goes low.
    const double weak_pos_prior = 1e-6;
    const double weak_yaw_prior = 1e-6;
    for (int i = 1; i < S; ++i) {
      const int idx = 3 * i;
      H(idx + 0, idx + 0) += weak_pos_prior;
      H(idx + 1, idx + 1) += weak_pos_prior;
      H(idx + 2, idx + 2) += weak_yaw_prior;
    }

    // Motion factors.
    for (int i = 0; i < N; ++i) {
      const auto & f = scan_factor_buf_[i];
      const int idx_i = 3 * i;
      const int idx_j = 3 * (i + 1);

      const double xi = x(idx_i + 0);
      const double yi = x(idx_i + 1);
      const double ti = x(idx_i + 2);
      const double xj = x(idx_j + 0);
      const double yj = x(idx_j + 1);
      const double tj = x(idx_j + 2);

      const double dpx = xj - xi;
      const double dpy = yj - yi;
      const double c = std::cos(ti);
      const double s = std::sin(ti);
      const double pred_dx = c * dpx + s * dpy;
      const double pred_dy = -s * dpx + c * dpy;

      const double w_scan = scan_weight(f);
      const double w_trans = std::max(lidar_smoother_min_scan_weight_, w_scan) * std::max(1e-6, lidar_smoother_w_scan_);
      const double w_yaw_scan = std::max(lidar_smoother_min_scan_weight_, w_scan) * std::max(1e-6, lidar_smoother_w_scan_);
      const double w_yaw_imu = std::max(1e-6, lidar_smoother_w_imu_);

      const Eigen::Vector3d Ji_dx(-c, -s, pred_dy);
      const Eigen::Vector3d Jj_dx( c,  s, 0.0);
      const Eigen::Vector3d Ji_dy( s, -c, -pred_dx);
      const Eigen::Vector3d Jj_dy(-s,  c, 0.0);

      // Translation residual in frame i.
      add_scalar_residual(H, g, idx_i, Ji_dx, idx_j, Jj_dx, pred_dx - f.dx, w_trans);
      add_scalar_residual(H, g, idx_i, Ji_dy, idx_j, Jj_dy, pred_dy - f.dy, w_trans);

      // Relative yaw from scan.
      {
        const double r = normalizeYaw((tj - ti) - f.dyaw_scan);
        const Eigen::Vector3d Ji(0.0, 0.0, -1.0);
        const Eigen::Vector3d Jj(0.0, 0.0,  1.0);
        add_scalar_residual(H, g, idx_i, Ji, idx_j, Jj, r, w_yaw_scan);
      }
      // Relative yaw from IMU prediction.
      {
        const double r = normalizeYaw((tj - ti) - f.dyaw_imu);
        const Eigen::Vector3d Ji(0.0, 0.0, -1.0);
        const Eigen::Vector3d Jj(0.0, 0.0,  1.0);
        add_scalar_residual(H, g, idx_i, Ji, idx_j, Jj, r, w_yaw_imu);
      }

      // Zero-velocity update (ZUPT) factor when the interval is confidently stationary.
      if (lidar_smoother_zupt_enable_ && f.stationary) {
        add_scalar_residual(H, g, idx_i, Ji_dx, idx_j, Jj_dx, pred_dx, zupt_w_trans);
        add_scalar_residual(H, g, idx_i, Ji_dy, idx_j, Jj_dy, pred_dy, zupt_w_trans);
        const double r = normalizeYaw(tj - ti);
        const Eigen::Vector3d Ji(0.0, 0.0, -1.0);
        const Eigen::Vector3d Jj(0.0, 0.0,  1.0);
        add_scalar_residual(H, g, idx_i, Ji, idx_j, Jj, r, zupt_w_yaw);
      }

      // Midpoint non-holonomic constraint (NHC): lateral displacement should stay near zero
      // in the body frame at the midpoint orientation of the interval.
      if (lidar_smoother_nhc_enable_ && nhc_w > 0.0) {
        const double dtheta = normalizeYaw(tj - ti);
        const double half_theta = 0.5 * dtheta;
        const double ch = std::cos(half_theta);
        const double sh = std::sin(half_theta);
        const double dx_mid = ch * pred_dx + sh * pred_dy;
        const double dy_mid = -sh * pred_dx + ch * pred_dy;

        Eigen::Vector3d Ji_nhc = (-sh) * Ji_dx + ch * Ji_dy;
        Eigen::Vector3d Jj_nhc = (-sh) * Jj_dx + ch * Jj_dy;
        Ji_nhc(2) += 0.5 * dx_mid;
        Jj_nhc(2) += -0.5 * dx_mid;

        const double w = nhc_w * huber_scale(dy_mid, nhc_huber_delta);
        add_scalar_residual(H, g, idx_i, Ji_nhc, idx_j, Jj_nhc, dy_mid, w);
      }
    }

    // Fixed-lag smoothness prior on second differences of x/y/yaw.
    if (lambda > 0.0 && S >= 3) {
      const double wx = lambda;
      const double wy = lambda;
      const double wt = lambda;
      for (int i = 1; i + 1 < S; ++i) {
        for (int comp = 0; comp < 3; ++comp) {
          const int idx_m = 3 * (i - 1) + comp;
          const int idx_0 = 3 * i + comp;
          const int idx_p = 3 * (i + 1) + comp;
          double r = x(idx_p) - 2.0 * x(idx_0) + x(idx_m);
          if (comp == 2) r = normalizeYaw(r);
          const double w = (comp == 0) ? wx : (comp == 1 ? wy : wt);
          H(idx_m, idx_m) += w;
          H(idx_0, idx_0) += 4.0 * w;
          H(idx_p, idx_p) += w;
          H(idx_m, idx_0) += -2.0 * w; H(idx_0, idx_m) += -2.0 * w;
          H(idx_m, idx_p) +=  1.0 * w; H(idx_p, idx_m) +=  1.0 * w;
          H(idx_0, idx_p) += -2.0 * w; H(idx_p, idx_0) += -2.0 * w;
          g(idx_m) += w * r;
          g(idx_0) += -2.0 * w * r;
          g(idx_p) += w * r;
        }
      }
    }

    Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
    if (ldlt.info() != Eigen::Success) {
      break;
    }
    Eigen::VectorXd dx = ldlt.solve(-g);
    if (!dx.allFinite()) {
      break;
    }

    x += dx;
    for (int i = 0; i < S; ++i) {
      x(3 * i + 2) = normalizeYaw(x(3 * i + 2));
    }
    if (dx.norm() < 1e-6) {
      break;
    }
  }

  if (!x.allFinite()) {
    last_smoother_solution_.resize(0);
    has_last_smoother_solution_ = false;
    return false;
  }

  odom_x_ = x(3 * N + 0);
  odom_y_ = x(3 * N + 1);
  odom_yaw_ = normalizeYaw(x(3 * N + 2));
  last_smoother_solution_ = x;
  has_last_smoother_solution_ = true;
  return true;
}

void GyroOdometerNode::onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!lidar_odom_enable_) return;

  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());

  // Resolve static extrinsic (base <- scan_frame). We keep it cached.
  const std::string scan_frame = msg->header.frame_id;
  Eigen::Matrix4f T_base_scan = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T_scan_base = Eigen::Matrix4f::Identity();
  bool has_extrinsic = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    has_extrinsic = has_scan_extrinsic_ && (scan_frame_id_ == scan_frame);
    if (has_extrinsic) {
      T_base_scan = T_base_scan_;
      T_scan_base = T_scan_base_;
    }
  }
  if (!has_extrinsic) {
    if (scan_frame.empty() || scan_frame == base_frame_) {
      T_base_scan = Eigen::Matrix4f::Identity();
      T_scan_base = Eigen::Matrix4f::Identity();
      has_extrinsic = true;
    } else {
      try {
        const auto tf = tf_buffer_->lookupTransform(base_frame_, scan_frame, tf2::TimePointZero);
        Eigen::Quaternionf q(
          static_cast<float>(tf.transform.rotation.w),
          static_cast<float>(tf.transform.rotation.x),
          static_cast<float>(tf.transform.rotation.y),
          static_cast<float>(tf.transform.rotation.z));
        q.normalize();
        T_base_scan.setIdentity();
        T_base_scan.block<3, 3>(0, 0) = q.toRotationMatrix();
        T_base_scan(0, 3) = static_cast<float>(tf.transform.translation.x);
        T_base_scan(1, 3) = static_cast<float>(tf.transform.translation.y);
        T_base_scan(2, 3) = static_cast<float>(tf.transform.translation.z);
        T_scan_base = T_base_scan.inverse();
        has_extrinsic = true;

        std::lock_guard<std::mutex> lk(mtx_);
        has_scan_extrinsic_ = true;
        scan_frame_id_ = scan_frame;
        T_base_scan_ = T_base_scan;
        T_scan_base_ = T_scan_base;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[pure_gyro_odometer] TF lookup failed for %s <- %s: %s. Using identity extrinsic.",
          base_frame_.c_str(), scan_frame.c_str(), ex.what());
        T_base_scan = Eigen::Matrix4f::Identity();
        T_scan_base = Eigen::Matrix4f::Identity();
      }
    }
  }

  // Convert to PCL and downsample.
  auto cloud_in = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud_in);
  auto cloud = filterAndDownsample(cloud_in, lidar_min_range_m_, lidar_max_range_m_, lidar_voxel_leaf_m_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr prev;
  Eigen::Matrix4f guess;
  rclcpp::Time prev_lidar_stamp = stamp;
  std::string guess_mode_used{"full"};
  bool force_full_guess = false;
  bool prev_interval_degenerate = false;
  bool prev_interval_scan_rejected = false;
  bool has_last_lidar_yaw_imu_for_guess = false;
  double last_lidar_yaw_imu_for_guess = 0.0;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_prev_cloud_ || !prev_cloud_ || prev_cloud_->empty()) {
      prev_cloud_ = cloud;
      has_prev_cloud_ = true;
      last_lidar_.stamp = stamp;
      last_lidar_.valid = false;
      last_lidar_.converged = false;
      last_lidar_.degeneracy = DegeneracyInfo{};
      last_gicp_guess_ = Eigen::Matrix4f::Identity();
      next_icp_force_full_guess_ = false;
      last_icp_guess_mode_ = "identity";
      next_icp_guess_mode_ = lidar_guess_use_imu_yaw_only_ ? "yaw_only" : "full";
      degeneracy_state_ = false;
      has_critical_latch_until_ = false;
      critical_clear_streak_ = 0;
      weak_observation_streak_ = 0;
      speed_mismatch_streak_ = 0;
      if (!has_odom_pose_) {
        odom_yaw_ = yaw_imu_;
        has_odom_pose_ = true;
      }
      last_lidar_yaw_imu_ = yaw_imu_;
      has_last_lidar_yaw_imu_ = true;
      return;
    }
    prev = prev_cloud_;
    guess = last_gicp_guess_;
    prev_lidar_stamp = last_lidar_.stamp;
    force_full_guess = next_icp_force_full_guess_;
    prev_interval_degenerate = last_lidar_.degeneracy.degenerate;
    prev_interval_scan_rejected = last_lidar_.degeneracy.scan_rejected;
    has_last_lidar_yaw_imu_for_guess = has_last_lidar_yaw_imu_;
    last_lidar_yaw_imu_for_guess = last_lidar_yaw_imu_;
  }

  double imu_delta_yaw_meas = 0.0;
  bool has_imu_delta_yaw = computeImuDeltaYaw(prev_lidar_stamp, stamp, imu_delta_yaw_meas);
  if (!has_imu_delta_yaw && has_last_lidar_yaw_imu_for_guess) {
    imu_delta_yaw_meas = normalizeYaw(yaw_imu_ - last_lidar_yaw_imu_for_guess);
    has_imu_delta_yaw = true;
  }

  double wheel_distance_guess = 0.0;
  const bool has_wheel_distance = use_wheel_speed_ && computeWheelDistance(prev_lidar_stamp, stamp, wheel_distance_guess);
  const bool has_current_prior_guess = has_imu_delta_yaw && has_wheel_distance;
  const Eigen::Vector3d delta_prior_guess =
    has_current_prior_guess ? arcMotionPrior(wheel_distance_guess, imu_delta_yaw_meas) : Eigen::Vector3d::Zero();

  if (lidar_guess_use_imu_yaw_only_ && has_last_lidar_yaw_imu_for_guess && !force_full_guess) {
    guess = Eigen::Matrix4f::Identity();
    guess.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(static_cast<float>(imu_delta_yaw_meas), Eigen::Vector3f::UnitZ()).toRotationMatrix();
    guess_mode_used = "yaw_only";
  } else if (force_full_guess && wheel_degeneracy_full_guess_use_current_prior_ && has_current_prior_guess) {
    guess = se2DeltaToScanGuess(delta_prior_guess, T_base_scan, T_scan_base);
    if (prev_interval_degenerate || prev_interval_scan_rejected) {
      guess_mode_used = "full_wheel_imu_prior_due_to_previous_critical_or_reject";
    } else {
      guess_mode_used = "full_wheel_imu_prior";
    }
  } else if (force_full_guess) {
    if (prev_interval_degenerate || prev_interval_scan_rejected) {
      guess_mode_used = "full_due_to_previous_critical_or_reject";
    } else {
      guess_mode_used = "full_due_to_previous_request";
    }
  } else if (lidar_guess_use_imu_yaw_only_ && !has_last_lidar_yaw_imu_for_guess) {
    guess_mode_used = "full_no_imu_delta";
  } else {
    guess_mode_used = "full";
  }

  if (!prev || prev->size() < 50 || cloud->size() < 50) {
    std::lock_guard<std::mutex> lk(mtx_);
    prev_cloud_ = cloud;
    has_prev_cloud_ = true;
    last_lidar_.stamp = stamp;
    last_lidar_.valid = false;
    last_lidar_.converged = false;
    last_lidar_.degeneracy = DegeneracyInfo{};
    next_icp_force_full_guess_ = false;
    last_icp_guess_mode_ = "insufficient_points";
    next_icp_guess_mode_ = lidar_guess_use_imu_yaw_only_ ? "yaw_only" : "full";
    degeneracy_state_ = false;
    has_critical_latch_until_ = false;
    critical_clear_streak_ = 0;
    weak_observation_streak_ = 0;
    speed_mismatch_streak_ = 0;
    if (!has_odom_pose_) {
      odom_yaw_ = yaw_imu_;
      has_odom_pose_ = true;
    }
    last_lidar_yaw_imu_ = yaw_imu_;
    has_last_lidar_yaw_imu_ = true;
    return;
  }

  const bool use_small_gicp =
    (lidar_backend_ == "SMALL_GICP" || lidar_backend_ == "small_gicp" ||
     lidar_backend_ == "VGICP" || lidar_backend_ == "GICP");

  pcl::PointCloud<pcl::PointXYZ> aligned;
  bool converged = false;
  double fitness = std::numeric_limits<double>::infinity();
  Eigen::Matrix4f T_prev_curr_scan = guess;
  Matrix6d hessian_scan = Matrix6d::Zero();
  bool has_hessian = false;

  if (use_small_gicp) {
    small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg;
    reg.setNumThreads(lidar_num_threads_);
    reg.setRegistrationType(lidar_registration_type_);
    reg.setMaxCorrespondenceDistance(gicp_max_corr_dist_m_);
    reg.setMaximumIterations(gicp_max_iterations_);
    reg.setTransformationEpsilon(gicp_trans_eps_);
    reg.setRotationEpsilon(gicp_rot_eps_);
    reg.setCorrespondenceRandomness(gicp_corr_randomness_);
    reg.setVoxelResolution(gicp_voxel_resolution_);
    reg.setInputTarget(prev);
    reg.setInputSource(cloud);
    reg.align(aligned, guess);
    converged = reg.hasConverged();
    fitness = reg.getFitnessScore(gicp_max_corr_dist_m_);
    const auto res = reg.getRegistrationResult();
    T_prev_curr_scan = res.T_target_source.matrix().cast<float>();
    hessian_scan = res.H;
    has_hessian = res.H.allFinite();
  } else {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaxCorrespondenceDistance(static_cast<float>(gicp_max_corr_dist_m_));
    gicp.setMaximumIterations(gicp_max_iterations_);
    gicp.setTransformationEpsilon(static_cast<float>(gicp_trans_eps_));
    gicp.setRotationEpsilon(static_cast<float>(gicp_rot_eps_));
    gicp.setCorrespondenceRandomness(gicp_corr_randomness_);
    gicp.setMaximumOptimizerIterations(20);
    gicp.setInputTarget(prev);
    gicp.setInputSource(cloud);
    gicp.align(aligned, guess);

    converged = gicp.hasConverged();
    fitness = std::isfinite(gicp.getFitnessScore()) ? gicp.getFitnessScore() : std::numeric_limits<double>::infinity();
    T_prev_curr_scan = gicp.getFinalTransformation();
  }

  // T_target_source maps current(source) into previous(target) scan frame.
  const Eigen::Matrix4f T_prev_curr_base = T_base_scan * T_prev_curr_scan * T_scan_base;
  const Eigen::Matrix3d R = T_prev_curr_base.block<3, 3>(0, 0).cast<double>();
  const Eigen::Vector3d t = T_prev_curr_base.block<3, 1>(0, 3).cast<double>();

  const double raw_dx = t.x();
  const double raw_dy = t.y();
  const double dz = t.z();
  const double raw_dyaw = yawFromRot(R);

  double dt = 0.0;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    dt = (stamp - last_lidar_.stamp).seconds();
  }
  if (!std::isfinite(dt) || dt <= 1e-6) dt = 0.0;

  bool stationary_now = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    updateStopState(stamp);
    stationary_now = is_stopped_;
  }

  DegeneracyInfo degeneracy;
  degeneracy.has_hessian = has_hessian;
  degeneracy.stationary_now = stationary_now;
  Eigen::Vector3d delta_scan(raw_dx, raw_dy, raw_dyaw);
  Eigen::Vector3d delta_used = delta_scan;
  Eigen::Vector3d delta_prior = Eigen::Vector3d::Zero();
  Matrix3d weak_projector_eval = Matrix3d::Zero();
  bool detection_enabled_for_frame = false;

  const bool detection_pipeline_enabled = wheel_degeneracy_enable_ && use_wheel_speed_;
  const bool wheel_prior_available = use_wheel_speed_ && has_imu_delta_yaw && has_wheel_distance;
  if (wheel_prior_available) {
    degeneracy.wheel_prior_available = true;
    degeneracy.wheel_distance = wheel_distance_guess;
    delta_prior = arcMotionPrior(wheel_distance_guess, imu_delta_yaw_meas);
    degeneracy.prior_dx = delta_prior.x();
    degeneracy.prior_dy = delta_prior.y();
    degeneracy.prior_dyaw = normalizeYaw(delta_prior.z());

    if (dt > 1e-6) {
      degeneracy.scan_speed_mps = std::sqrt(raw_dx * raw_dx + raw_dy * raw_dy) / dt;
      degeneracy.wheel_speed_mps = std::fabs(wheel_distance_guess) / dt;
      degeneracy.speed_diff_mps = std::fabs(degeneracy.scan_speed_mps - degeneracy.wheel_speed_mps);
      degeneracy.speed_mismatch =
        degeneracy.speed_diff_mps > std::max(0.0, wheel_degeneracy_scan_wheel_speed_diff_thr_mps_);
    }
  }

  const double bad_fit_fitness_thr =
    (wheel_degeneracy_bad_fit_fitness_thr_ > 0.0) ? wheel_degeneracy_bad_fit_fitness_thr_ : gicp_fitness_max_;
  degeneracy.bad_fit = (!converged) || !std::isfinite(fitness) || (fitness > bad_fit_fitness_thr) || (dt <= 0.0);
  degeneracy.scan_rejected = detection_pipeline_enabled && wheel_prior_available && degeneracy.bad_fit;

  if (wheel_degeneracy_enable_ && has_hessian && wheel_prior_available) {
    const Matrix6d tmp = adjointRotationFirst(T_scan_base.cast<double>()).transpose() * hessian_scan *
                         adjointRotationFirst(T_scan_base.cast<double>());
    const Matrix6d H_base = 0.5 * (tmp + tmp.transpose());

    Matrix3d info_se2 = Matrix3d::Zero();
    if (reduceHessianToSe2Info(H_base, info_se2)) {
      degeneracy.has_hessian = true;
      degeneracy.detection_enabled = true;
      detection_enabled_for_frame = true;

      Eigen::Vector3d eigvals_norm = Eigen::Vector3d::Zero();
      int weak_count = 0;
      double score = 0.0;
      std::tie(weak_projector_eval, eigvals_norm, weak_count, score) = computeWeakProjector(
        info_se2,
        wheel_degeneracy_yaw_metric_m_,
        wheel_degeneracy_rel_eigenvalue_thr_,
        wheel_degeneracy_abs_eigenvalue_thr_,
        1.0);

      degeneracy.weak_direction_count = weak_count;
      degeneracy.score = score;
      degeneracy.eig_min = eigvals_norm(0);
      degeneracy.eig_mid = eigvals_norm(1);
      degeneracy.eig_max = eigvals_norm(2);
      degeneracy.assist_candidate = false;
      degeneracy.weak_observation = (weak_count > 0 && score > wheel_degeneracy_score_thr_);

      if (weak_count > 0) {
        const Eigen::Vector3d weak_innovation = weak_projector_eval * (delta_scan - delta_prior);
        degeneracy.prior_conflict_trans = weak_innovation.head<2>().norm();
        degeneracy.prior_conflict_yaw = std::fabs(weak_innovation.z());
        degeneracy.prior_conflict_metric = se2MetricNorm(weak_innovation, wheel_degeneracy_yaw_metric_m_);
        degeneracy.prior_conflict =
          (degeneracy.prior_conflict_trans > wheel_degeneracy_prior_conflict_trans_thr_m_) ||
          (degeneracy.prior_conflict_yaw > wheel_degeneracy_prior_conflict_yaw_thr_rad_) ||
          (degeneracy.prior_conflict_metric > wheel_degeneracy_prior_conflict_metric_thr_m_);

        const Eigen::Vector3d weak_stationary_delta = weak_projector_eval * delta_scan;
        degeneracy.stationary_drift_metric =
          se2MetricNorm(weak_stationary_delta, wheel_degeneracy_yaw_metric_m_);
        degeneracy.stationary_drift = stationary_now && (
          weak_stationary_delta.head<2>().norm() > wheel_degeneracy_stationary_trans_thr_m_ ||
          std::fabs(weak_stationary_delta.z()) > wheel_degeneracy_stationary_yaw_thr_rad_);
      }
    }
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);

    weak_observation_streak_ = detection_enabled_for_frame && degeneracy.weak_observation ?
      (weak_observation_streak_ + 1) : 0;
    speed_mismatch_streak_ =
      (detection_pipeline_enabled && degeneracy.wheel_prior_available && degeneracy.speed_mismatch) ?
      (speed_mismatch_streak_ + 1) : 0;
    const bool persistent_speed_mismatch =
      speed_mismatch_streak_ >= kCriticalKeepSpeedMismatchStreak;

    if (!detection_pipeline_enabled) {
      degeneracy_state_ = false;
      has_critical_latch_until_ = false;
      critical_clear_streak_ = 0;
      speed_mismatch_streak_ = 0;
    } else {
      const bool critical_candidate =
        detection_enabled_for_frame &&
        degeneracy.weak_observation &&
        (degeneracy.prior_conflict || degeneracy.stationary_drift);

      if (degeneracy.scan_rejected || critical_candidate) {
        degeneracy_state_ = true;
        critical_clear_streak_ = 0;
        const double hold_sec = std::max(0.0, wheel_degeneracy_latch_hold_sec_);
        critical_latch_until_ = stamp + rclcpp::Duration::from_seconds(hold_sec);
        has_critical_latch_until_ = hold_sec > 0.0;
      } else if (degeneracy_state_) {
        const bool hold_active = has_critical_latch_until_ && (stamp <= critical_latch_until_);
        const bool keep_evidence =
          (detection_enabled_for_frame && (degeneracy.prior_conflict || degeneracy.stationary_drift)) ||
          persistent_speed_mismatch;
        if (hold_active || keep_evidence) {
          critical_clear_streak_ = 0;
        } else {
          ++critical_clear_streak_;
          if (critical_clear_streak_ >= std::max(1, wheel_degeneracy_latch_off_streak_thr_)) {
            degeneracy_state_ = false;
            has_critical_latch_until_ = false;
            critical_clear_streak_ = 0;
          }
        }
      } else {
        critical_clear_streak_ = 0;
      }
    }

    degeneracy.speed_mismatch_streak = speed_mismatch_streak_;
    degeneracy.assist_latched = false;
    degeneracy.degenerate = degeneracy_state_;
    if (degeneracy.scan_rejected) {
      degeneracy.pose_mode = kLidarPoseModeScanReject;
    } else if (degeneracy.degenerate) {
      degeneracy.pose_mode = kLidarPoseModeCritical;
    } else {
      degeneracy.pose_mode = kLidarPoseModeNormal;
    }
    degeneracy.assist_off_streak = 0;
    degeneracy.weak_observation_streak = weak_observation_streak_;
    degeneracy.bad_evidence_streak = 0;
    degeneracy.risk_ema = 0.0;
    degeneracy.low_obs_hold_remaining_sec = 0.0;
    degeneracy.critical_hold_remaining_sec =
      has_critical_latch_until_ ? std::max(0.0, (critical_latch_until_ - stamp).seconds()) : 0.0;
    degeneracy.critical_clear_streak = critical_clear_streak_;
  }

  const bool wheel_prior_ready_for_assist =
    degeneracy.wheel_prior_available &&
    !stationary_now &&
    (std::fabs(degeneracy.wheel_distance) >= wheel_degeneracy_min_wheel_dist_m_);

  const bool allow_strong_wheel_assist =
    wheel_prior_ready_for_assist && degeneracy.degenerate && !degeneracy.scan_rejected;

  const double assist_blend = allow_strong_wheel_assist ? clamp01(wheel_degeneracy_prior_blend_) : 0.0;
  degeneracy.assist_blend = assist_blend;

  if (degeneracy.scan_rejected && wheel_prior_ready_for_assist) {
    delta_used = delta_prior;
    delta_used(2) = normalizeYaw(delta_used(2));
    degeneracy.wheel_assisted = true;
    degeneracy.wheel_assisted_weak = false;
    degeneracy.wheel_assisted_strong = true;
    degeneracy.assist_blend = 1.0;
  } else if (assist_blend > 1e-9) {
    const Matrix3d correction_projector = assist_blend * weak_projector_eval;
    delta_used = fuseWeakDirections(delta_scan, delta_prior, correction_projector);
    if (delta_used.allFinite()) {
      delta_used(2) = normalizeYaw(delta_used(2));
      degeneracy.wheel_assisted = ((delta_used - delta_scan).norm() > 1e-6);
      degeneracy.wheel_assisted_strong = degeneracy.wheel_assisted;
      degeneracy.wheel_assisted_weak = false;
      if (!degeneracy.wheel_assisted) {
        degeneracy.assist_blend = 0.0;
      }
    } else {
      delta_used = delta_scan;
      degeneracy.wheel_assisted = false;
      degeneracy.wheel_assisted_weak = false;
      degeneracy.wheel_assisted_strong = false;
      degeneracy.assist_blend = 0.0;
    }
  }

  if (!delta_used.allFinite()) {
    delta_used = delta_scan;
    degeneracy.wheel_assisted = false;
    degeneracy.wheel_assisted_weak = false;
    degeneracy.wheel_assisted_strong = false;
    degeneracy.assist_blend = 0.0;
  }
  delta_used(2) = normalizeYaw(delta_used(2));

  Eigen::Matrix4f T_prev_curr_scan_guess = T_prev_curr_scan;
  if (degeneracy.wheel_assisted) {
    Eigen::Matrix4f T_prev_curr_base_guess = T_prev_curr_base;
    T_prev_curr_base_guess(0, 3) = static_cast<float>(delta_used.x());
    T_prev_curr_base_guess(1, 3) = static_cast<float>(delta_used.y());

    tf2::Matrix3x3 raw_tf(
      R(0, 0), R(0, 1), R(0, 2),
      R(1, 0), R(1, 1), R(1, 2),
      R(2, 0), R(2, 1), R(2, 2));
    double raw_roll = 0.0;
    double raw_pitch = 0.0;
    double raw_yaw_unused = 0.0;
    raw_tf.getRPY(raw_roll, raw_pitch, raw_yaw_unused);
    (void)raw_yaw_unused;

    tf2::Quaternion q_rpy;
    q_rpy.setRPY(raw_roll, raw_pitch, delta_used.z());
    q_rpy.normalize();
    Eigen::Quaternionf q_used(
      static_cast<float>(q_rpy.w()),
      static_cast<float>(q_rpy.x()),
      static_cast<float>(q_rpy.y()),
      static_cast<float>(q_rpy.z()));
    q_used.normalize();
    T_prev_curr_base_guess.block<3, 3>(0, 0) = q_used.toRotationMatrix();

    T_prev_curr_scan_guess = T_scan_base * T_prev_curr_base_guess * T_base_scan;
    if (!T_prev_curr_scan_guess.allFinite()) {
      T_prev_curr_scan_guess = T_prev_curr_scan;
    }
  }

  const bool deadreckon_fallback_used = degeneracy.scan_rejected && wheel_prior_ready_for_assist;

  LidarOdomSample out;
  out.stamp = stamp;
  out.dt = dt;
  out.valid =
    ((std::isfinite(fitness) && (fitness <= gicp_fitness_max_) && dt > 0.0 && !degeneracy.scan_rejected) ||
    deadreckon_fallback_used);
  out.converged = converged;
  out.fitness = fitness;
  out.raw_dx = raw_dx;
  out.raw_dy = raw_dy;
  out.raw_dyaw = raw_dyaw;
  out.dx = delta_used.x();
  out.dy = delta_used.y();
  out.dyaw = delta_used.z();
  if (dt > 1e-6) {
    out.raw_vx = out.raw_dx / dt;
    out.raw_vy = out.raw_dy / dt;
    out.raw_yaw_rate = out.raw_dyaw / dt;
    out.vx = out.dx / dt;
    out.vy = out.dy / dt;
    out.yaw_rate = out.dyaw / dt;
  }
  out.degeneracy = degeneracy;
  if (out.valid && dt > 1e-6) {
    if ((out.degeneracy.degenerate || out.degeneracy.scan_rejected) && out.degeneracy.wheel_prior_available) {
      out.v = std::fabs(out.degeneracy.wheel_distance) / dt;
    } else {
      out.v = std::sqrt(out.dx * out.dx + out.dy * out.dy + dz * dz) / dt;
    }
  }

  const bool force_full_guess_next =
    out.valid &&
    !out.degeneracy.stationary_now &&
    (out.degeneracy.degenerate || out.degeneracy.scan_rejected);
  out.degeneracy.force_full_guess_next = force_full_guess_next;
  std::string next_guess_mode = "full";
  if (lidar_guess_use_imu_yaw_only_) {
    if (force_full_guess_next) {
      next_guess_mode = out.degeneracy.scan_rejected ?
        "full_due_to_current_scan_reject" : "full_due_to_current_degeneracy";
    } else {
      next_guess_mode = "yaw_only";
    }
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_odom_pose_) {
      odom_yaw_ = yaw_imu_;
      has_odom_pose_ = true;
    }

    updateStopState(stamp);

    if (out.valid && lidar_pose_se2_enable_) {
      if (out.degeneracy.scan_rejected && out.degeneracy.wheel_prior_available) {
        const double dyaw_fused = has_imu_delta_yaw ? imu_delta_yaw_meas : out.dyaw;
        const double yaw_mid = odom_yaw_ + 0.5 * dyaw_fused;
        const double c = std::cos(yaw_mid);
        const double s = std::sin(yaw_mid);
        odom_x_ += c * out.dx - s * out.dy;
        odom_y_ += s * out.dx + c * out.dy;
        odom_yaw_ = normalizeYaw(odom_yaw_ + dyaw_fused);
      } else if (lidar_smoother_enable_ && has_imu_delta_yaw) {
        ScanFactor factor;
        factor.stamp_prev = last_lidar_.stamp;
        factor.stamp_curr = stamp;
        factor.dx = out.dx;
        factor.dy = out.dy;
        factor.dyaw_scan = out.dyaw;
        factor.dyaw_imu = imu_delta_yaw_meas;
        factor.fitness = out.fitness;
        factor.converged = out.converged;
        factor.stationary = is_stopped_;
        factor.wheel_assisted = out.degeneracy.wheel_assisted;
        updateMiniSmootherLocked(factor);
      } else {
        double dyaw_fused = has_imu_delta_yaw ? imu_delta_yaw_meas : out.dyaw;
        if (has_imu_delta_yaw && std::fabs(lidar_yaw_blend_imu_) > 1e-6) {
          const double b = std::max(0.0, std::min(1.0, lidar_yaw_blend_imu_));
          dyaw_fused = normalizeYaw((1.0 - b) * imu_delta_yaw_meas + b * out.dyaw);
        }
        const double yaw_mid = odom_yaw_ + 0.5 * dyaw_fused;
        const double c = std::cos(yaw_mid);
        const double s = std::sin(yaw_mid);
        odom_x_ += c * out.dx - s * out.dy;
        odom_y_ += s * out.dx + c * out.dy;
        odom_yaw_ = normalizeYaw(odom_yaw_ + dyaw_fused);
      }
    }

    const double motion_dist = std::sqrt(out.dx * out.dx + out.dy * out.dy);
    if (out.valid) {
      double step_xy = odom_cov_base_xy_step_;
      double step_yaw = odom_cov_base_yaw_step_;
      step_xy += odom_cov_xy_per_meter_ * motion_dist;
      step_yaw += odom_cov_yaw_per_rad_ * std::fabs(out.dyaw);
      if (std::isfinite(out.fitness) && out.fitness > 0.0) {
        step_xy += odom_cov_fitness_xy_scale_ * out.fitness;
        step_yaw += odom_cov_fitness_yaw_scale_ * out.fitness;
      }
      if (!out.converged) {
        step_xy *= 2.0;
        step_yaw *= 2.0;
      }
      if (out.degeneracy.weak_observation) {
        const double weak_scale = 1.0 + clamp01(out.degeneracy.score);
        step_xy *= weak_scale;
        step_yaw *= weak_scale;
      }
      if (out.degeneracy.wheel_assisted) {
        step_xy *= std::max(1.0, odom_cov_wheel_assist_scale_);
        step_yaw *= std::max(1.0, odom_cov_wheel_assist_scale_);
      }
      if (out.degeneracy.degenerate) {
        step_xy *= std::max(1.0, odom_cov_degenerate_scale_);
        step_yaw *= std::max(1.0, odom_cov_degenerate_scale_);
      }
      if (is_stopped_ && !out.degeneracy.stationary_drift) {
        step_xy *= 0.25;
        step_yaw *= 0.25;
      }
      odom_cov_total_xy_ += std::max(0.0, step_xy);
      odom_cov_total_yaw_ += std::max(0.0, step_yaw);
    } else {
      odom_cov_total_xy_ += std::max(0.0, odom_cov_invalid_xy_step_);
      odom_cov_total_yaw_ += std::max(0.0, odom_cov_invalid_yaw_step_);
    }

    last_lidar_yaw_imu_ = yaw_imu_;
    has_last_lidar_yaw_imu_ = true;
    last_lidar_ = out;
    prev_cloud_ = cloud;
    has_prev_cloud_ = true;
    last_gicp_guess_ = T_prev_curr_scan_guess;
    next_icp_force_full_guess_ = force_full_guess_next;
    last_icp_guess_mode_ = guess_mode_used;
    next_icp_guess_mode_ = next_guess_mode;
    last_integrate_stamp_ = stamp;
    has_last_integrate_ = true;
  }

  publishDegeneracyDebug(stamp, out, guess_mode_used, next_guess_mode);
}

bool GyroOdometerNode::computeAccVariance(const rclcpp::Time & nowt, double window_sec, double & out_var) const
{
  out_var = 1e9;
  if (imu_buf_.size() < 10) return false;

  const double t_now = nowt.seconds();

  std::vector<double> ax;
  std::vector<double> ay;
  std::vector<double> az;
  std::vector<double> gz;
  ax.reserve(64);
  ay.reserve(64);
  az.reserve(64);
  gz.reserve(64);

  for (int i = static_cast<int>(imu_buf_.size()) - 1; i >= 0; --i) {
    const double ti = imu_buf_[i].stamp.seconds();
    if ((t_now - ti) > window_sec) break;
    ax.push_back(imu_buf_[i].acc.x());
    ay.push_back(imu_buf_[i].acc.y());
    az.push_back(imu_buf_[i].acc.z());
    gz.push_back(imu_buf_[i].gyro_z);
  }
  if (gz.size() < 10) return false;

  auto variance = [](const std::vector<double> & v) -> double {
    if (v.size() < 2) return 1e9;
    double m = 0.0;
    for (double x : v) m += x;
    m /= static_cast<double>(v.size());
    double s = 0.0;
    for (double x : v) s += (x - m) * (x - m);
    return s / std::max(1.0, static_cast<double>(v.size() - 1));
  };

  out_var = variance(ax) + variance(ay) + variance(az);
  return true;
}

void GyroOdometerNode::updateStopState(const rclcpp::Time & nowt)
{
  if (!stop_enable_) {
    is_stopped_ = false;
    has_stop_candidate_since_ = false;
    return;
  }

  // Prefer speed if available (wheel or lidar)
  bool has_v = false;
  double v = 0.0;

  if (use_wheel_speed_ && has_wheel_) {
    const double age = std::fabs((nowt - last_wheel_.stamp).seconds());
    if (std::isfinite(age) && age <= wheel_speed_timeout_sec_) {
      v = last_wheel_.v_raw * wheel_speed_scale_;
      has_v = true;
    }
  }
  if (!has_v && lidar_odom_enable_ && last_lidar_.valid) {
    const double age = std::fabs((nowt - last_lidar_.stamp).seconds());
    if (std::isfinite(age) && age <= lidar_timeout_sec_) {
      v = last_lidar_.v;
      has_v = true;
    }
  }

  bool stop_candidate = false;
  if (has_v) {
    stop_candidate = (std::fabs(v) < stop_speed_thr_mps_);
  } else {
    // fallback: IMU-based stop detection
    double var_a = 1e9;
    const bool ok = computeAccVariance(nowt, 0.3, var_a);
    if (!ok) {
      stop_candidate = false;
    } else {
      // gyro mean over same window
      double gz_mean = 0.0;
      int n = 0;
      const double t_now = nowt.seconds();
      for (int i = static_cast<int>(imu_buf_.size()) - 1; i >= 0; --i) {
        const double ti = imu_buf_[i].stamp.seconds();
        if ((t_now - ti) > 0.3) break;
        gz_mean += imu_buf_[i].gyro_z;
        ++n;
      }
      if (n < 10) {
        stop_candidate = false;
      } else {
        gz_mean /= static_cast<double>(n);
        stop_candidate = (var_a < stop_acc_var_thr_) && (std::fabs(gz_mean) < stop_gyro_abs_thr_rad_s_);
      }
    }
  }

  // Hold time hysteresis
  if (stop_candidate) {
    if (!has_stop_candidate_since_) {
      stop_candidate_since_ = nowt;
      has_stop_candidate_since_ = true;
    }
    const double held = (nowt - stop_candidate_since_).seconds();
    is_stopped_ = (std::isfinite(held) && held >= stop_hold_sec_);
  } else {
    has_stop_candidate_since_ = false;
    is_stopped_ = false;
  }

  // When stopped, reset accel-based speed estimate.
  if (is_stopped_) {
    v_acc_est_ = 0.0;
    has_v_acc_ = true;
  }
}

void GyroOdometerNode::onPublishTimer()
{
  const rclcpp::Time nowt = now();

  nav_msgs::msg::Odometry odom_raw;
  nav_msgs::msg::Odometry odom_filtered;
  std_msgs::msg::Bool stopped;
  std_msgs::msg::Bool degeneracy_flag;
  std_msgs::msg::UInt8 pose_mode_msg;

  double yaw = 0.0;
  double bg = 0.0;
  bool has_yaw_rate_out = false;
  double yaw_rate_out = 0.0;
  bool imu_extrinsic_cached = false;
  std::string imu_frame_id;
  bool stopped_now = false;
  bool has_v_pred = false;
  double v_pred = 0.0;
  double v_raw = 0.0;
  double v_wheel = 0.0;
  double v_lidar = 0.0;
  bool has_wheel = false;
  bool has_lidar = false;
  double wheel_scale = 1.0;
  double odom_x = 0.0;
  double odom_y = 0.0;
  double odom_yaw = 0.0;
  double filtered_x = 0.0;
  double filtered_y = 0.0;
  double filtered_yaw = 0.0;
  bool have_filtered_pose = false;
  double raw_twist_x = 0.0;
  double raw_twist_y = 0.0;
  double raw_twist_yaw = 0.0;
  bool has_raw_twist = false;
  double filtered_twist_x = 0.0;
  double filtered_twist_y = 0.0;
  double filtered_twist_yaw = 0.0;
  bool has_filtered_twist = false;
  double odom_cov_xy_total = 0.0;
  double odom_cov_yaw_total = 0.0;
  LidarOdomSample lidar = {};
  std::string speed_source{"none"};
  std::string last_icp_guess_mode;
  std::string next_icp_guess_mode;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    updateStopState(nowt);
    stopped_now = is_stopped_;
    yaw = yaw_imu_;
    bg = bg_est_;
    imu_extrinsic_cached = has_imu_extrinsic_;
    imu_frame_id = imu_frame_id_;

    if (!imu_buf_.empty()) {
      yaw_rate_out = imu_buf_.back().gyro_z - bg_est_;
      has_yaw_rate_out = true;
    }
    wheel_scale = wheel_speed_scale_;
    odom_x = odom_x_;
    odom_y = odom_y_;
    odom_yaw = has_odom_pose_ ? odom_yaw_ : yaw_imu_;
    odom_cov_xy_total = odom_cov_total_xy_;
    odom_cov_yaw_total = odom_cov_total_yaw_;

    has_wheel = (use_wheel_speed_ && has_wheel_);
    if (has_wheel) {
      const double age = std::fabs((nowt - last_wheel_.stamp).seconds());
      if (std::isfinite(age) && age <= wheel_speed_timeout_sec_) {
        v_raw = last_wheel_.v_raw;
      } else {
        has_wheel = false;
      }
    }

    lidar = last_lidar_;
    last_icp_guess_mode = last_icp_guess_mode_;
    next_icp_guess_mode = next_icp_guess_mode_;
    has_lidar = (lidar_odom_enable_ && lidar.valid);
    if (has_lidar) {
      const double age = std::fabs((nowt - lidar.stamp).seconds());
      if (!(std::isfinite(age) && age <= lidar_timeout_sec_)) {
        has_lidar = false;
      }
    }
    if (has_lidar) {
      v_lidar = lidar.v;
    }

    bool has_wheel_speed = false;
    if (has_wheel) {
      double v_scaled = v_raw * wheel_speed_scale_;
      if (wheel_low_speed_enable_) {
        if (std::fabs(v_raw) < wheel_low_speed_deadband_mps_ && !stopped_now && has_v_acc_) {
          double ax_abs = 0.0;
          if (!imu_buf_.empty()) {
            ax_abs = std::fabs(imu_buf_.back().acc.x());
          }
          if (ax_abs > wheel_low_speed_acc_thr_mps2_) {
            const double corr = std::max(-wheel_low_speed_max_corr_mps_,
              std::min(wheel_low_speed_max_corr_mps_, (v_acc_est_ - v_scaled)));
            v_scaled = v_scaled + wheel_low_speed_blend_ * corr;
          }
        }
      }
      v_wheel = v_scaled;
      has_wheel_speed = true;
    }

    const bool wheel_speed_override_active =
      lidar.degeneracy.scan_rejected ||
      lidar.degeneracy.degenerate ||
      lidar.degeneracy.wheel_assisted_strong;

    if (has_lidar && !(use_wheel_speed_ && has_wheel_speed && wheel_speed_override_active)) {
      v_pred = v_lidar;
      has_v_pred = true;
      speed_source = "lidar";
    } else if (has_wheel_speed) {
      v_pred = v_wheel;
      has_v_pred = true;
      speed_source = "wheel";
    } else {
      v_pred = 0.0;
      has_v_pred = false;
      speed_source = "none";
    }

    if (!has_last_integrate_) {
      last_integrate_stamp_ = nowt;
      has_last_integrate_ = true;
    }

    double pred_x = odom_x_;
    double pred_y = odom_y_;
    double pred_yaw = has_odom_pose_ ? odom_yaw_ : yaw_imu_;
    const double dt_pred = (nowt - last_integrate_stamp_).seconds();
    if (std::isfinite(dt_pred) && dt_pred > 0.0 && dt_pred < 1.0 && has_v_pred) {
      double imu_delta = 0.0;
      if (has_last_lidar_yaw_imu_) {
        imu_delta = normalizeYaw(yaw_imu_ - last_lidar_yaw_imu_);
      }
      pred_yaw = normalizeYaw((has_odom_pose_ ? odom_yaw_ : yaw_imu_) + imu_delta);
      pred_x = odom_x_ + std::cos(pred_yaw) * v_pred * dt_pred;
      pred_y = odom_y_ + std::sin(pred_yaw) * v_pred * dt_pred;
    }

    const bool use_deadreckon_fallback = (!lidar_odom_enable_) || !has_lidar;
    if (use_deadreckon_fallback && std::isfinite(dt_pred) && dt_pred > 0.0 && dt_pred < 1.0 && has_v_pred) {
      if (!has_odom_pose_) {
        odom_yaw_ = yaw_imu_;
        has_odom_pose_ = true;
      }
      odom_x_ = pred_x;
      odom_y_ = pred_y;
      odom_yaw_ = pred_yaw;
      odom_cov_total_xy_ += std::max(0.0, odom_cov_deadreckon_xy_per_sec_) * dt_pred;
      odom_cov_total_yaw_ += std::max(0.0, odom_cov_deadreckon_yaw_per_sec_) * dt_pred;
      odom_cov_xy_total = odom_cov_total_xy_;
      odom_cov_yaw_total = odom_cov_total_yaw_;
      last_integrate_stamp_ = nowt;
    }

    odom_x = pred_x;
    odom_y = pred_y;
    odom_yaw = pred_yaw;

    if (has_lidar && std::isfinite(lidar.dt) && lidar.dt > 1e-6) {
      raw_twist_x = lidar.vx;
      raw_twist_y = lidar.vy;
      raw_twist_yaw = has_yaw_rate_out ? yaw_rate_out : lidar.yaw_rate;
      has_raw_twist = true;
    } else if (has_v_pred || has_yaw_rate_out) {
      raw_twist_x = has_v_pred ? v_pred : 0.0;
      raw_twist_y = 0.0;
      raw_twist_yaw = has_yaw_rate_out ? yaw_rate_out : 0.0;
      has_raw_twist = has_v_pred || has_yaw_rate_out;
    } else {
      raw_twist_x = 0.0;
      raw_twist_y = 0.0;
      raw_twist_yaw = 0.0;
      has_raw_twist = false;
    }

    if (out_filtered_odom_enable_ && !out_filtered_odom_topic_.empty() && out_filtered_odom_topic_ != out_odom_topic_) {
      const double dt_filtered = has_filtered_publish_state_ ? (nowt - last_filtered_publish_stamp_).seconds() : 0.0;
      const bool reset_filtered =
        !has_filtered_publish_state_ ||
        !std::isfinite(dt_filtered) ||
        dt_filtered <= 1e-4 ||
        dt_filtered > std::max(0.1, filtered_odom_reset_gap_sec_);

      if (reset_filtered) {
        filtered_pub_x_ = odom_x;
        filtered_pub_y_ = odom_y;
        filtered_pub_yaw_ = odom_yaw;
        filtered_pub_dx_ = 0.0;
        filtered_pub_dy_ = 0.0;
        filtered_pub_dyaw_ = 0.0;
        if (filtered_odom_zero_when_stopped_ && stopped_now) {
          filtered_pub_vx_ = 0.0;
          filtered_pub_vy_ = 0.0;
          filtered_pub_yaw_rate_ = 0.0;
        } else {
          filtered_pub_vx_ = has_raw_twist ? raw_twist_x : 0.0;
          filtered_pub_vy_ = has_raw_twist ? raw_twist_y : 0.0;
          filtered_pub_yaw_rate_ = has_raw_twist ? raw_twist_yaw : (has_yaw_rate_out ? yaw_rate_out : 0.0);
        }
        has_filtered_publish_state_ = true;
      } else if (filtered_odom_zero_when_stopped_ && stopped_now) {
        filtered_pub_dx_ = 0.0;
        filtered_pub_dy_ = 0.0;
        filtered_pub_dyaw_ = 0.0;
        filtered_pub_vx_ = 0.0;
        filtered_pub_vy_ = 0.0;
        filtered_pub_yaw_rate_ = 0.0;
      } else {
        const double target_vx = has_raw_twist ? raw_twist_x : 0.0;
        const double target_vy = has_raw_twist ? raw_twist_y : 0.0;
        const double target_w = has_raw_twist ? raw_twist_yaw : (has_yaw_rate_out ? yaw_rate_out : 0.0);

        const double alpha = clamp01(filtered_odom_lowpass_alpha_);
        double cmd_vx = alpha * filtered_pub_vx_ + (1.0 - alpha) * target_vx;
        double cmd_vy = alpha * filtered_pub_vy_ + (1.0 - alpha) * target_vy;
        double cmd_w = alpha * filtered_pub_yaw_rate_ + (1.0 - alpha) * target_w;

        const double vx_step = std::max(0.0, filtered_odom_linear_rate_limit_mps2_) * dt_filtered;
        const double vy_step = std::max(0.0, filtered_odom_lateral_rate_limit_mps2_) * dt_filtered;
        const double w_step = std::max(0.0, filtered_odom_yaw_rate_limit_radps2_) * dt_filtered;

        cmd_vx = filtered_pub_vx_ + std::max(-vx_step, std::min(vx_step, cmd_vx - filtered_pub_vx_));
        cmd_vy = filtered_pub_vy_ + std::max(-vy_step, std::min(vy_step, cmd_vy - filtered_pub_vy_));
        cmd_w = filtered_pub_yaw_rate_ + std::max(-w_step, std::min(w_step, cmd_w - filtered_pub_yaw_rate_));

        filtered_pub_dx_ = cmd_vx * dt_filtered;
        filtered_pub_dy_ = cmd_vy * dt_filtered;
        filtered_pub_dyaw_ = cmd_w * dt_filtered;
        integrateSe2Delta(filtered_pub_x_, filtered_pub_y_, filtered_pub_yaw_,
          Eigen::Vector3d(filtered_pub_dx_, filtered_pub_dy_, filtered_pub_dyaw_));
        filtered_pub_vx_ = cmd_vx;
        filtered_pub_vy_ = cmd_vy;
        filtered_pub_yaw_rate_ = cmd_w;
      }

      last_filtered_publish_stamp_ = nowt;
      filtered_x = filtered_pub_x_;
      filtered_y = filtered_pub_y_;
      filtered_yaw = filtered_pub_yaw_;
      filtered_twist_x = filtered_pub_vx_;
      filtered_twist_y = filtered_pub_vy_;
      filtered_twist_yaw = filtered_pub_yaw_rate_;
      has_filtered_twist = has_filtered_publish_state_;
      have_filtered_pose = has_filtered_publish_state_;
    }
  }

  auto fillOdom = [&](nav_msgs::msg::Odometry & odom_msg,
                      double x, double y, double yaw_angle,
                      double vx, double vy, double wz,
                      bool has_twist) {
    odom_msg.header.stamp = nowt;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    const auto q = quatFromYaw(yaw_angle);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
    odom_msg.pose.covariance[0] = std::max(0.0, odom_cov_xy_total);
    odom_msg.pose.covariance[7] = std::max(0.0, odom_cov_xy_total);
    odom_msg.pose.covariance[14] = 1.0e6;
    odom_msg.pose.covariance[21] = 1.0e6;
    odom_msg.pose.covariance[28] = 1.0e6;
    odom_msg.pose.covariance[35] = std::max(0.0, odom_cov_yaw_total);

    std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
    odom_msg.twist.covariance[0] = has_twist ? std::max(1.0e-4, 0.25 * odom_cov_xy_total) : 1.0e6;
    odom_msg.twist.covariance[7] = has_twist ? std::max(1.0e-4, 0.25 * odom_cov_xy_total) : 1.0e6;
    odom_msg.twist.covariance[14] = 1.0e6;
    odom_msg.twist.covariance[21] = 1.0e6;
    odom_msg.twist.covariance[28] = 1.0e6;
    odom_msg.twist.covariance[35] = has_twist ? std::max(1.0e-5, 0.25 * odom_cov_yaw_total) : 1.0e6;

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = wz;
  };

  fillOdom(odom_raw, odom_x, odom_y, odom_yaw, raw_twist_x, raw_twist_y, raw_twist_yaw, has_raw_twist);
  pub_odom_raw_->publish(odom_raw);

  if (pub_odom_filtered_ && have_filtered_pose) {
    fillOdom(
      odom_filtered, filtered_x, filtered_y, filtered_yaw,
      filtered_twist_x, filtered_twist_y, filtered_twist_yaw, has_filtered_twist);
    pub_odom_filtered_->publish(odom_filtered);
  }

  stopped.data = stopped_now;
  pub_stopped_->publish(stopped);

  if (pub_degenerate_) {
    degeneracy_flag.data = lidar.degeneracy.degenerate;
    pub_degenerate_->publish(degeneracy_flag);
  }
  if (pub_pose_mode_) {
    pose_mode_msg.data = lidar.degeneracy.pose_mode;
    pub_pose_mode_->publish(pose_mode_msg);
  }

  {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = nowt;
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (lidar_odom_enable_ && !has_lidar) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    if (lidar.degeneracy.degenerate) {
      st.level = std::max<uint8_t>(st.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    }
    st.name = "localization/gyro_odometer";
    if (lidar.degeneracy.scan_rejected) {
      st.message = stopped_now ? "scan rejected while stationary" : "running with scan rejection fallback";
    } else if (lidar.degeneracy.degenerate) {
      st.message = stopped_now ? "critical degeneracy while stationary" : "running with latched critical degeneracy";
    } else if (stopped_now) {
      st.message = "stopped";
    } else if (lidar_odom_enable_ && !has_lidar) {
      st.message = "running dead-reckoning fallback";
    } else {
      st.message = "running";
    }
    st.hardware_id = "none";

    auto add = [&](const std::string & k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = v;
      st.values.push_back(kv);
    };

    add("use_wheel_speed", use_wheel_speed_ ? "true" : "false");
    add("lidar_odom_enabled", lidar_odom_enable_ ? "true" : "false");
    add("yaw_imu", std::to_string(yaw));
    add("bg_est", std::to_string(bg));
    add("yaw_rate_out", std::to_string(has_yaw_rate_out ? yaw_rate_out : 0.0));
    add("v_pred", std::to_string(v_pred));
    add("has_v_pred", has_v_pred ? "true" : "false");
    add("prediction_speed_source", speed_source);
    add("raw_twist_x", std::to_string(raw_twist_x));
    add("raw_twist_y", std::to_string(raw_twist_y));
    add("raw_twist_yaw", std::to_string(raw_twist_yaw));
    add("filtered_twist_x", std::to_string(filtered_twist_x));
    add("filtered_twist_y", std::to_string(filtered_twist_y));
    add("filtered_twist_yaw", std::to_string(filtered_twist_yaw));
    add("odom_cov_xy_total", std::to_string(odom_cov_xy_total));
    add("odom_cov_yaw_total", std::to_string(odom_cov_yaw_total));
    add("raw_odom_topic", out_odom_topic_);
    add("filtered_odom_topic", pub_odom_filtered_ ? out_filtered_odom_topic_ : std::string("disabled"));
    add("lidar_pose_mode_topic", pub_pose_mode_ ? out_pose_mode_topic_ : std::string("disabled"));

    add("imu_corrected.enable", imu_corrected_enable_ ? "true" : "false");
    add("imu_corrected.apply_tf", imu_corrected_apply_tf_ ? "true" : "false");
    add("imu_extrinsic_cached", imu_extrinsic_cached ? "true" : "false");
    add("imu_frame", imu_extrinsic_cached ? imu_frame_id : std::string(""));

    if (use_wheel_speed_) {
      add("wheel_scale", std::to_string(wheel_scale));
      add("wheel_raw", std::to_string(v_raw));
      add("wheel_v_scaled", std::to_string(v_wheel));
      add("wheel_assist_enabled", wheel_degeneracy_enable_ ? "true" : "false");
    }
    if (lidar_odom_enable_) {
      add("icp_guess_mode_last", last_icp_guess_mode);
      add("icp_guess_mode_next", next_icp_guess_mode);
      add("degeneracy_debug_pub_enabled", wheel_degeneracy_debug_pub_enable_ ? "true" : "false");
      add("degeneracy_debug_topic", wheel_degeneracy_debug_pub_enable_ ? wheel_degeneracy_debug_topic_ : std::string(""));
      add("lidar_valid", lidar.valid ? "true" : "false");
      add("lidar_converged", lidar.converged ? "true" : "false");
      add("lidar_fitness", std::to_string(lidar.fitness));
      add("lidar_dx", std::to_string(lidar.dx));
      add("lidar_dy", std::to_string(lidar.dy));
      add("lidar_dyaw", std::to_string(lidar.dyaw));
      add("lidar_raw_dx", std::to_string(lidar.raw_dx));
      add("lidar_raw_dy", std::to_string(lidar.raw_dy));
      add("lidar_raw_dyaw", std::to_string(lidar.raw_dyaw));
      add("lidar_v", std::to_string(lidar.v));
      add("lidar_has_hessian", lidar.degeneracy.has_hessian ? "true" : "false");
      add("degeneracy_detection_enabled", lidar.degeneracy.detection_enabled ? "true" : "false");
      add("lidar_weak_observation", lidar.degeneracy.weak_observation ? "true" : "false");
      add("lidar_assist_candidate", lidar.degeneracy.assist_candidate ? "true" : "false");
      add("lidar_pose_mode", std::to_string(static_cast<int>(lidar.degeneracy.pose_mode)));
      add("lidar_pose_mode_name", lidarPoseModeToString(lidar.degeneracy.pose_mode));
      add("lidar_degenerate", lidar.degeneracy.degenerate ? "true" : "false");
      add("scan_rejected", lidar.degeneracy.scan_rejected ? "true" : "false");
      add("speed_mismatch", lidar.degeneracy.speed_mismatch ? "true" : "false");
      add("speed_mismatch_streak", std::to_string(lidar.degeneracy.speed_mismatch_streak));
      add("critical_clear_streak", std::to_string(lidar.degeneracy.critical_clear_streak));
      add("critical_hold_remaining_sec", std::to_string(lidar.degeneracy.critical_hold_remaining_sec));
      add("wheel_assist_active", lidar.degeneracy.wheel_assisted ? "true" : "false");
      add("wheel_assist_strong", lidar.degeneracy.wheel_assisted_strong ? "true" : "false");
      add("wheel_assist_blend", std::to_string(lidar.degeneracy.assist_blend));
      add("wheel_prior_available", lidar.degeneracy.wheel_prior_available ? "true" : "false");
      add("stationary_now", lidar.degeneracy.stationary_now ? "true" : "false");
      add("stationary_drift", lidar.degeneracy.stationary_drift ? "true" : "false");
      add("prior_conflict", lidar.degeneracy.prior_conflict ? "true" : "false");
      add("bad_fit", lidar.degeneracy.bad_fit ? "true" : "false");
      add("force_full_guess_next", lidar.degeneracy.force_full_guess_next ? "true" : "false");
      add("degeneracy_score", std::to_string(lidar.degeneracy.score));
      add("weak_direction_count", std::to_string(lidar.degeneracy.weak_direction_count));
      add("hessian_eig_min", std::to_string(lidar.degeneracy.eig_min));
      add("hessian_eig_mid", std::to_string(lidar.degeneracy.eig_mid));
      add("hessian_eig_max", std::to_string(lidar.degeneracy.eig_max));
      add("wheel_distance", std::to_string(lidar.degeneracy.wheel_distance));
      add("wheel_prior_dx", std::to_string(lidar.degeneracy.prior_dx));
      add("wheel_prior_dy", std::to_string(lidar.degeneracy.prior_dy));
      add("wheel_prior_dyaw", std::to_string(lidar.degeneracy.prior_dyaw));
      add("prior_conflict_metric", std::to_string(lidar.degeneracy.prior_conflict_metric));
      add("prior_conflict_trans", std::to_string(lidar.degeneracy.prior_conflict_trans));
      add("prior_conflict_yaw", std::to_string(lidar.degeneracy.prior_conflict_yaw));
      add("stationary_drift_metric", std::to_string(lidar.degeneracy.stationary_drift_metric));
      add("scan_speed_mps", std::to_string(lidar.degeneracy.scan_speed_mps));
      add("wheel_speed_mps", std::to_string(lidar.degeneracy.wheel_speed_mps));
      add("speed_diff_mps", std::to_string(lidar.degeneracy.speed_diff_mps));
    }

    arr.status.push_back(st);
    pub_diag_->publish(arr);
  }
}

void GyroOdometerNode::publishDegeneracyDebug(
  const rclcpp::Time & stamp, const LidarOdomSample & sample, const std::string & guess_mode_used,
  const std::string & next_guess_mode)
{
  if (!pub_degeneracy_debug_) {
    return;
  }

  std_msgs::msg::String msg;
  std::ostringstream oss;
  oss << "stamp: " << formatDouble(stamp.seconds(), 6) << "\n";
  oss << "guess_mode_used: " << guess_mode_used << "\n";
  oss << "next_guess_mode: " << next_guess_mode << "\n";
  oss << "lidar_valid: " << boolString(sample.valid) << "\n";
  oss << "lidar_converged: " << boolString(sample.converged) << "\n";
  oss << "lidar_fitness: " << formatDouble(sample.fitness) << "\n";
  oss << "degeneracy_detection_enabled: " << boolString(sample.degeneracy.detection_enabled) << "\n";
  oss << "lidar_has_hessian: " << boolString(sample.degeneracy.has_hessian) << "\n";
  oss << "lidar_weak_observation: " << boolString(sample.degeneracy.weak_observation) << "\n";
  oss << "lidar_assist_candidate: " << boolString(sample.degeneracy.assist_candidate) << "\n";
  oss << "lidar_pose_mode: " << static_cast<int>(sample.degeneracy.pose_mode) << "\n";
  oss << "lidar_pose_mode_name: " << lidarPoseModeToString(sample.degeneracy.pose_mode) << "\n";
  oss << "lidar_degenerate: " << boolString(sample.degeneracy.degenerate) << "\n";
  oss << "scan_rejected: " << boolString(sample.degeneracy.scan_rejected) << "\n";
  oss << "speed_mismatch: " << boolString(sample.degeneracy.speed_mismatch) << "\n";
  oss << "speed_mismatch_streak: " << sample.degeneracy.speed_mismatch_streak << "\n";
  oss << "critical_clear_streak: " << sample.degeneracy.critical_clear_streak << "\n";
  oss << "critical_hold_remaining_sec: " << formatDouble(sample.degeneracy.critical_hold_remaining_sec) << "\n";
  oss << "wheel_prior_available: " << boolString(sample.degeneracy.wheel_prior_available) << "\n";
  oss << "wheel_assist_active: " << boolString(sample.degeneracy.wheel_assisted) << "\n";
  oss << "wheel_assist_strong: " << boolString(sample.degeneracy.wheel_assisted_strong) << "\n";
  oss << "wheel_assist_blend: " << formatDouble(sample.degeneracy.assist_blend) << "\n";
  oss << "prior_conflict: " << boolString(sample.degeneracy.prior_conflict) << "\n";
  oss << "stationary_now: " << boolString(sample.degeneracy.stationary_now) << "\n";
  oss << "stationary_drift: " << boolString(sample.degeneracy.stationary_drift) << "\n";
  oss << "bad_fit: " << boolString(sample.degeneracy.bad_fit) << "\n";
  oss << "force_full_guess_next: " << boolString(sample.degeneracy.force_full_guess_next) << "\n";
  oss << "weak_direction_count: " << sample.degeneracy.weak_direction_count << "\n";
  oss << "degeneracy_score: " << formatDouble(sample.degeneracy.score) << "\n";
  oss << "hessian_eig_min: " << formatDouble(sample.degeneracy.eig_min) << "\n";
  oss << "hessian_eig_mid: " << formatDouble(sample.degeneracy.eig_mid) << "\n";
  oss << "hessian_eig_max: " << formatDouble(sample.degeneracy.eig_max) << "\n";
  oss << "raw_dx: " << formatDouble(sample.raw_dx) << "\n";
  oss << "raw_dy: " << formatDouble(sample.raw_dy) << "\n";
  oss << "raw_dyaw: " << formatDouble(sample.raw_dyaw) << "\n";
  oss << "used_dx: " << formatDouble(sample.dx) << "\n";
  oss << "used_dy: " << formatDouble(sample.dy) << "\n";
  oss << "used_dyaw: " << formatDouble(sample.dyaw) << "\n";
  oss << "wheel_distance: " << formatDouble(sample.degeneracy.wheel_distance) << "\n";
  oss << "wheel_prior_dx: " << formatDouble(sample.degeneracy.prior_dx) << "\n";
  oss << "wheel_prior_dy: " << formatDouble(sample.degeneracy.prior_dy) << "\n";
  oss << "wheel_prior_dyaw: " << formatDouble(sample.degeneracy.prior_dyaw) << "\n";
  oss << "prior_conflict_metric: " << formatDouble(sample.degeneracy.prior_conflict_metric) << "\n";
  oss << "prior_conflict_trans: " << formatDouble(sample.degeneracy.prior_conflict_trans) << "\n";
  oss << "prior_conflict_yaw: " << formatDouble(sample.degeneracy.prior_conflict_yaw) << "\n";
  oss << "stationary_drift_metric: " << formatDouble(sample.degeneracy.stationary_drift_metric) << "\n";
  oss << "scan_speed_mps: " << formatDouble(sample.degeneracy.scan_speed_mps) << "\n";
  oss << "wheel_speed_mps: " << formatDouble(sample.degeneracy.wheel_speed_mps) << "\n";
  oss << "speed_diff_mps: " << formatDouble(sample.degeneracy.speed_diff_mps) << "\n";
  oss << "score_thr: " << formatDouble(wheel_degeneracy_score_thr_) << "\n";
  oss << "prior_blend_strong: " << formatDouble(wheel_degeneracy_prior_blend_) << "\n";
  oss << "rel_eigenvalue_thr: " << formatDouble(wheel_degeneracy_rel_eigenvalue_thr_) << "\n";
  oss << "abs_eigenvalue_thr: " << formatDouble(wheel_degeneracy_abs_eigenvalue_thr_) << "\n";
  oss << "yaw_metric_m: " << formatDouble(wheel_degeneracy_yaw_metric_m_) << "\n";
  oss << "critical_hold_sec: " << formatDouble(wheel_degeneracy_latch_hold_sec_) << "\n";
  oss << "critical_clear_frames: " << wheel_degeneracy_latch_off_streak_thr_ << "\n";
  oss << "scan_wheel_speed_diff_thr_mps: " << formatDouble(wheel_degeneracy_scan_wheel_speed_diff_thr_mps_) << "\n";
  oss << "min_wheel_dist_m: " << formatDouble(wheel_degeneracy_min_wheel_dist_m_) << "\n";
  msg.data = oss.str();
  pub_degeneracy_debug_->publish(msg);
}

void GyroOdometerNode::publishDiagnostics(const rclcpp::Time & stamp, const std::string & level, const std::string & msg)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.level = diagLevelFromString(level);
  st.name = "localization/gyro_odometer";
  st.message = msg;
  st.hardware_id = "none";
  arr.status.push_back(st);
  pub_diag_->publish(arr);
}

}  // namespace pure_gyro_odometer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pure_gyro_odometer::GyroOdometerNode)
