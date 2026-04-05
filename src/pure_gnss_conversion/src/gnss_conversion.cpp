#include "pure_gnss_conversion/gnss_conversion.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <iterator>
#include <stdexcept>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tf2/exceptions.h>

GnssConversion::GnssConversion(const rclcpp::NodeOptions & options)
: Node("gnss_conversion", options)
{
  declareParameters();
  loadParameters();
  validateParameters();
  initializeProjection();

  if (enable_lever_arm_correction_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  fix_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "fix", rclcpp::SensorDataQoS(),
    std::bind(&GnssConversion::fixCallback, this, std::placeholders::_1));

  doppler_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "fix_velocity", rclcpp::SensorDataQoS(),
    std::bind(&GnssConversion::dopplerCallback, this, std::placeholders::_1));

  if (use_imu_yaw_rate_fallback_ && !imu_topic_.empty()) {
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GnssConversion::imuCallback, this, std::placeholders::_1));
  }

  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("global_pose", 10);

  if (publish_gnss_odometry_) {
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("gnss_odometry", 10);
  }

  if (publish_gnss_fusion_input_) {
    gnss_fusion_input_publisher_ =
      this->create_publisher<pure_gnss_msgs::msg::GnssFusionInput>("gnss_fusion_input", 10);
  }

  if (publish_diagnostics_) {
    diagnostic_publisher_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
    const double heartbeat_period = 1.0 / std::max(0.2, heartbeat_hz_);
    heartbeat_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(heartbeat_period),
      std::bind(&GnssConversion::onHeartbeat, this));
  }

  RCLCPP_INFO(
    this->get_logger(),
    "gnss_conversion started. frame_id=%s child_frame_id=%s antenna_frame_id=%s subtract_min_offset=%s sync_tolerance=%.3f[s] buffer_retention=%.3f[s] max_buffer_size=%d diagnostics=%s gnss_odom=%s gnss_fusion_input=%s lever_arm=%s allow_uncorrected_no_heading=%s initial_heading_deg=%.3f imu_fallback=%s imu_topic=%s imu_retention=%.3f[s] imu_timeout=%.3f[s]",
    frame_id_.c_str(), child_frame_id_.c_str(), antenna_frame_id_.c_str(),
    subtract_min_offset_ ? "true" : "false", sync_tolerance_sec_, buffer_retention_sec_,
    max_buffer_size_, publish_diagnostics_ ? "true" : "false",
    publish_gnss_odometry_ ? "true" : "false",
    publish_gnss_fusion_input_ ? "true" : "false",
    enable_lever_arm_correction_ ? "true" : "false",
    allow_uncorrected_output_without_heading_ ? "true" : "false",
    initial_heading_deg_,
    use_imu_yaw_rate_fallback_ ? "true" : "false",
    imu_topic_.empty() ? "<empty>" : imu_topic_.c_str(),
    imu_buffer_retention_sec_, imu_yaw_rate_timeout_sec_);
}

void GnssConversion::declareParameters()
{
  this->declare_parameter("p0", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("gnss0", std::vector<double>{35.681236, 139.767125, 0.0});
  this->declare_parameter("a", 6378137.0);
  this->declare_parameter("F", 298.257222);
  this->declare_parameter("m0", 0.9999);
  this->declare_parameter("min_x", 0.0);
  this->declare_parameter("min_y", 0.0);
  this->declare_parameter("doppler_min_speed_mps", 1.33);
  this->declare_parameter("doppler_speed_sigma_th", 10.0);
  this->declare_parameter("heading_sigma_deg_th", 10.0);
  this->declare_parameter("initial_heading_deg", 0.0);
  this->declare_parameter("imu_topic", std::string("/localization/imu_corrected"));
  this->declare_parameter("use_imu_yaw_rate_fallback", true);
  this->declare_parameter("imu_buffer_retention_sec", 3.0);
  this->declare_parameter("imu_max_buffer_size", 500);
  this->declare_parameter("imu_yaw_rate_timeout_sec", 0.2);
  this->declare_parameter("imu_yaw_rate_variance_fallback", 0.25);
  this->declare_parameter("sync_tolerance_sec", 0.03);
  this->declare_parameter("buffer_retention_sec", 1.0);
  this->declare_parameter("max_buffer_size", 100);
  this->declare_parameter("frame_id", std::string("map"));
  this->declare_parameter("child_frame_id", std::string("base_link"));
  this->declare_parameter("antenna_frame_id", std::string("gnss_antenna"));
  this->declare_parameter("subtract_min_offset", true);
  this->declare_parameter("publish_gnss_odometry", true);
  this->declare_parameter("publish_gnss_fusion_input", true);
  this->declare_parameter("publish_diagnostics", true);
  this->declare_parameter("enable_lever_arm_correction", true);
  this->declare_parameter("allow_uncorrected_output_without_heading", true);
  this->declare_parameter("uncorrected_xy_variance_scale", 1.0);
  this->declare_parameter("tf_missing_extra_cov_xy", 25.0);
  this->declare_parameter("heartbeat_hz", 2.0);
  this->declare_parameter("input_stale_timeout_sec", 2.0);
  this->declare_parameter("sync_stale_timeout_sec", 2.0);
}

void GnssConversion::loadParameters()
{
  this->get_parameter("p0", p0_);
  this->get_parameter("gnss0", gnss0_);
  this->get_parameter("a", a_);
  this->get_parameter("F", F_);
  this->get_parameter("m0", m0_);
  this->get_parameter("min_x", min_x_);
  this->get_parameter("min_y", min_y_);
  this->get_parameter("doppler_min_speed_mps", doppler_min_speed_mps_);
  this->get_parameter("doppler_speed_sigma_th", doppler_speed_sigma_th_);
  this->get_parameter("heading_sigma_deg_th", heading_sigma_deg_th_);
  this->get_parameter("initial_heading_deg", initial_heading_deg_);
  initial_heading_rad_ = deg2rad(initial_heading_deg_);
  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("use_imu_yaw_rate_fallback", use_imu_yaw_rate_fallback_);
  this->get_parameter("imu_buffer_retention_sec", imu_buffer_retention_sec_);
  this->get_parameter("imu_max_buffer_size", imu_max_buffer_size_);
  this->get_parameter("imu_yaw_rate_timeout_sec", imu_yaw_rate_timeout_sec_);
  this->get_parameter("imu_yaw_rate_variance_fallback", imu_yaw_rate_variance_fallback_);
  this->get_parameter("sync_tolerance_sec", sync_tolerance_sec_);
  this->get_parameter("buffer_retention_sec", buffer_retention_sec_);
  this->get_parameter("max_buffer_size", max_buffer_size_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("child_frame_id", child_frame_id_);
  this->get_parameter("antenna_frame_id", antenna_frame_id_);
  this->get_parameter("subtract_min_offset", subtract_min_offset_);
  this->get_parameter("publish_gnss_odometry", publish_gnss_odometry_);
  this->get_parameter("publish_gnss_fusion_input", publish_gnss_fusion_input_);
  this->get_parameter("publish_diagnostics", publish_diagnostics_);
  this->get_parameter("enable_lever_arm_correction", enable_lever_arm_correction_);
  this->get_parameter(
    "allow_uncorrected_output_without_heading",
    allow_uncorrected_output_without_heading_);
  this->get_parameter("uncorrected_xy_variance_scale", uncorrected_xy_variance_scale_);
  this->get_parameter("tf_missing_extra_cov_xy", tf_missing_extra_cov_xy_);
  this->get_parameter("heartbeat_hz", heartbeat_hz_);
  this->get_parameter("input_stale_timeout_sec", input_stale_timeout_sec_);
  this->get_parameter("sync_stale_timeout_sec", sync_stale_timeout_sec_);
}

void GnssConversion::validateParameters() const
{
  if (p0_.size() < 3) {
    throw std::invalid_argument("parameter 'p0' must have 3 elements: [x, y, z]");
  }
  if (gnss0_.size() < 3) {
    throw std::invalid_argument("parameter 'gnss0' must have 3 elements: [lat_deg, lon_deg, alt_m]");
  }
  if (F_ <= 0.0) {
    throw std::invalid_argument("parameter 'F' must be positive");
  }
  if (a_ <= 0.0) {
    throw std::invalid_argument("parameter 'a' must be positive");
  }
  if (m0_ <= 0.0) {
    throw std::invalid_argument("parameter 'm0' must be positive");
  }
  if (sync_tolerance_sec_ < 0.0) {
    throw std::invalid_argument("parameter 'sync_tolerance_sec' must be non-negative");
  }
  if (imu_buffer_retention_sec_ <= 0.0) {
    throw std::invalid_argument("parameter 'imu_buffer_retention_sec' must be positive");
  }
  if (imu_max_buffer_size_ <= 0) {
    throw std::invalid_argument("parameter 'imu_max_buffer_size' must be positive");
  }
  if (imu_yaw_rate_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("parameter 'imu_yaw_rate_timeout_sec' must be positive");
  }
  if (imu_yaw_rate_variance_fallback_ < 0.0) {
    throw std::invalid_argument("parameter 'imu_yaw_rate_variance_fallback' must be non-negative");
  }
  if (buffer_retention_sec_ <= 0.0) {
    throw std::invalid_argument("parameter 'buffer_retention_sec' must be positive");
  }
  if (buffer_retention_sec_ < sync_tolerance_sec_) {
    throw std::invalid_argument(
            "parameter 'buffer_retention_sec' must be greater than or equal to 'sync_tolerance_sec'");
  }
  if (max_buffer_size_ <= 0) {
    throw std::invalid_argument("parameter 'max_buffer_size' must be positive");
  }
  if (heartbeat_hz_ <= 0.0) {
    throw std::invalid_argument("parameter 'heartbeat_hz' must be positive");
  }
  if (input_stale_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("parameter 'input_stale_timeout_sec' must be positive");
  }
  if (sync_stale_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("parameter 'sync_stale_timeout_sec' must be positive");
  }
  if (frame_id_.empty()) {
    throw std::invalid_argument("parameter 'frame_id' must not be empty");
  }
  if (child_frame_id_.empty()) {
    throw std::invalid_argument("parameter 'child_frame_id' must not be empty");
  }
  if (enable_lever_arm_correction_ && antenna_frame_id_.empty()) {
    throw std::invalid_argument("parameter 'antenna_frame_id' must not be empty when lever-arm correction is enabled");
  }
  if (!std::isfinite(initial_heading_deg_)) {
    throw std::invalid_argument("parameter 'initial_heading_deg' must be finite");
  }
  if (uncorrected_xy_variance_scale_ < 0.0) {
    throw std::invalid_argument("parameter 'uncorrected_xy_variance_scale' must be non-negative");
  }
  if (tf_missing_extra_cov_xy_ < 0.0) {
    throw std::invalid_argument("parameter 'tf_missing_extra_cov_xy' must be non-negative");
  }
}

void GnssConversion::initializeProjection()
{
  origin_lat_rad_ = deg2rad(gnss0_[0]);
  origin_lon_rad_ = deg2rad(gnss0_[1]);
  offset_z_ = p0_[2] - gnss0_[2];

  const double n = 1.0 / (2.0 * F_ - 1.0);
  const double n2 = std::pow(n, 2);
  const double n3 = std::pow(n, 3);
  const double n4 = std::pow(n, 4);
  const double n5 = std::pow(n, 5);

  alpha_[0] = n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0 + 41.0 * n4 / 180.0 - 127.0 * n5 / 288.0;
  alpha_[1] = 13.0 * n2 / 48.0 - 3.0 * n3 / 5.0 + 557.0 * n4 / 1440.0 + 281.0 * n5 / 630.0;
  alpha_[2] = 61.0 * n3 / 240.0 - 103.0 * n4 / 140.0 + 15061.0 * n5 / 26880.0;
  alpha_[3] = 49561.0 * n4 / 161280.0 - 179.0 * n5 / 168.0;
  alpha_[4] = 34729.0 * n5 / 80640.0;

  A_[0] = 1.0 + n2 / 4.0 + n4 / 64.0;
  A_[1] = -3.0 / 2.0 * (n - n3 / 8.0 - n5 / 64.0);
  A_[2] = 15.0 / 16.0 * (n2 - n4 / 4.0);
  A_[3] = -35.0 / 48.0 * (n3 - 5.0 * n5 / 16.0);
  A_[4] = 315.0 * n4 / 512.0;
  A_[5] = -693.0 * n5 / 1280.0;

  A_bar_ = m0_ * a_ * A_[0] / (1.0 + n);

  S_bar_phi0_ = A_[0] * origin_lat_rad_;
  for (int i = 1; i <= 5; ++i) {
    S_bar_phi0_ += A_[i] * std::sin(2.0 * i * origin_lat_rad_);
  }
  S_bar_phi0_ *= m0_ * a_ / (1.0 + n);

  kt_ = 2.0 * std::sqrt(n) / (1.0 + n);
  K_.setIdentity();
  R_ = Eigen::Rotation2Dd(-M_PI / 2.0);
}

void GnssConversion::fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  const rclcpp::Time stamp = resolveStamp(msg->header.stamp);
  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    has_last_fix_ = true;
    last_fix_stamp_ = stamp;
    last_fix_status_ = static_cast<int>(msg->status.status);
    updateLatestInputStampLocked(stamp);
  }

  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    publishGnssFusionInputStatus(stamp, static_cast<int>(msg->status.status));
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "NavSatFix status is NO_FIX. Pose publish skipped.");
    return;
  }

  if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) || !std::isfinite(msg->altitude)) {
    publishGnssFusionInputStatus(stamp, static_cast<int>(msg->status.status));
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "NavSatFix contains NaN/Inf. Pose publish skipped.");
    return;
  }

  double x{0.0};
  double y{0.0};
  gaussKruger(deg2rad(msg->latitude), deg2rad(msg->longitude), x, y);

  FixMeasurement fix;
  fix.stamp = stamp;
  fix.x = x - (subtract_min_offset_ ? min_x_ : 0.0);
  fix.y = y - (subtract_min_offset_ ? min_y_ : 0.0);
  fix.z = msg->altitude + offset_z_;
  fix.position_covariance = msg->position_covariance;
  fix.position_covariance_type = msg->position_covariance_type;
  fix.status = static_cast<int>(msg->status.status);
  fix.frame_id = msg->header.frame_id;

  DopplerMeasurement matched_doppler;
  bool matched{false};
  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    pruneBuffersLocked();
    matched = tryMatchFixLocked(fix, matched_doppler);
    if (!matched) {
      bufferFixMeasurementLocked(fix);
    }
  }

  if (matched) {
    publishSynchronized(fix, matched_doppler);
  } else {
    publishGnssFusionInputStatus(stamp, fix.status);
  }
}

void GnssConversion::dopplerCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  const double ve = msg->twist.twist.linear.x;
  const double vn = msg->twist.twist.linear.y;

  if (!std::isfinite(ve) || !std::isfinite(vn)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "fix_velocity contains NaN/Inf. Doppler publish skipped.");
    return;
  }

  const double var_e = sanitizeVariance(msg->twist.covariance[0]);
  const double var_n = sanitizeVariance(msg->twist.covariance[7]);
  const double cov_en = 0.5 * (
    sanitizeCovariance(msg->twist.covariance[1]) +
    sanitizeCovariance(msg->twist.covariance[6]));
  const double speed = std::hypot(ve, vn);
  const double speed_sq = ve * ve + vn * vn;

  DopplerMeasurement doppler;
  doppler.stamp = resolveStamp(msg->header.stamp);
  doppler.speed = speed;

  if (speed_sq > 1.0e-12) {
    const double speed_variance =
      ((ve * ve) * var_e + 2.0 * ve * vn * cov_en + (vn * vn) * var_n) / speed_sq;
    const double heading_variance =
      ((vn * vn) * var_e - 2.0 * ve * vn * cov_en + (ve * ve) * var_n) /
      (speed_sq * speed_sq);

    doppler.speed_variance = std::max(speed_variance, 0.0);
    doppler.heading = std::atan2(vn, ve);
    doppler.heading_variance = std::max(heading_variance, 0.0);
  }

  const double speed_sigma = std::sqrt(std::max(doppler.speed_variance, 0.0));
  const double heading_sigma_deg =
    std::sqrt(std::max(doppler.heading_variance, 0.0)) * 180.0 / M_PI;

  doppler.has_valid_heading =
    speed >= doppler_min_speed_mps_ &&
    speed_sigma <= doppler_speed_sigma_th_ &&
    heading_sigma_deg <= heading_sigma_deg_th_;

  FixMeasurement matched_fix;
  bool matched{false};
  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    has_last_doppler_ = true;
    last_doppler_stamp_ = doppler.stamp;
    last_speed_mps_ = doppler.speed;
    last_speed_sigma_mps_ = std::sqrt(std::max(doppler.speed_variance, 0.0));
    last_heading_valid_ = doppler.has_valid_heading;
    last_heading_sigma_deg_ = std::sqrt(std::max(doppler.heading_variance, 0.0)) * 180.0 / M_PI;
    updateLatestInputStampLocked(doppler.stamp);
    pruneBuffersLocked();
    matched = tryMatchDopplerLocked(doppler, matched_fix);
    if (!matched) {
      bufferDopplerMeasurementLocked(doppler);
    }
  }

  if (matched) {
    publishSynchronized(matched_fix, doppler);
  }
}

void GnssConversion::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const rclcpp::Time stamp = resolveStamp(msg->header.stamp);
  const double yaw_rate = msg->angular_velocity.z;
  if (!std::isfinite(yaw_rate)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "IMU angular_velocity.z contains NaN/Inf. IMU yaw-rate fallback sample skipped.");
    return;
  }

  double yaw_rate_variance = imu_yaw_rate_variance_fallback_;
  if (msg->angular_velocity_covariance[0] >= 0.0) {
    yaw_rate_variance = std::max(sanitizeVariance(msg->angular_velocity_covariance[8]), 0.0);
  }

  ImuMeasurement imu;
  imu.stamp = stamp;
  imu.yaw_rate = yaw_rate;
  imu.yaw_rate_variance = yaw_rate_variance;

  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    has_last_imu_ = true;
    last_imu_stamp_ = imu.stamp;
    last_imu_yaw_rate_rad_s_ = imu.yaw_rate;
    last_imu_yaw_rate_sigma_rad_s_ = std::sqrt(std::max(imu.yaw_rate_variance, 0.0));
    pruneImuBufferLocked(imu.stamp);
    bufferImuMeasurementLocked(imu);
  }
}

void GnssConversion::gaussKruger(double rad_phi, double rad_lambda, double & x, double & y) const
{
  const double t = std::sinh(
    std::atanh(std::sin(rad_phi)) - kt_ * std::atanh(kt_ * std::sin(rad_phi)));
  const double t_bar = std::sqrt(1.0 + std::pow(t, 2));
  const double diff_lambda = rad_lambda - origin_lon_rad_;
  const double lambda_cos = std::cos(diff_lambda);
  const double lambda_sin = std::sin(diff_lambda);
  const double zeta = std::atan2(t, lambda_cos);
  const double eta = std::atanh(lambda_sin / t_bar);

  Eigen::Vector2d p;
  p(0) = zeta;
  p(1) = eta;

  for (int i = 1; i <= 5; ++i) {
    p(0) += alpha_[i - 1] * std::sin(2.0 * i * zeta) * std::cosh(2.0 * i * eta);
    p(1) += alpha_[i - 1] * std::cos(2.0 * i * zeta) * std::sinh(2.0 * i * eta);
  }

  p(0) = p(0) * A_bar_ - S_bar_phi0_;
  p(1) *= A_bar_;
  p = K_ * R_ * p;

  x = p(0) + p0_[0];
  y = -p(1) + p0_[1];
}

void GnssConversion::publishSynchronized(
  const FixMeasurement & fix, const DopplerMeasurement & doppler)
{
  const OutputPose output = buildOutputPose(fix, doppler);

  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    has_last_match_ = true;
    last_match_stamp_ = fix.stamp;
    last_sync_dt_sec_ = std::fabs((fix.stamp - doppler.stamp).seconds());
    last_heading_valid_ = doppler.has_valid_heading;
    last_speed_mps_ = doppler.speed;
    last_speed_sigma_mps_ = std::sqrt(std::max(doppler.speed_variance, 0.0));
    last_heading_sigma_deg_ = std::sqrt(std::max(doppler.heading_variance, 0.0)) * 180.0 / M_PI;
    last_lever_arm_applied_ = output.lever_arm_applied;
    last_tf_available_ = output.tf_available;
    last_correction_mode_ = output.correction_mode;
    last_heading_source_ = output.heading_source;
    last_antenna_frame_id_ = output.antenna_frame_id;
    last_navsatfix_frame_id_ = output.navsatfix_frame_id;
    last_tf_error_ = output.tf_error;
    last_lever_arm_xy_m_ = output.lever_arm_xy_m;
    last_added_cov_xy_ = output.added_cov_xy;
    has_last_output_heading_ = true;
    has_last_output_heading_stamp_ = true;
    last_output_heading_rad_ = output.correction_heading_rad;
    last_output_heading_variance_ = output.cov_yaw;
    last_output_heading_stamp_ = output.stamp;
    if (doppler.has_valid_heading) {
      has_valid_heading_history_ = true;
    }
  }

  publishPose(output);
  if (publish_gnss_odometry_) {
    publishOdometry(output);
  }
  publishGnssFusionInput(output, fix.status);
}

GnssConversion::OutputPose GnssConversion::buildOutputPose(
  const FixMeasurement & fix,
  const DopplerMeasurement & doppler)
{
  OutputPose output;
  output.stamp = fix.stamp;
  output.antenna_frame_id = resolveAntennaFrameId(fix);
  output.navsatfix_frame_id = fix.frame_id;
  output.yaw_valid = doppler.has_valid_heading;
  output.speed = doppler.speed;
  output.speed_variance = doppler.speed_variance;

  const double unknown_pose_variance = 1.0e6;
  double correction_heading_rad = initial_heading_rad_;
  double heading_variance = unknown_pose_variance;
  std::string heading_source = "initial_param";

  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    if (doppler.has_valid_heading) {
      correction_heading_rad = doppler.heading;
      heading_variance = doppler.heading_variance;
      heading_source = "doppler";
    } else if (
      use_imu_yaw_rate_fallback_ &&
      has_last_output_heading_ &&
      has_last_output_heading_stamp_)
    {
      pruneImuBufferLocked(fix.stamp);
      double delta_yaw = 0.0;
      double integrated_variance = 0.0;
      if (integrateImuYawRateLocked(
            last_output_heading_stamp_, fix.stamp, delta_yaw, integrated_variance))
      {
        correction_heading_rad = normalizeAngle(last_output_heading_rad_ + delta_yaw);
        heading_variance = std::min(
          unknown_pose_variance,
          std::max(0.0, last_output_heading_variance_) + std::max(0.0, integrated_variance));
        heading_source = "imu_yaw_rate";
      } else {
        correction_heading_rad = last_output_heading_rad_;
        heading_variance = last_output_heading_variance_;
        heading_source = has_valid_heading_history_ ? "previous_sample" : "initial_param";
      }
    } else if (has_last_output_heading_) {
      correction_heading_rad = last_output_heading_rad_;
      heading_variance = last_output_heading_variance_;
      heading_source = has_valid_heading_history_ ? "previous_sample" : "initial_param";
    }
  }
  output.heading_source = heading_source;
  output.correction_heading_rad = correction_heading_rad;

  output.T_map_child.translation() = Eigen::Vector3d(fix.x, fix.y, fix.z);
  const tf2::Quaternion q_heading_tf = headingQuaternion(correction_heading_rad);
  Eigen::Quaterniond q_heading(
    q_heading_tf.w(), q_heading_tf.x(), q_heading_tf.y(), q_heading_tf.z());
  if (q_heading.norm() < 1.0e-9) {
    q_heading = Eigen::Quaterniond::Identity();
  }
  q_heading.normalize();
  output.T_map_child.linear() = q_heading.toRotationMatrix();

  if (fix.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    output.cov_x = unknown_pose_variance;
    output.cov_y = unknown_pose_variance;
    output.cov_z = unknown_pose_variance;
  } else {
    output.cov_x = std::max(sanitizeVariance(fix.position_covariance[0]), 0.0);
    output.cov_y = std::max(sanitizeVariance(fix.position_covariance[4]), 0.0);
    output.cov_z = std::max(sanitizeVariance(fix.position_covariance[8]), 0.0);
  }
  output.cov_yaw = std::min(unknown_pose_variance, std::max(0.0, heading_variance));

  if (!enable_lever_arm_correction_) {
    output.correction_mode = doppler.has_valid_heading ? "disabled" : "disabled_no_lever_arm";
    if (!doppler.has_valid_heading && !allow_uncorrected_output_without_heading_) {
      output.cov_x = unknown_pose_variance;
      output.cov_y = unknown_pose_variance;
      output.added_cov_xy = unknown_pose_variance;
    }
    return output;
  }

  geometry_msgs::msg::TransformStamped tf_antenna_child;
  std::string tf_error;
  if (!lookupAntennaToChildTransform(output.antenna_frame_id, tf_antenna_child, tf_error)) {
    output.tf_available = false;
    output.correction_mode = "uncorrected_tf_unavailable";
    output.tf_error = tf_error;
    output.cov_x += tf_missing_extra_cov_xy_;
    output.cov_y += tf_missing_extra_cov_xy_;
    output.added_cov_xy = tf_missing_extra_cov_xy_;
    return output;
  }

  output.tf_available = true;
  const Eigen::Isometry3d T_antenna_child = transformMsgToIso(tf_antenna_child.transform);
  output.lever_arm_xy_m =
    std::hypot(T_antenna_child.translation().x(), T_antenna_child.translation().y());

  Eigen::Isometry3d T_map_antenna = Eigen::Isometry3d::Identity();
  T_map_antenna.translation() = Eigen::Vector3d(fix.x, fix.y, fix.z);
  T_map_antenna.linear() = q_heading.toRotationMatrix();
  output.T_map_child = T_map_antenna * T_antenna_child;

  Eigen::Quaterniond q_out(output.T_map_child.linear());
  if (q_out.norm() < 1.0e-9) {
    q_out = Eigen::Quaterniond::Identity();
  }
  q_out.normalize();
  output.T_map_child.linear() = q_out.toRotationMatrix();

  output.lever_arm_applied = true;
  if (doppler.has_valid_heading) {
    output.correction_mode = "applied";
    return output;
  }

  if (heading_source == "imu_yaw_rate") {
    output.correction_mode = "applied_imu_yaw_rate";
    const double extra_cov = std::min(
      unknown_pose_variance,
      output.lever_arm_xy_m * output.lever_arm_xy_m *
      std::max(0.0, output.cov_yaw) * std::max(0.0, uncorrected_xy_variance_scale_));
    output.cov_x += extra_cov;
    output.cov_y += extra_cov;
    output.added_cov_xy = extra_cov;
    return output;
  }

  if (heading_source == "previous_sample") {
    output.correction_mode = "applied_previous_heading";
  } else {
    output.correction_mode = "applied_initial_heading";
  }

  const double extra_cov =
    0.5 * output.lever_arm_xy_m * output.lever_arm_xy_m *
    std::max(0.0, uncorrected_xy_variance_scale_);
  if (allow_uncorrected_output_without_heading_) {
    output.cov_x += extra_cov;
    output.cov_y += extra_cov;
    output.added_cov_xy = extra_cov;
  } else {
    output.cov_x = unknown_pose_variance;
    output.cov_y = unknown_pose_variance;
    output.added_cov_xy = unknown_pose_variance;
  }
  return output;
}

void GnssConversion::publishPose(const OutputPose & output)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = output.stamp;
  pose.header.frame_id = frame_id_;
  pose.pose.position.x = output.T_map_child.translation().x();
  pose.pose.position.y = output.T_map_child.translation().y();
  pose.pose.position.z = output.T_map_child.translation().z();

  Eigen::Quaterniond q(output.T_map_child.linear());
  if (q.norm() < 1.0e-9) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  pose_publisher_->publish(pose);
}

void GnssConversion::publishOdometry(const OutputPose & output)
{
  if (!publish_gnss_odometry_ || !odometry_publisher_) {
    return;
  }

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = output.stamp;
  odom.header.frame_id = frame_id_;
  odom.child_frame_id = child_frame_id_;

  odom.pose.pose.position.x = output.T_map_child.translation().x();
  odom.pose.pose.position.y = output.T_map_child.translation().y();
  odom.pose.pose.position.z = output.T_map_child.translation().z();

  Eigen::Quaterniond q(output.T_map_child.linear());
  if (q.norm() < 1.0e-9) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
  odom.pose.covariance[0] = output.cov_x;
  odom.pose.covariance[7] = output.cov_y;
  odom.pose.covariance[14] = output.cov_z;
  odom.pose.covariance[21] = 1.0e6;
  odom.pose.covariance[28] = 1.0e6;
  odom.pose.covariance[35] = output.cov_yaw;

  std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
  odom.twist.twist.linear.x = output.speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.covariance[0] = output.speed_variance;
  odom.twist.covariance[7] = 1.0e6;
  odom.twist.covariance[14] = 1.0e6;
  odom.twist.covariance[21] = 1.0e6;
  odom.twist.covariance[28] = 1.0e6;
  odom.twist.covariance[35] = 1.0e6;

  odometry_publisher_->publish(odom);
}

void GnssConversion::publishGnssFusionInputStatus(const rclcpp::Time & stamp, int fix_status)
{
  if (!publish_gnss_fusion_input_ || !gnss_fusion_input_publisher_) {
    return;
  }

  pure_gnss_msgs::msg::GnssFusionInput msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  msg.has_odom = false;
  msg.fix_status = static_cast<int8_t>(fix_status);
  gnss_fusion_input_publisher_->publish(msg);
}

void GnssConversion::publishGnssFusionInput(const OutputPose & output, int fix_status)
{
  if (!publish_gnss_fusion_input_ || !gnss_fusion_input_publisher_) {
    return;
  }

  pure_gnss_msgs::msg::GnssFusionInput msg;
  msg.header.stamp = output.stamp;
  msg.header.frame_id = frame_id_;
  msg.has_odom = true;
  msg.fix_status = static_cast<int8_t>(fix_status);

  nav_msgs::msg::Odometry & odom = msg.odom;
  odom.header.stamp = output.stamp;
  odom.header.frame_id = frame_id_;
  odom.child_frame_id = child_frame_id_;

  odom.pose.pose.position.x = output.T_map_child.translation().x();
  odom.pose.pose.position.y = output.T_map_child.translation().y();
  odom.pose.pose.position.z = output.T_map_child.translation().z();

  Eigen::Quaterniond q(output.T_map_child.linear());
  if (q.norm() < 1.0e-9) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
  odom.pose.covariance[0] = output.cov_x;
  odom.pose.covariance[7] = output.cov_y;
  odom.pose.covariance[14] = output.cov_z;
  odom.pose.covariance[21] = 1.0e6;
  odom.pose.covariance[28] = 1.0e6;
  odom.pose.covariance[35] = output.cov_yaw;

  std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
  odom.twist.twist.linear.x = output.speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.covariance[0] = output.speed_variance;
  odom.twist.covariance[7] = 1.0e6;
  odom.twist.covariance[14] = 1.0e6;
  odom.twist.covariance[21] = 1.0e6;
  odom.twist.covariance[28] = 1.0e6;
  odom.twist.covariance[35] = 1.0e6;

  gnss_fusion_input_publisher_->publish(msg);
}

void GnssConversion::onHeartbeat()
{
  if (!publish_diagnostics_) {
    return;
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "publishing synchronized GNSS pose";

  const rclcpp::Time nowt = this->now();
  bool has_last_fix = false;
  bool has_last_doppler = false;
  bool has_last_match = false;
  rclcpp::Time last_fix_stamp(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_doppler_stamp(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_match_stamp(0, 0, RCL_ROS_TIME);
  int last_fix_status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  bool last_tf_available = false;
  std::string last_correction_mode;

  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    has_last_fix = has_last_fix_;
    has_last_doppler = has_last_doppler_;
    has_last_match = has_last_match_;
    last_fix_stamp = last_fix_stamp_;
    last_doppler_stamp = last_doppler_stamp_;
    last_match_stamp = last_match_stamp_;
    last_fix_status = last_fix_status_;
    last_tf_available = last_tf_available_;
    last_correction_mode = last_correction_mode_;
  }

  const double fix_age = has_last_fix ? std::fabs((nowt - last_fix_stamp).seconds()) : std::numeric_limits<double>::infinity();
  const double doppler_age = has_last_doppler ? std::fabs((nowt - last_doppler_stamp).seconds()) : std::numeric_limits<double>::infinity();
  const double match_age = has_last_match ? std::fabs((nowt - last_match_stamp).seconds()) : std::numeric_limits<double>::infinity();

  if (!has_last_fix && !has_last_doppler) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "waiting for fix and fix_velocity";
  } else if (has_last_fix && last_fix_status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "NavSatFix is NO_FIX";
  } else if (!has_last_match && (has_last_fix || has_last_doppler)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "waiting synchronized fix/fix_velocity pair";
  } else if ((has_last_fix && fix_age > input_stale_timeout_sec_) ||
             (has_last_doppler && doppler_age > input_stale_timeout_sec_)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "GNSS input appears stale (check use_sim_time during rosbag playback)";
  } else if (has_last_match && match_age > sync_stale_timeout_sec_) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "no synchronized fix/fix_velocity pair recently";
  } else if (enable_lever_arm_correction_ && has_last_match && !last_tf_available) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "GNSS antenna TF unavailable: publishing uncorrected position";
  } else if (enable_lever_arm_correction_ && last_correction_mode == "applied_initial_heading") {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "Doppler heading invalid: using initial heading parameter for lever-arm correction";
  } else if (enable_lever_arm_correction_ && last_correction_mode == "applied_previous_heading") {
    level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    message = "publishing synchronized GNSS pose (lever-arm corrected with previous heading)";
  } else if (enable_lever_arm_correction_ && last_correction_mode == "applied_imu_yaw_rate") {
    level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    message = "publishing synchronized GNSS pose (lever-arm corrected with IMU yaw-rate fallback)";
  }

  publishDiagnostics(level, message);
}

void GnssConversion::publishDiagnostics(uint8_t level, const std::string & message)
{
  if (!publish_diagnostics_ || !diagnostic_publisher_) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.level = level;
  st.name = "localization/gnss_conversion";
  st.message = message;
  st.hardware_id = "none";

  auto add = [&](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    st.values.push_back(kv);
  };

  bool has_last_fix = false;
  bool has_last_doppler = false;
  bool has_last_match = false;
  rclcpp::Time last_fix_stamp(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_doppler_stamp(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_match_stamp(0, 0, RCL_ROS_TIME);
  int last_fix_status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  bool last_heading_valid = false;
  double last_speed_mps = 0.0;
  double last_speed_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double last_heading_sigma_deg = std::numeric_limits<double>::quiet_NaN();
  double last_sync_dt_sec = std::numeric_limits<double>::quiet_NaN();
  std::size_t fix_buffer_size = 0;
  std::size_t doppler_buffer_size = 0;
  bool has_latest_input_stamp = false;
  rclcpp::Time latest_input_stamp(0, 0, RCL_ROS_TIME);
  bool last_lever_arm_applied = false;
  bool last_tf_available = false;
  std::string last_correction_mode;
  std::string last_heading_source;
  std::string last_antenna_frame_id;
  std::string last_navsatfix_frame_id;
  std::string last_tf_error;
  double last_lever_arm_xy_m = 0.0;
  double last_added_cov_xy = 0.0;
  bool has_valid_heading_history = false;
  double last_output_heading_rad = 0.0;
  double last_output_heading_variance = 1.0e6;
  std::size_t imu_buffer_size = 0;
  bool has_last_imu = false;
  rclcpp::Time last_imu_stamp(0, 0, RCL_ROS_TIME);
  double last_imu_yaw_rate_rad_s = 0.0;
  double last_imu_yaw_rate_sigma_rad_s = std::numeric_limits<double>::quiet_NaN();

  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    has_last_fix = has_last_fix_;
    has_last_doppler = has_last_doppler_;
    has_last_match = has_last_match_;
    last_fix_stamp = last_fix_stamp_;
    last_doppler_stamp = last_doppler_stamp_;
    last_match_stamp = last_match_stamp_;
    last_fix_status = last_fix_status_;
    last_heading_valid = last_heading_valid_;
    last_speed_mps = last_speed_mps_;
    last_speed_sigma_mps = last_speed_sigma_mps_;
    last_heading_sigma_deg = last_heading_sigma_deg_;
    last_sync_dt_sec = last_sync_dt_sec_;
    fix_buffer_size = fix_buffer_.size();
    doppler_buffer_size = doppler_buffer_.size();
    has_latest_input_stamp = has_latest_input_stamp_;
    latest_input_stamp = latest_input_stamp_;
    last_lever_arm_applied = last_lever_arm_applied_;
    last_tf_available = last_tf_available_;
    last_correction_mode = last_correction_mode_;
    last_heading_source = last_heading_source_;
    last_antenna_frame_id = last_antenna_frame_id_;
    last_navsatfix_frame_id = last_navsatfix_frame_id_;
    last_tf_error = last_tf_error_;
    last_lever_arm_xy_m = last_lever_arm_xy_m_;
    last_added_cov_xy = last_added_cov_xy_;
    has_valid_heading_history = has_valid_heading_history_;
    last_output_heading_rad = last_output_heading_rad_;
    last_output_heading_variance = last_output_heading_variance_;
    imu_buffer_size = imu_buffer_.size();
    has_last_imu = has_last_imu_;
    last_imu_stamp = last_imu_stamp_;
    last_imu_yaw_rate_rad_s = last_imu_yaw_rate_rad_s_;
    last_imu_yaw_rate_sigma_rad_s = last_imu_yaw_rate_sigma_rad_s_;
  }

  const rclcpp::Time nowt = this->now();
  add("publish_gnss_odometry", publish_gnss_odometry_ ? "true" : "false");
  add("publish_gnss_fusion_input", publish_gnss_fusion_input_ ? "true" : "false");
  add("sync_tolerance_sec", std::to_string(sync_tolerance_sec_));
  add("buffer_retention_sec", std::to_string(buffer_retention_sec_));
  add("use_imu_yaw_rate_fallback", use_imu_yaw_rate_fallback_ ? "true" : "false");
  add("imu_topic", imu_topic_.empty() ? "<empty>" : imu_topic_);
  add("imu_buffer_retention_sec", std::to_string(imu_buffer_retention_sec_));
  add("imu_yaw_rate_timeout_sec", std::to_string(imu_yaw_rate_timeout_sec_));
  add("fix_buffer.size", std::to_string(fix_buffer_size));
  add("doppler_buffer.size", std::to_string(doppler_buffer_size));
  add("imu_buffer.size", std::to_string(imu_buffer_size));
  add("last_fix.status", std::to_string(last_fix_status));
  add("last_heading_valid", last_heading_valid ? "true" : "false");
  add("last_speed_mps", std::to_string(last_speed_mps));
  add("last_speed_sigma_mps", std::to_string(last_speed_sigma_mps));
  add("last_heading_sigma_deg", std::to_string(last_heading_sigma_deg));
  add("last_sync_dt_sec", std::to_string(last_sync_dt_sec));
  add("has_latest_input_stamp", has_latest_input_stamp ? "true" : "false");
  add("enable_lever_arm_correction", enable_lever_arm_correction_ ? "true" : "false");
  add("initial_heading_deg", std::to_string(initial_heading_deg_));
  add(
    "allow_uncorrected_output_without_heading",
    allow_uncorrected_output_without_heading_ ? "true" : "false");
  add("last_lever_arm_applied", last_lever_arm_applied ? "true" : "false");
  add("last_tf_available", last_tf_available ? "true" : "false");
  add("last_correction_mode", last_correction_mode);
  add("last_heading_source", last_heading_source);
  add("has_valid_heading_history", has_valid_heading_history ? "true" : "false");
  add("last_output_heading_deg", std::to_string(last_output_heading_rad * 180.0 / M_PI));
  add("last_output_heading_sigma_deg", std::to_string(std::sqrt(std::max(0.0, last_output_heading_variance)) * 180.0 / M_PI));
  add("has_last_imu", has_last_imu ? "true" : "false");
  add("last_imu_yaw_rate_rad_s", std::to_string(last_imu_yaw_rate_rad_s));
  add("last_imu_yaw_rate_sigma_rad_s", std::to_string(last_imu_yaw_rate_sigma_rad_s));
  add("antenna_frame_id", last_antenna_frame_id);
  add("last_navsatfix_frame_id", last_navsatfix_frame_id.empty() ? "<empty>" : last_navsatfix_frame_id);
  add("last_lever_arm_xy_m", std::to_string(last_lever_arm_xy_m));
  add("last_added_cov_xy", std::to_string(last_added_cov_xy));
  if (!last_tf_error.empty()) {
    add("last_tf_error", last_tf_error);
  }

  if (has_last_fix) {
    add("last_fix_age_sec", std::to_string(std::fabs((nowt - last_fix_stamp).seconds())));
  }
  if (has_last_doppler) {
    add("last_doppler_age_sec", std::to_string(std::fabs((nowt - last_doppler_stamp).seconds())));
  }
  if (has_last_match) {
    add("last_match_age_sec", std::to_string(std::fabs((nowt - last_match_stamp).seconds())));
  }
  if (has_last_imu) {
    add("last_imu_age_sec", std::to_string(std::fabs((nowt - last_imu_stamp).seconds())));
  }
  if (has_latest_input_stamp) {
    add("latest_input_age_sec", std::to_string(std::fabs((nowt - latest_input_stamp).seconds())));
  }
  if (!last_navsatfix_frame_id.empty() &&
      enable_lever_arm_correction_ &&
      last_navsatfix_frame_id != last_antenna_frame_id) {
    add("frame_note", "using antenna_frame_id parameter for TF; NavSatFix header.frame_id differs");
  }

  arr.status.push_back(st);
  diagnostic_publisher_->publish(arr);
}

rclcpp::Time GnssConversion::resolveStamp(const builtin_interfaces::msg::Time & stamp) const
{
  rclcpp::Time resolved(stamp);
  if (resolved.nanoseconds() == 0) {
    resolved = this->now();
  }
  return resolved;
}

tf2::Quaternion GnssConversion::headingQuaternion(double heading)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, heading);
  q.normalize();
  return q;
}

std::string GnssConversion::resolveAntennaFrameId(const FixMeasurement & fix) const
{
  (void)fix;
  return antenna_frame_id_;
}

bool GnssConversion::lookupAntennaToChildTransform(
  const std::string & antenna_frame_id,
  geometry_msgs::msg::TransformStamped & tf,
  std::string & error) const
{
  if (!tf_buffer_) {
    error = "tf buffer is not initialized";
    return false;
  }

  try {
    tf = tf_buffer_->lookupTransform(antenna_frame_id, child_frame_id_, tf2::TimePointZero);
    return true;
  } catch (const tf2::TransformException & ex) {
    error = ex.what();
    return false;
  }
}

void GnssConversion::updateLatestInputStampLocked(const rclcpp::Time & stamp)
{
  if (!has_latest_input_stamp_ || stamp.nanoseconds() > latest_input_stamp_.nanoseconds()) {
    latest_input_stamp_ = stamp;
    has_latest_input_stamp_ = true;
  }
}

void GnssConversion::pruneBuffersLocked()
{
  if (!has_latest_input_stamp_) {
    return;
  }

  const int64_t retention_ns = bufferRetentionNanoseconds();
  const int64_t reference_ns = latest_input_stamp_.nanoseconds();

  auto fix_is_stale = [reference_ns, retention_ns](const FixMeasurement & fix) {
      return (reference_ns - fix.stamp.nanoseconds()) > retention_ns;
    };
  auto doppler_is_stale = [reference_ns, retention_ns](const DopplerMeasurement & doppler) {
      return (reference_ns - doppler.stamp.nanoseconds()) > retention_ns;
    };

  fix_buffer_.erase(
    std::remove_if(fix_buffer_.begin(), fix_buffer_.end(), fix_is_stale),
    fix_buffer_.end());
  doppler_buffer_.erase(
    std::remove_if(doppler_buffer_.begin(), doppler_buffer_.end(), doppler_is_stale),
    doppler_buffer_.end());
}

void GnssConversion::pruneImuBufferLocked(const rclcpp::Time & reference_stamp)
{
  const int64_t retention_ns =
    static_cast<int64_t>(std::llround(imu_buffer_retention_sec_ * 1.0e9));
  const int64_t reference_ns = reference_stamp.nanoseconds();

  imu_buffer_.erase(
    std::remove_if(
      imu_buffer_.begin(), imu_buffer_.end(),
      [reference_ns, retention_ns](const ImuMeasurement & imu) {
        return (reference_ns - imu.stamp.nanoseconds()) > retention_ns;
      }),
    imu_buffer_.end());
}

void GnssConversion::bufferFixMeasurementLocked(const FixMeasurement & fix)
{
  const int64_t stamp_ns = fix.stamp.nanoseconds();
  const auto insert_it = std::upper_bound(
    fix_buffer_.begin(), fix_buffer_.end(), stamp_ns,
    [](const int64_t value, const FixMeasurement & item) {
      return value < item.stamp.nanoseconds();
    });
  fix_buffer_.insert(insert_it, fix);

  while (fix_buffer_.size() > static_cast<std::size_t>(max_buffer_size_)) {
    fix_buffer_.pop_front();
  }
}

void GnssConversion::bufferDopplerMeasurementLocked(const DopplerMeasurement & doppler)
{
  const int64_t stamp_ns = doppler.stamp.nanoseconds();
  const auto insert_it = std::upper_bound(
    doppler_buffer_.begin(), doppler_buffer_.end(), stamp_ns,
    [](const int64_t value, const DopplerMeasurement & item) {
      return value < item.stamp.nanoseconds();
    });
  doppler_buffer_.insert(insert_it, doppler);

  while (doppler_buffer_.size() > static_cast<std::size_t>(max_buffer_size_)) {
    doppler_buffer_.pop_front();
  }
}

void GnssConversion::bufferImuMeasurementLocked(const ImuMeasurement & imu)
{
  const int64_t stamp_ns = imu.stamp.nanoseconds();
  const auto insert_it = std::upper_bound(
    imu_buffer_.begin(), imu_buffer_.end(), stamp_ns,
    [](const int64_t value, const ImuMeasurement & item) {
      return value < item.stamp.nanoseconds();
    });
  imu_buffer_.insert(insert_it, imu);

  while (imu_buffer_.size() > static_cast<std::size_t>(imu_max_buffer_size_)) {
    imu_buffer_.pop_front();
  }
}

bool GnssConversion::integrateImuYawRateLocked(
  const rclcpp::Time & start_stamp, const rclcpp::Time & end_stamp,
  double & delta_yaw, double & integrated_variance) const
{
  delta_yaw = 0.0;
  integrated_variance = 0.0;

  const int64_t start_ns = start_stamp.nanoseconds();
  const int64_t end_ns = end_stamp.nanoseconds();
  if (end_ns < start_ns) {
    return false;
  }
  if (end_ns == start_ns) {
    return true;
  }
  if (imu_buffer_.empty()) {
    return false;
  }

  const double max_gap_sec = std::max(0.0, imu_yaw_rate_timeout_sec_);
  const auto upper = std::upper_bound(
    imu_buffer_.begin(), imu_buffer_.end(), start_ns,
    [](const int64_t value, const ImuMeasurement & item) {
      return value < item.stamp.nanoseconds();
    });

  double current_rate = 0.0;
  double current_variance = imu_yaw_rate_variance_fallback_;
  int64_t current_time_ns = start_ns;
  auto it = upper;

  if (upper != imu_buffer_.begin()) {
    const auto prev = std::prev(upper);
    current_rate = prev->yaw_rate;
    current_variance = prev->yaw_rate_variance;
  } else {
    if (upper == imu_buffer_.end()) {
      return false;
    }
    const double gap_sec = (upper->stamp.nanoseconds() - start_ns) * 1.0e-9;
    if (gap_sec > max_gap_sec) {
      return false;
    }
    current_rate = upper->yaw_rate;
    current_variance = upper->yaw_rate_variance;
  }

  for (; it != imu_buffer_.end(); ++it) {
    const int64_t sample_ns = it->stamp.nanoseconds();
    if (sample_ns <= start_ns) {
      continue;
    }
    if (sample_ns > end_ns) {
      break;
    }

    const double dt_sec = (sample_ns - current_time_ns) * 1.0e-9;
    if (dt_sec < 0.0 || dt_sec > max_gap_sec) {
      return false;
    }

    delta_yaw += current_rate * dt_sec;
    integrated_variance += std::max(0.0, current_variance) * dt_sec * dt_sec;
    current_rate = it->yaw_rate;
    current_variance = it->yaw_rate_variance;
    current_time_ns = sample_ns;
  }

  const double tail_dt_sec = (end_ns - current_time_ns) * 1.0e-9;
  if (tail_dt_sec < 0.0 || tail_dt_sec > max_gap_sec) {
    return false;
  }

  delta_yaw += current_rate * tail_dt_sec;
  integrated_variance += std::max(0.0, current_variance) * tail_dt_sec * tail_dt_sec;
  return true;
}

bool GnssConversion::tryMatchFixLocked(
  const FixMeasurement & fix, DopplerMeasurement & matched_doppler)
{
  std::size_t doppler_index{0};
  int64_t delta_ns{0};
  if (!findClosestDopplerMeasurementLocked(fix.stamp, doppler_index, delta_ns)) {
    return false;
  }

  matched_doppler = doppler_buffer_.at(doppler_index);
  doppler_buffer_.erase(doppler_buffer_.begin() + static_cast<std::ptrdiff_t>(doppler_index));
  return true;
}

bool GnssConversion::tryMatchDopplerLocked(
  const DopplerMeasurement & doppler, FixMeasurement & matched_fix)
{
  std::size_t fix_index{0};
  int64_t delta_ns{0};
  if (!findClosestFixMeasurementLocked(doppler.stamp, fix_index, delta_ns)) {
    return false;
  }

  matched_fix = fix_buffer_.at(fix_index);
  fix_buffer_.erase(fix_buffer_.begin() + static_cast<std::ptrdiff_t>(fix_index));
  return true;
}

bool GnssConversion::findClosestFixMeasurementLocked(
  const rclcpp::Time & stamp, std::size_t & index, int64_t & delta_ns) const
{
  if (fix_buffer_.empty()) {
    return false;
  }

  const int64_t target_ns = stamp.nanoseconds();
  const int64_t tolerance_ns = syncToleranceNanoseconds();
  bool found{false};
  int64_t best_dt = std::numeric_limits<int64_t>::max();
  std::size_t best_index{0};

  for (std::size_t i = 0; i < fix_buffer_.size(); ++i) {
    const int64_t dt = std::llabs(fix_buffer_[i].stamp.nanoseconds() - target_ns);
    if (dt <= tolerance_ns && dt < best_dt) {
      best_dt = dt;
      best_index = i;
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  index = best_index;
  delta_ns = best_dt;
  return true;
}

bool GnssConversion::findClosestDopplerMeasurementLocked(
  const rclcpp::Time & stamp, std::size_t & index, int64_t & delta_ns) const
{
  if (doppler_buffer_.empty()) {
    return false;
  }

  const int64_t target_ns = stamp.nanoseconds();
  const int64_t tolerance_ns = syncToleranceNanoseconds();
  bool found{false};
  int64_t best_dt = std::numeric_limits<int64_t>::max();
  std::size_t best_index{0};

  for (std::size_t i = 0; i < doppler_buffer_.size(); ++i) {
    const int64_t dt = std::llabs(doppler_buffer_[i].stamp.nanoseconds() - target_ns);
    if (dt <= tolerance_ns && dt < best_dt) {
      best_dt = dt;
      best_index = i;
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  index = best_index;
  delta_ns = best_dt;
  return true;
}

int64_t GnssConversion::syncToleranceNanoseconds() const
{
  return static_cast<int64_t>(std::llround(sync_tolerance_sec_ * 1.0e9));
}

int64_t GnssConversion::bufferRetentionNanoseconds() const
{
  return static_cast<int64_t>(std::llround(buffer_retention_sec_ * 1.0e9));
}

double GnssConversion::deg2rad(double degree)
{
  return degree * M_PI / 180.0;
}

double GnssConversion::sanitizeVariance(double value)
{
  if (!std::isfinite(value)) {
    return 1.0e6;
  }
  return std::max(value, 0.0);
}

double GnssConversion::sanitizeCovariance(double value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return value;
}

double GnssConversion::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Isometry3d GnssConversion::transformMsgToIso(const geometry_msgs::msg::Transform & t)
{
  Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  if (q.norm() < 1.0e-9) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
  return T;
}
