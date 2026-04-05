#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pure_gnss_msgs/msg/gnss_fusion_input.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class GnssConversion : public rclcpp::Node
{
public:
  explicit GnssConversion(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct FixMeasurement
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    std::array<double, 9> position_covariance{};
    uint8_t position_covariance_type{sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN};
    int status{sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX};
    std::string frame_id;
  };

  struct DopplerMeasurement
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    double speed{0.0};
    double speed_variance{1.0e6};
    double heading{0.0};
    double heading_variance{1.0e6};
    bool has_valid_heading{false};
  };

  struct ImuMeasurement
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    double yaw_rate{0.0};
    double yaw_rate_variance{1.0e6};
  };

  struct OutputPose
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Eigen::Isometry3d T_map_child{Eigen::Isometry3d::Identity()};
    double cov_x{1.0e6};
    double cov_y{1.0e6};
    double cov_z{1.0e6};
    double cov_yaw{1.0e6};
    double speed{0.0};
    double speed_variance{1.0e6};
    bool yaw_valid{false};
    bool lever_arm_applied{false};
    bool tf_available{false};
    std::string correction_mode{"raw"};
    std::string heading_source{"none"};
    std::string antenna_frame_id;
    std::string navsatfix_frame_id;
    std::string tf_error;
    double lever_arm_xy_m{0.0};
    double added_cov_xy{0.0};
    double correction_heading_rad{0.0};
  };

  void declareParameters();
  void loadParameters();
  void validateParameters() const;
  void initializeProjection();

  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void dopplerCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void gaussKruger(double rad_phi, double rad_lambda, double & x, double & y) const;
  void publishSynchronized(const FixMeasurement & fix, const DopplerMeasurement & doppler);
  OutputPose buildOutputPose(const FixMeasurement & fix, const DopplerMeasurement & doppler);
  void publishPose(const OutputPose & output);
  void publishOdometry(const OutputPose & output);
  void publishGnssFusionInputStatus(const rclcpp::Time & stamp, int fix_status);
  void publishGnssFusionInput(const OutputPose & output, int fix_status);
  void onHeartbeat();
  void publishDiagnostics(uint8_t level, const std::string & message);
  rclcpp::Time resolveStamp(const builtin_interfaces::msg::Time & stamp) const;
  static tf2::Quaternion headingQuaternion(double heading);
  std::string resolveAntennaFrameId(const FixMeasurement & fix) const;
  bool lookupAntennaToChildTransform(
    const std::string & antenna_frame_id,
    geometry_msgs::msg::TransformStamped & tf,
    std::string & error) const;

  void updateLatestInputStampLocked(const rclcpp::Time & stamp);
  void pruneBuffersLocked();
  void pruneImuBufferLocked(const rclcpp::Time & reference_stamp);
  void bufferFixMeasurementLocked(const FixMeasurement & fix);
  void bufferDopplerMeasurementLocked(const DopplerMeasurement & doppler);
  void bufferImuMeasurementLocked(const ImuMeasurement & imu);
  bool integrateImuYawRateLocked(
    const rclcpp::Time & start_stamp, const rclcpp::Time & end_stamp,
    double & delta_yaw, double & integrated_variance) const;
  bool tryMatchFixLocked(const FixMeasurement & fix, DopplerMeasurement & matched_doppler);
  bool tryMatchDopplerLocked(const DopplerMeasurement & doppler, FixMeasurement & matched_fix);
  bool findClosestFixMeasurementLocked(
    const rclcpp::Time & stamp, std::size_t & index, int64_t & delta_ns) const;
  bool findClosestDopplerMeasurementLocked(
    const rclcpp::Time & stamp, std::size_t & index, int64_t & delta_ns) const;
  int64_t syncToleranceNanoseconds() const;
  int64_t bufferRetentionNanoseconds() const;

  static double deg2rad(double degree);
  static double sanitizeVariance(double value);
  static double sanitizeCovariance(double value);
  static double normalizeAngle(double angle);
  static Eigen::Isometry3d transformMsgToIso(const geometry_msgs::msg::Transform & t);

  // Parameters
  std::vector<double> p0_;
  std::vector<double> gnss0_;
  double a_{6378137.0};
  double F_{298.257222};
  double m0_{0.9999};
  double min_x_{0.0};
  double min_y_{0.0};
  double offset_z_{0.0};
  double doppler_min_speed_mps_{1.33};
  double doppler_speed_sigma_th_{10.0};
  double heading_sigma_deg_th_{10.0};
  double initial_heading_deg_{0.0};
  double initial_heading_rad_{0.0};
  std::string imu_topic_{"/localization/imu_corrected"};
  bool use_imu_yaw_rate_fallback_{true};
  double imu_buffer_retention_sec_{3.0};
  int imu_max_buffer_size_{500};
  double imu_yaw_rate_timeout_sec_{0.2};
  double imu_yaw_rate_variance_fallback_{0.25};
  double sync_tolerance_sec_{0.03};
  double buffer_retention_sec_{1.0};
  int max_buffer_size_{100};
  std::string frame_id_{"map"};
  std::string child_frame_id_{"base_link"};
  std::string antenna_frame_id_{"gnss_antenna"};
  bool subtract_min_offset_{true};
  bool publish_gnss_odometry_{true};
  bool publish_gnss_fusion_input_{true};
  bool publish_diagnostics_{true};
  bool enable_lever_arm_correction_{true};
  bool allow_uncorrected_output_without_heading_{true};
  double uncorrected_xy_variance_scale_{1.0};
  double tf_missing_extra_cov_xy_{25.0};
  double heartbeat_hz_{2.0};
  double input_stale_timeout_sec_{2.0};
  double sync_stale_timeout_sec_{2.0};

  // Projection constants
  double origin_lat_rad_{0.0};
  double origin_lon_rad_{0.0};
  double alpha_[5]{};
  double A_[6]{};
  double A_bar_{0.0};
  double S_bar_phi0_{0.0};
  double kt_{0.0};
  Eigen::Matrix2d K_{Eigen::Matrix2d::Identity()};
  Eigen::Rotation2Dd R_{0.0};

  // Buffered unmatched measurements and diagnostic state
  std::deque<FixMeasurement> fix_buffer_;
  std::deque<DopplerMeasurement> doppler_buffer_;
  std::deque<ImuMeasurement> imu_buffer_;
  rclcpp::Time latest_input_stamp_{0, 0, RCL_ROS_TIME};
  bool has_latest_input_stamp_{false};
  rclcpp::Time last_fix_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_doppler_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_match_stamp_{0, 0, RCL_ROS_TIME};
  bool has_last_fix_{false};
  bool has_last_doppler_{false};
  bool has_last_match_{false};
  int last_fix_status_{sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX};
  bool last_heading_valid_{false};
  double last_speed_mps_{0.0};
  double last_speed_sigma_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_heading_sigma_deg_{std::numeric_limits<double>::quiet_NaN()};
  double last_sync_dt_sec_{std::numeric_limits<double>::quiet_NaN()};
  bool last_lever_arm_applied_{false};
  bool last_tf_available_{false};
  std::string last_correction_mode_{"none"};
  std::string last_heading_source_{"none"};
  std::string last_antenna_frame_id_{"gnss_antenna"};
  std::string last_navsatfix_frame_id_;
  std::string last_tf_error_;
  double last_lever_arm_xy_m_{0.0};
  double last_added_cov_xy_{0.0};
  bool has_last_output_heading_{false};
  bool has_last_output_heading_stamp_{false};
  bool has_valid_heading_history_{false};
  double last_output_heading_rad_{0.0};
  double last_output_heading_variance_{1.0e6};
  bool has_last_imu_{false};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
  double last_imu_yaw_rate_rad_s_{0.0};
  double last_imu_yaw_rate_sigma_rad_s_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Time last_output_heading_stamp_{0, 0, RCL_ROS_TIME};
  std::mutex sync_mutex_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr doppler_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<pure_gnss_msgs::msg::GnssFusionInput>::SharedPtr gnss_fusion_input_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
