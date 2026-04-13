#pragma once

#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pure_gnss_msgs/msg/gnss_fusion_input.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pure_gnss_map_odom_fusion
{

class MapOdomFusionNode : public rclcpp::Node
{
public:
  explicit MapOdomFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct OdomSample
  {
    rclcpp::Time stamp;
    Eigen::Isometry3d T_odom_base{Eigen::Isometry3d::Identity()};
    geometry_msgs::msg::Twist twist;
    double cov_xy_total{0.0};
    double cov_yaw_total{0.0};
  };

  struct AbsolutePoseMeasurement
  {
    rclcpp::Time stamp;
    Eigen::Isometry3d T_map_base{Eigen::Isometry3d::Identity()};
    double cov_xy{1.0e6};
    double cov_yaw{1.0e6};
    bool yaw_valid{true};
    bool has_gnss_fix_status{false};
    int gnss_fix_status{sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX};
    std::string source{"none"};
  };

  struct AnchorState
  {
    rclcpp::Time stamp;
    Eigen::Isometry3d T_map_odom{Eigen::Isometry3d::Identity()};
    double cov_xy{0.25};
    double cov_yaw{0.04};
    double odom_cov_xy_ref{0.0};
    double odom_cov_yaw_ref{0.0};
    std::string source{"none"};
  };

  struct GnssStatusState
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    int status{sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX};
    bool valid{false};
  };

  enum class GnssFixState
  {
    UNKNOWN,
    GOOD,
    BAD,
  };

  static double wrapAngle(double a);
  static double clamp01(double a);
  static double yawFromIso(const Eigen::Isometry3d & T);
  static Eigen::Quaterniond quatFromYaw(double yaw);
  static Eigen::Isometry3d poseMsgToIso(const geometry_msgs::msg::Pose & p);
  static geometry_msgs::msg::Pose isoToPoseMsg(const Eigen::Isometry3d & T);
  static Eigen::Isometry3d deltaFromTwist(const geometry_msgs::msg::Twist & tw, double dt);

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGnssInput(const pure_gnss_msgs::msg::GnssFusionInput::SharedPtr msg);
  void onLegacyAnchorPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onPublishTimer();
  void onHeartbeat();

  std::optional<OdomSample> interpolateOdom(const rclcpp::Time & stamp) const;
  std::optional<OdomSample> latestOdom(const rclcpp::Time & stamp) const;
  void handleMeasurement(const AbsolutePoseMeasurement & meas, bool allow_pending = true);
  bool resolvePendingMeasurement(const rclcpp::Time & nowt);
  void applyMeasurement(const AbsolutePoseMeasurement & meas, const OdomSample & od);
  AbsolutePoseMeasurement measurementFromGnssInput(const pure_gnss_msgs::msg::GnssFusionInput & msg) const;
  AbsolutePoseMeasurement measurementFromPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
    const std::string & source) const;
  GnssFixState gnssFixState(const rclcpp::Time & stamp) const;
  GnssFixState gnssFixStateFromStatus(int status) const;
  double qualityFromCovariance(double cov, double ref, double max) const;
  void publishFused(const rclcpp::Time & stamp, const std::optional<OdomSample> & od = std::nullopt);
  void publishDiag(uint8_t level, const std::string & msg);

  // Parameters
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  std::string odom_topic_{"/localization/gyro_lidar_odom"};
  std::string gnss_input_topic_{"/localization/gnss_fusion_input"};
  std::string gnss_odom_topic_deprecated_{"/localization/gnss_odometry"};
  std::string gnss_fix_topic_deprecated_{"/ublox_gps_node/fix"};
  std::string legacy_anchor_topic_{""};
  std::string initialpose_topic_{"/initialpose"};

  std::string out_pose_topic_{"/localization/ekf_pose"};
  std::string out_odom_topic_{"/localization/ekf_odom"};

  bool publish_tf_{true};
  double publish_rate_hz_{50.0};
  double heartbeat_hz_{2.0};

  double odom_buffer_sec_{10.0};
  double odom_timeout_sec_{5.0};
  double odom_max_extrapolate_sec_{0.5};

  double anchor_cov_xy_default_{0.25};
  double anchor_cov_yaw_default_{0.04};
  double cov_xy_drift_per_sec_{0.2};
  double cov_yaw_drift_per_sec_{0.02};
  double no_fix_cov_drift_scale_{3.0};

  bool use_gnss_status_{true};
  double gnss_status_timeout_sec_{3.0};
  int gnss_fix_min_status_{sensor_msgs::msg::NavSatStatus::STATUS_FIX};
  bool gnss_init_require_fix_{true};

  double gnss_base_alpha_xy_{0.35};
  double gnss_base_alpha_yaw_{0.20};
  double gnss_min_alpha_xy_{0.02};
  double gnss_min_alpha_yaw_{0.0};
  double gnss_no_fix_ratio_xy_{0.15};
  double gnss_no_fix_ratio_yaw_{0.0};
  double gnss_cov_ref_xy_{0.5};
  double gnss_cov_ref_yaw_{0.10};
  double gnss_max_cov_xy_{100.0};
  double gnss_max_cov_yaw_{10.0};
  double gnss_position_jump_reject_m_{20.0};
  double gnss_yaw_jump_reject_rad_{1.57};

  // Additional GNSS gain boost when dead-reckoning covariance has grown.
  double odom_cov_ref_xy_{0.50};
  double odom_cov_ref_yaw_{0.10};
  double odom_max_cov_xy_{100.0};
  double odom_max_cov_yaw_{10.0};
  double odom_alpha_boost_xy_{0.35};
  double odom_alpha_boost_yaw_{0.20};

  // State
  mutable std::mutex mtx_;
  std::deque<OdomSample> odom_buf_;
  std::optional<AnchorState> anchor_;
  std::optional<AbsolutePoseMeasurement> pending_measurement_;
  GnssStatusState gnss_status_;
  rclcpp::Time last_anchor_stamp_{0, 0, RCL_ROS_TIME};
  double last_alpha_xy_{0.0};
  double last_alpha_yaw_{0.0};
  double last_meas_cov_xy_{0.0};
  double last_meas_cov_yaw_{0.0};
  std::string last_measurement_source_{"none"};
  GnssFixState last_gnss_fix_state_{GnssFixState::UNKNOWN};
  std::string last_rejected_gnss_reason_{"none"};
  rclcpp::Time last_rejected_gnss_stamp_{0, 0, RCL_ROS_TIME};
  std::string last_rejected_gnss_header_frame_;
  std::string last_rejected_gnss_child_frame_;

  // ROS I/F
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<pure_gnss_msgs::msg::GnssFusionInput>::SharedPtr sub_gnss_input_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_legacy_anchor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  rclcpp::TimerBase::SharedPtr timer_pub_;
  rclcpp::TimerBase::SharedPtr timer_heartbeat_;
};

}  // namespace pure_gnss_map_odom_fusion
