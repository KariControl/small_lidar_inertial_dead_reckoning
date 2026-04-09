#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pure_gyro_odometer
{

class GyroOdometerNode : public rclcpp::Node
{
public:
  explicit GyroOdometerNode(const rclcpp::NodeOptions & options);

private:
  struct ImuSample
  {
    rclcpp::Time stamp;
    double gyro_z{0.0};
    Eigen::Vector3d acc{0.0, 0.0, 0.0};
  };

  struct WheelSample
  {
    rclcpp::Time stamp;
    double v_raw{0.0};
  };

  struct DegeneracyInfo
  {
    bool has_hessian{false};
    bool degenerate{false};
    bool wheel_prior_available{false};
    bool wheel_assisted{false};
    int weak_direction_count{0};
    double score{0.0};
    double eig_min{0.0};
    double eig_mid{0.0};
    double eig_max{0.0};
    double wheel_distance{0.0};
    double prior_dx{0.0};
    double prior_dy{0.0};
    double prior_dyaw{0.0};
  };

  struct LidarOdomSample
  {
    rclcpp::Time stamp;
    bool valid{false};
    bool converged{false};
    double fitness{0.0};
    double dx{0.0};
    double dy{0.0};
    double dyaw{0.0};
    double v{0.0};
    double raw_dx{0.0};
    double raw_dy{0.0};
    double raw_dyaw{0.0};
    DegeneracyInfo degeneracy;
  };

  struct ScanFactor
  {
    rclcpp::Time stamp_prev;
    rclcpp::Time stamp_curr;
    double dx{0.0};
    double dy{0.0};
    double dyaw_scan{0.0};
    double dyaw_imu{0.0};
    double fitness{0.0};
    bool converged{false};
    bool stationary{false};
    bool wheel_assisted{false};
  };

  static double normalizeYaw(double a);
  static double yawFromRot(const Eigen::Matrix3d & R);
  static Eigen::Quaterniond quatFromYaw(double yaw);

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onWheelTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onReferencePose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void onPublishTimer();
  void updateStopState(const rclcpp::Time & nowt);
  void publishDiagnostics(const rclcpp::Time & stamp, const std::string & level, const std::string & msg);
  void publishDegeneracyDebug(
    const rclcpp::Time & stamp, const LidarOdomSample & sample, const std::string & guess_mode_used,
    const std::string & next_guess_mode);

  bool computeAccVariance(const rclcpp::Time & nowt, double window_sec, double & out_var) const;
  bool computeImuDeltaYaw(const rclcpp::Time & t0, const rclcpp::Time & t1, double & out_dyaw) const;
  bool computeWheelDistance(const rclcpp::Time & t0, const rclcpp::Time & t1, double & out_dist) const;
  double scanYawWeight(double fitness, bool converged) const;
  bool updateMiniSmootherLocked(const ScanFactor & factor);

  // -------- Parameters --------
  std::string base_frame_;
  std::string odom_frame_;

  std::string imu_topic_;
  std::string wheel_speed_topic_;
  std::string reference_pose_topic_;
  std::string points_topic_;

  std::string out_twist_topic_;
  std::string out_odom_topic_;
  std::string out_stopped_topic_;

  // Publish IMU corrected into base_frame (TF-applied + yaw gyro bias corrected)
  std::string out_imu_topic_;
  bool imu_corrected_enable_{true};
  bool imu_corrected_apply_tf_{true};
  bool imu_corrected_transform_orientation_{false};

  double publish_rate_hz_{50.0};

  // Stop detection
  bool stop_enable_{true};
  double stop_speed_thr_mps_{0.15};
  double stop_gyro_abs_thr_rad_s_{0.05};
  double stop_acc_var_thr_{0.0225};
  double stop_hold_sec_{0.5};

  // Gyro bias
  bool gyro_bias_enable_{true};
  double gyro_bias_tau_sec_{10.0};
  double gyro_bias_max_abs_rad_s_{0.5};

  // Wheel speed
  bool use_wheel_speed_{false};
  double wheel_speed_scale_{1.0};
  double wheel_speed_timeout_sec_{0.2};

  bool wheel_low_speed_enable_{true};
  double wheel_low_speed_deadband_mps_{0.12};
  double wheel_low_speed_acc_thr_mps2_{0.2};
  double wheel_low_speed_blend_{0.5};
  double wheel_low_speed_max_corr_mps_{0.3};

  // Online scale estimation (optional)
  bool wheel_scale_est_enable_{false};
  double wheel_scale_est_tau_sec_{30.0};
  double wheel_scale_est_min_ref_dist_m_{2.0};
  double wheel_scale_est_min_wheel_dist_m_{1.0};
  double wheel_scale_min_{0.5};
  double wheel_scale_max_{2.0};

  // Degeneracy-aware wheel-speed assist (small_gicp Hessian based)
  bool wheel_degeneracy_enable_{true};
  double wheel_degeneracy_yaw_metric_m_{2.0};
  double wheel_degeneracy_rel_eigenvalue_thr_{0.10};
  double wheel_degeneracy_abs_eigenvalue_thr_{0.0};
  double wheel_degeneracy_min_wheel_dist_m_{0.05};
  double wheel_degeneracy_prior_blend_{1.0};
  bool wheel_degeneracy_debug_pub_enable_{false};
  std::string wheel_degeneracy_debug_topic_{"/localization/lidar_degeneracy_debug"};

  // LiDAR odometry
  bool lidar_odom_enable_{true};
  std::string lidar_backend_{"SMALL_GICP"};
  std::string lidar_registration_type_{"VGICP"};
  int lidar_num_threads_{8};
  double lidar_timeout_sec_{0.5};
  double lidar_min_range_m_{2.0};
  double lidar_max_range_m_{80.0};
  double lidar_voxel_leaf_m_{0.4};
  double gicp_max_corr_dist_m_{2.5};
  int gicp_max_iterations_{30};
  double gicp_trans_eps_{1e-3};
  double gicp_rot_eps_{2e-3};
  int gicp_corr_randomness_{20};
  double gicp_fitness_max_{5.0};
  double gicp_voxel_resolution_{1.0};
  bool lidar_pose_se2_enable_{true};
  double lidar_yaw_blend_imu_{0.0};
  bool lidar_guess_use_imu_yaw_only_{true};

  // Lightweight fixed-lag smoothing for local odometry
  bool lidar_smoother_enable_{true};
  int lidar_smoother_window_size_{20};
  double lidar_smoother_w_imu_{2.0};
  double lidar_smoother_w_scan_{1.0};
  double lidar_smoother_lambda_{0.5};
  double lidar_smoother_fitness_sigma_{1.0};
  double lidar_smoother_min_scan_weight_{0.1};
  double lidar_smoother_max_scan_weight_{5.0};
  bool lidar_smoother_zupt_enable_{false};
  double lidar_smoother_zupt_w_trans_{25.0};
  double lidar_smoother_zupt_w_yaw_{25.0};
  bool lidar_smoother_nhc_enable_{false};
  double lidar_smoother_nhc_w_lateral_{2.0};
  double lidar_smoother_nhc_huber_delta_m_{0.1};

  // -------- ROS I/F --------
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_wheel_twist_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_ref_pose_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stopped_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_corrected_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_degeneracy_debug_;

  rclcpp::TimerBase::SharedPtr timer_;

  // -------- State --------
  mutable std::mutex mtx_;

  std::deque<ImuSample> imu_buf_;
  double bg_est_{0.0};
  rclcpp::Time last_imu_stamp_;
  bool has_last_imu_{false};

  // IMU-integrated yaw (bias corrected)
  double yaw_imu_{0.0};

  // Cached IMU extrinsic (base <- imu_frame)
  bool has_imu_extrinsic_{false};
  std::string imu_frame_id_;
  Eigen::Quaterniond q_base_imu_{1.0, 0.0, 0.0, 0.0};
  Eigen::Matrix3d R_base_imu_{Eigen::Matrix3d::Identity()};

  // Wheel speed
  std::deque<WheelSample> wheel_buf_;
  WheelSample last_wheel_;
  bool has_wheel_{false};
  double v_acc_est_{0.0};
  bool has_v_acc_{false};

  // Online scale estimation
  bool has_ref_pose_{false};
  rclcpp::Time last_ref_stamp_;
  double last_ref_x_{0.0};
  double last_ref_y_{0.0};
  double wheel_dist_since_ref_{0.0};

  // LiDAR odometry
  LidarOdomSample last_lidar_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
  Eigen::Matrix4f last_gicp_guess_{Eigen::Matrix4f::Identity()};
  bool has_prev_cloud_{false};
  bool lidar_active_{false};

  // LiDAR extrinsic (base <- scan_frame)
  bool has_scan_extrinsic_{false};
  std::string scan_frame_id_;
  Eigen::Matrix4f T_base_scan_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f T_scan_base_{Eigen::Matrix4f::Identity()};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Integrated odom pose (for debug output)
  rclcpp::Time last_integrate_stamp_;
  bool has_last_integrate_{false};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool has_odom_pose_{false};
  double last_lidar_yaw_imu_{0.0};
  bool has_last_lidar_yaw_imu_{false};
  bool next_icp_force_full_guess_{false};
  std::string last_icp_guess_mode_{"identity"};
  std::string next_icp_guess_mode_{"yaw_only"};

  // Fixed-lag smoothing state for local odometry
  std::deque<ScanFactor> scan_factor_buf_;
  double smoother_base_x_{0.0};
  double smoother_base_y_{0.0};
  double smoother_base_yaw_{0.0};
  bool has_smoother_base_{false};
  Eigen::VectorXd last_smoother_solution_;
  bool has_last_smoother_solution_{false};

  // Stop state
  bool is_stopped_{false};
  bool has_stop_candidate_since_{false};
  rclcpp::Time stop_candidate_since_;
};

}  // namespace pure_gyro_odometer
