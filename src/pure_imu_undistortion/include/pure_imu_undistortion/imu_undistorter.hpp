#ifndef PURE_IMU_UNDISTORTION__IMU_UNDISTORTER_HPP_
#define PURE_IMU_UNDISTORTION__IMU_UNDISTORTER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pure_imu_undistortion
{

class ImuUndistorter : public rclcpp::Node
{
public:
  explicit ImuUndistorter(const rclcpp::NodeOptions & options);

private:
  // ===== Parameters =====
  std::string base_frame_;
  std::string imu_frame_;   // empty => use msg frame
  std::string scan_frame_;  // empty => use msg frame

  std::string points_in_topic_;
  std::string points_out_topic_;
  std::string imu_topic_;
  std::string twist_topic_;   // optional

  // time field candidates (mixed)
  std::vector<std::string> time_fields_;
  bool prefer_relative_time_;
  double time_scale_;  // 0 => auto
  double fallback_scan_period_;
  bool cloud_stamp_is_start_;
  std::string reference_time_; // "start" or "end"

  // buffers
  double imu_buffer_sec_;
  double twist_buffer_sec_;
  double max_imu_gap_sec_;
  double max_time_offset_sec_;

  // translation (optional)
  bool use_translation_;
  bool use_twist_speed_;
  double default_speed_mps_;   // if use_translation true but twist not available => this constant used
  double max_speed_mps_;

  // output/diagnostics
  bool publish_diagnostics_;
  int diag_throttle_ms_;

  // ===== ROS =====
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool has_T_base_scan_{false};
  Eigen::Isometry3d T_base_scan_{Eigen::Isometry3d::Identity()}; // base <- scan

  bool has_T_base_imu_{false};
  Eigen::Isometry3d T_base_imu_{Eigen::Isometry3d::Identity()};  // base <- imu

  // ===== Buffers =====
  struct ImuSample
  {
    rclcpp::Time stamp;
    Eigen::Vector3d gyro; // rad/s in imu frame
  };

  struct TwistSample
  {
    rclcpp::Time stamp;
    double speed_mps; // forward speed in base frame
  };

  mutable std::mutex mtx_;
  std::deque<ImuSample> imu_buf_;
  std::deque<TwistSample> twist_buf_;

  // ===== Internal helpers =====
  static double toSec(const rclcpp::Time & t);
  static rclcpp::Time fromSec(const rclcpp::Clock & clock, double sec);

  bool ensureStaticTf(const std::string & scan_frame, const std::string & imu_frame);

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void pruneBuffers(const rclcpp::Time & nowt);

  // point time extraction
  struct PointTimeInfo
  {
    bool has_time_field{false};
    std::string field_name;
    uint8_t datatype{0};
    uint32_t offset{0};

    bool used_linear_fallback{false};

    double scan_period{0.0};
    double time_scale{1.0};      // raw -> sec
    bool interpreted_as_relative{true};
    double t0_sec{0.0};          // scan start
    double t_ref_sec{0.0};       // reference time (start or end)
  };

  bool findTimeField(const sensor_msgs::msg::PointCloud2 & msg,
                     std::string & out_name,
                     uint8_t & out_datatype,
                     uint32_t & out_offset) const;

  bool readFieldAsDouble(const uint8_t * ptr, uint8_t datatype, double & v) const;

  double estimateTimeScale(double raw_range, double scan_period) const;

  PointTimeInfo preparePointTimeInfo(const sensor_msgs::msg::PointCloud2 & msg) const;

  bool computePointDtSec(const sensor_msgs::msg::PointCloud2 & msg,
                         const PointTimeInfo & ti,
                         int point_index,
                         int point_count,
                         const uint8_t * point_ptr,
                         double & out_dt) const;

  // build relative pose trajectory from IMU(+optional speed)
  struct PoseSample
  {
    double t_sec{0.0};               // absolute time (sec)
    Eigen::Quaterniond q_WB;         // orientation of base in W
    Eigen::Vector3d p_WB{0,0,0};     // position of base in W (optional)
  };

  bool buildBaseTrajectory(double t0_sec, double t1_sec,
                           std::vector<PoseSample> & out_traj,
                           std::string & out_reason);

  bool orientationAt(const std::vector<PoseSample> & traj, double t_sec, Eigen::Quaterniond & q_WB) const;
  bool positionAt(const std::vector<PoseSample> & traj, double t_sec, Eigen::Vector3d & p_WB) const;

  // deskew core
  bool deskewPointCloud(const sensor_msgs::msg::PointCloud2 & in,
                        sensor_msgs::msg::PointCloud2 & out,
                        std::string & out_reason);

  bool findXYZOffsets(const sensor_msgs::msg::PointCloud2 & msg,
                      uint32_t & off_x, uint32_t & off_y, uint32_t & off_z,
                      uint8_t & dt_x, uint8_t & dt_y, uint8_t & dt_z) const;

  // diagnostics
  void publishDiag(const rclcpp::Time & stamp,
                   const std::string & level,
                   const std::string & msg,
                   const PointTimeInfo & ti) const;

  static uint8_t toDiagLevel(const std::string & level);;
};

}  // namespace pure_imu_undistortion

#endif
