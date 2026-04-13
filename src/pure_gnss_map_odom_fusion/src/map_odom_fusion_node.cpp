#include "pure_gnss_map_odom_fusion/map_odom_fusion_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pure_gnss_map_odom_fusion
{

using diagnostic_msgs::msg::DiagnosticStatus;

double MapOdomFusionNode::wrapAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double MapOdomFusionNode::clamp01(double a)
{
  if (a < 0.0) return 0.0;
  if (a > 1.0) return 1.0;
  return a;
}

double MapOdomFusionNode::yawFromIso(const Eigen::Isometry3d & T)
{
  return wrapAngle(std::atan2(T.linear()(1, 0), T.linear()(0, 0)));
}

Eigen::Quaterniond MapOdomFusionNode::quatFromYaw(double yaw)
{
  Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  q.normalize();
  return q;
}

Eigen::Isometry3d MapOdomFusionNode::poseMsgToIso(const geometry_msgs::msg::Pose & p)
{
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (q.norm() < 1e-9) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  return T;
}

geometry_msgs::msg::Pose MapOdomFusionNode::isoToPoseMsg(const Eigen::Isometry3d & T)
{
  geometry_msgs::msg::Pose p;
  p.position.x = T.translation().x();
  p.position.y = T.translation().y();
  p.position.z = T.translation().z();

  Eigen::Quaterniond q(T.linear());
  if (q.norm() < 1e-9) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

Eigen::Isometry3d MapOdomFusionNode::deltaFromTwist(
  const geometry_msgs::msg::Twist & tw,
  double dt)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  if (!std::isfinite(dt) || dt <= 0.0) {
    return T;
  }

  const double vx = tw.linear.x;
  const double vy = tw.linear.y;
  const double wz = tw.angular.z;
  const double dyaw = wz * dt;

  if (std::fabs(dyaw) < 1e-6 || std::fabs(wz) < 1e-6) {
    T.translation() = Eigen::Vector3d(vx * dt, vy * dt, 0.0);
    return T;
  }

  const double s = std::sin(dyaw);
  const double c = std::cos(dyaw);
  T.translation() = Eigen::Vector3d(
    (s * vx - (1.0 - c) * vy) / wz,
    ((1.0 - c) * vx + s * vy) / wz,
    0.0);
  T.linear() = Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return T;
}

MapOdomFusionNode::MapOdomFusionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("map_odom_fusion", options)
{
  map_frame_ = declare_parameter<std::string>("map_frame", map_frame_);
  odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
  base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);

  odom_topic_ = declare_parameter<std::string>("odom_topic", odom_topic_);
  gnss_input_topic_ = declare_parameter<std::string>("gnss_input_topic", gnss_input_topic_);
  gnss_odom_topic_deprecated_ = declare_parameter<std::string>("gnss_odom_topic", gnss_odom_topic_deprecated_);
  gnss_fix_topic_deprecated_ = declare_parameter<std::string>("gnss_fix_topic", gnss_fix_topic_deprecated_);
  legacy_anchor_topic_ = declare_parameter<std::string>("legacy_anchor_topic", legacy_anchor_topic_);
  initialpose_topic_ = declare_parameter<std::string>("initialpose_topic", initialpose_topic_);

  out_pose_topic_ = declare_parameter<std::string>("out_pose_topic", out_pose_topic_);
  out_odom_topic_ = declare_parameter<std::string>("out_odom_topic", out_odom_topic_);

  publish_tf_ = declare_parameter<bool>("publish_tf", publish_tf_);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", publish_rate_hz_);
  heartbeat_hz_ = declare_parameter<double>("heartbeat_hz", heartbeat_hz_);

  odom_buffer_sec_ = declare_parameter<double>("odom_buffer_sec", odom_buffer_sec_);
  odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", odom_timeout_sec_);
  odom_max_extrapolate_sec_ = declare_parameter<double>("odom_max_extrapolate_sec", odom_max_extrapolate_sec_);

  anchor_cov_xy_default_ = declare_parameter<double>("anchor_cov_xy_default", anchor_cov_xy_default_);
  anchor_cov_yaw_default_ = declare_parameter<double>("anchor_cov_yaw_default", anchor_cov_yaw_default_);
  cov_xy_drift_per_sec_ = declare_parameter<double>("cov_xy_drift_per_sec", cov_xy_drift_per_sec_);
  cov_yaw_drift_per_sec_ = declare_parameter<double>("cov_yaw_drift_per_sec", cov_yaw_drift_per_sec_);
  no_fix_cov_drift_scale_ = declare_parameter<double>("no_fix_cov_drift_scale", no_fix_cov_drift_scale_);

  use_gnss_status_ = declare_parameter<bool>("use_gnss_status", use_gnss_status_);
  gnss_status_timeout_sec_ = declare_parameter<double>("gnss_status_timeout_sec", gnss_status_timeout_sec_);
  gnss_fix_min_status_ = declare_parameter<int>("gnss_fix_min_status", gnss_fix_min_status_);
  gnss_init_require_fix_ = declare_parameter<bool>("gnss_init_require_fix", gnss_init_require_fix_);

  gnss_base_alpha_xy_ = declare_parameter<double>("gnss_base_alpha_xy", gnss_base_alpha_xy_);
  gnss_base_alpha_yaw_ = declare_parameter<double>("gnss_base_alpha_yaw", gnss_base_alpha_yaw_);
  gnss_min_alpha_xy_ = declare_parameter<double>("gnss_min_alpha_xy", gnss_min_alpha_xy_);
  gnss_min_alpha_yaw_ = declare_parameter<double>("gnss_min_alpha_yaw", gnss_min_alpha_yaw_);
  gnss_no_fix_ratio_xy_ = declare_parameter<double>("gnss_no_fix_ratio_xy", gnss_no_fix_ratio_xy_);
  gnss_no_fix_ratio_yaw_ = declare_parameter<double>("gnss_no_fix_ratio_yaw", gnss_no_fix_ratio_yaw_);
  gnss_cov_ref_xy_ = declare_parameter<double>("gnss_cov_ref_xy", gnss_cov_ref_xy_);
  gnss_cov_ref_yaw_ = declare_parameter<double>("gnss_cov_ref_yaw", gnss_cov_ref_yaw_);
  gnss_max_cov_xy_ = declare_parameter<double>("gnss_max_cov_xy", gnss_max_cov_xy_);
  gnss_max_cov_yaw_ = declare_parameter<double>("gnss_max_cov_yaw", gnss_max_cov_yaw_);
  gnss_position_jump_reject_m_ = declare_parameter<double>("gnss_position_jump_reject_m", gnss_position_jump_reject_m_);
  gnss_yaw_jump_reject_rad_ = declare_parameter<double>("gnss_yaw_jump_reject_rad", gnss_yaw_jump_reject_rad_);
  odom_cov_ref_xy_ = declare_parameter<double>("odom_cov_ref_xy", odom_cov_ref_xy_);
  odom_cov_ref_yaw_ = declare_parameter<double>("odom_cov_ref_yaw", odom_cov_ref_yaw_);
  odom_max_cov_xy_ = declare_parameter<double>("odom_max_cov_xy", odom_max_cov_xy_);
  odom_max_cov_yaw_ = declare_parameter<double>("odom_max_cov_yaw", odom_max_cov_yaw_);
  odom_alpha_boost_xy_ = declare_parameter<double>("odom_alpha_boost_xy", odom_alpha_boost_xy_);
  odom_alpha_boost_yaw_ = declare_parameter<double>("odom_alpha_boost_yaw", odom_alpha_boost_yaw_);

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&MapOdomFusionNode::onOdom, this, std::placeholders::_1));

  if (!gnss_input_topic_.empty()) {
    sub_gnss_input_ = create_subscription<pure_gnss_msgs::msg::GnssFusionInput>(
      gnss_input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MapOdomFusionNode::onGnssInput, this, std::placeholders::_1));
  } else {
    RCLCPP_WARN(get_logger(), "gnss_input_topic is empty. GNSS anchor updates are disabled.");
  }

  if (!legacy_anchor_topic_.empty()) {
    sub_legacy_anchor_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      legacy_anchor_topic_, 10,
      std::bind(&MapOdomFusionNode::onLegacyAnchorPose, this, std::placeholders::_1));
  }

  if (!initialpose_topic_.empty()) {
    sub_initialpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, 10,
      std::bind(&MapOdomFusionNode::onInitialPose, this, std::placeholders::_1));
  }

  pub_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(out_pose_topic_, 10);
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(out_odom_topic_, 10);
  pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

  if (publish_tf_) {
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  const double pub_period = 1.0 / std::max(1.0, publish_rate_hz_);
  timer_pub_ = create_wall_timer(
    std::chrono::duration<double>(pub_period),
    std::bind(&MapOdomFusionNode::onPublishTimer, this));

  const double heartbeat_period = 1.0 / std::max(0.2, heartbeat_hz_);
  timer_heartbeat_ = create_wall_timer(
    std::chrono::duration<double>(heartbeat_period),
    std::bind(&MapOdomFusionNode::onHeartbeat, this));

  if (!gnss_odom_topic_deprecated_.empty() || !gnss_fix_topic_deprecated_.empty()) {
    RCLCPP_INFO(
      get_logger(),
      "gnss_odom_topic / gnss_fix_topic parameters are deprecated. Using gnss_input_topic=%s",
      gnss_input_topic_.c_str());
  }

  RCLCPP_INFO(
    get_logger(),
    "pure_gnss_map_odom_fusion started. odom=%s gnss_input=%s legacy_anchor=%s initialpose=%s",
    odom_topic_.c_str(),
    gnss_input_topic_.c_str(),
    legacy_anchor_topic_.empty() ? "<disabled>" : legacy_anchor_topic_.c_str(),
    initialpose_topic_.empty() ? "<disabled>" : initialpose_topic_.c_str());
}

std::optional<MapOdomFusionNode::OdomSample> MapOdomFusionNode::latestOdom(
  const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (odom_buf_.empty()) {
    return std::nullopt;
  }

  const auto & s = odom_buf_.back();
  const double age = std::fabs((stamp - s.stamp).seconds());
  if (age > odom_timeout_sec_) {
    return std::nullopt;
  }
  return s;
}

std::optional<MapOdomFusionNode::OdomSample> MapOdomFusionNode::interpolateOdom(
  const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (odom_buf_.empty()) {
    return std::nullopt;
  }

  if ((stamp - odom_buf_.back().stamp).seconds() > odom_max_extrapolate_sec_) {
    return std::nullopt;
  }

  if (stamp <= odom_buf_.front().stamp) {
    const double age = (odom_buf_.front().stamp - stamp).seconds();
    if (age <= odom_max_extrapolate_sec_) {
      return odom_buf_.front();
    }
    return std::nullopt;
  }

  for (std::size_t i = 1; i < odom_buf_.size(); ++i) {
    const auto & a = odom_buf_[i - 1];
    const auto & b = odom_buf_[i];
    if (a.stamp <= stamp && stamp <= b.stamp) {
      const double ta = (a.stamp - rclcpp::Time(0, 0, stamp.get_clock_type())).seconds();
      const double tb = (b.stamp - rclcpp::Time(0, 0, stamp.get_clock_type())).seconds();
      const double ts = (stamp - rclcpp::Time(0, 0, stamp.get_clock_type())).seconds();
      const double ratio = (tb - ta) > 1e-9 ? (ts - ta) / (tb - ta) : 0.0;

      OdomSample out = a;
      out.stamp = stamp;
      out.T_odom_base.translation() =
        (1.0 - ratio) * a.T_odom_base.translation() + ratio * b.T_odom_base.translation();
      const double ya = yawFromIso(a.T_odom_base);
      const double yb = yawFromIso(b.T_odom_base);
      const double y = wrapAngle(ya + ratio * wrapAngle(yb - ya));
      out.T_odom_base.linear() =
        Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      out.twist = (ratio < 0.5) ? a.twist : b.twist;
      return out;
    }
  }

  const auto & s = odom_buf_.back();
  const double dt = (stamp - s.stamp).seconds();
  if (dt >= 0.0 && dt <= odom_max_extrapolate_sec_) {
    OdomSample out = s;
    out.stamp = stamp;
    out.T_odom_base = s.T_odom_base * deltaFromTwist(s.twist, dt);
    return out;
  }

  return std::nullopt;
}

MapOdomFusionNode::AbsolutePoseMeasurement MapOdomFusionNode::measurementFromGnssInput(
  const pure_gnss_msgs::msg::GnssFusionInput & msg) const
{
  AbsolutePoseMeasurement meas;
  meas.stamp = rclcpp::Time(msg.header.stamp);
  if (meas.stamp.nanoseconds() == 0) {
    meas.stamp = rclcpp::Time(msg.odom.header.stamp);
  }
  if (meas.stamp.nanoseconds() == 0) {
    meas.stamp = now();
  }
  meas.T_map_base = poseMsgToIso(msg.odom.pose.pose);
  meas.source = "gnss";
  meas.has_gnss_fix_status = true;
  meas.gnss_fix_status = static_cast<int>(msg.fix_status);

  auto sanitize = [](double v) {
      if (!std::isfinite(v) || v < 0.0) {
        return 1.0e6;
      }
      return v;
    };

  const double cov_x = sanitize(msg.odom.pose.covariance[0]);
  const double cov_y = sanitize(msg.odom.pose.covariance[7]);
  meas.cov_xy = 0.5 * (cov_x + cov_y);
  meas.cov_yaw = sanitize(msg.odom.pose.covariance[35]);
  meas.yaw_valid = std::isfinite(meas.cov_yaw) && meas.cov_yaw < gnss_max_cov_yaw_;
  return meas;
}

MapOdomFusionNode::AbsolutePoseMeasurement MapOdomFusionNode::measurementFromPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
  const std::string & source) const
{
  AbsolutePoseMeasurement meas;
  meas.stamp = rclcpp::Time(msg.header.stamp);
  meas.T_map_base = poseMsgToIso(msg.pose.pose);
  meas.cov_xy = msg.pose.covariance[0] > 0.0 ? msg.pose.covariance[0] : anchor_cov_xy_default_;
  meas.cov_yaw = msg.pose.covariance[35] > 0.0 ? msg.pose.covariance[35] : anchor_cov_yaw_default_;
  meas.yaw_valid = true;
  meas.source = source;
  return meas;
}

void MapOdomFusionNode::handleMeasurement(const AbsolutePoseMeasurement & meas, bool allow_pending)
{
  auto od = interpolateOdom(meas.stamp);
  if (!od) {
    od = latestOdom(meas.stamp);
  }

  if (!od) {
    if (allow_pending) {
      std::lock_guard<std::mutex> lk(mtx_);
      pending_measurement_ = meas;
    }
    return;
  }

  applyMeasurement(meas, *od);
}

bool MapOdomFusionNode::resolvePendingMeasurement(const rclcpp::Time & nowt)
{
  (void)nowt;
  std::optional<AbsolutePoseMeasurement> pending;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pending = pending_measurement_;
  }

  if (!pending) {
    return false;
  }

  auto od = interpolateOdom(pending->stamp);
  if (!od) {
    od = latestOdom(pending->stamp);
  }
  if (!od) {
    return false;
  }

  applyMeasurement(*pending, *od);
  return true;
}

MapOdomFusionNode::GnssFixState MapOdomFusionNode::gnssFixState(const rclcpp::Time & stamp) const
{
  if (!use_gnss_status_) {
    return GnssFixState::GOOD;
  }

  std::lock_guard<std::mutex> lk(mtx_);
  if (!gnss_status_.valid) {
    return GnssFixState::UNKNOWN;
  }

  const rclcpp::Time ref = (stamp.nanoseconds() != 0) ? stamp : now();
  if (std::fabs((ref - gnss_status_.stamp).seconds()) > gnss_status_timeout_sec_) {
    return GnssFixState::BAD;
  }

  return gnssFixStateFromStatus(gnss_status_.status);
}

MapOdomFusionNode::GnssFixState MapOdomFusionNode::gnssFixStateFromStatus(int status) const
{
  if (!use_gnss_status_) {
    return GnssFixState::GOOD;
  }
  return status >= gnss_fix_min_status_ ? GnssFixState::GOOD : GnssFixState::BAD;
}

double MapOdomFusionNode::qualityFromCovariance(double cov, double ref, double max) const
{
  if (!std::isfinite(cov)) {
    return 0.0;
  }

  cov = std::max(0.0, cov);
  if (max > 0.0 && cov >= max) {
    return 0.0;
  }
  if (ref <= 1e-9) {
    return 1.0;
  }
  return clamp01(ref / (ref + cov));
}

void MapOdomFusionNode::applyMeasurement(const AbsolutePoseMeasurement & meas, const OdomSample & od)
{
  AnchorState current_anchor;
  bool has_anchor = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (anchor_) {
      current_anchor = *anchor_;
      has_anchor = true;
    }
  }

  const Eigen::Isometry3d T_map_base_pred =
    has_anchor ? (current_anchor.T_map_odom * od.T_odom_base) : od.T_odom_base;
  const Eigen::Vector3d pred_t = T_map_base_pred.translation();
  const Eigen::Vector3d meas_t = meas.T_map_base.translation();
  const double pred_yaw = yawFromIso(T_map_base_pred);
  const double meas_yaw = yawFromIso(meas.T_map_base);
  const double odom_cov_xy_since_anchor =
    has_anchor ? std::max(0.0, od.cov_xy_total - current_anchor.odom_cov_xy_ref) : od.cov_xy_total;
  const double odom_cov_yaw_since_anchor =
    has_anchor ? std::max(0.0, od.cov_yaw_total - current_anchor.odom_cov_yaw_ref) : od.cov_yaw_total;

  GnssFixState fix_state = GnssFixState::GOOD;
  double alpha_xy = 1.0;
  double alpha_yaw = meas.yaw_valid ? 1.0 : 0.0;

  if (meas.source == "gnss") {
    fix_state = meas.has_gnss_fix_status ?
      gnssFixStateFromStatus(meas.gnss_fix_status) : gnssFixState(meas.stamp);

    alpha_xy = clamp01(gnss_base_alpha_xy_ * qualityFromCovariance(meas.cov_xy, gnss_cov_ref_xy_, gnss_max_cov_xy_));
    alpha_yaw = meas.yaw_valid ? clamp01(gnss_base_alpha_yaw_ * qualityFromCovariance(meas.cov_yaw, gnss_cov_ref_yaw_, gnss_max_cov_yaw_)) : 0.0;

    const double innovation_xy = (meas_t.head<2>() - pred_t.head<2>()).norm();
    if (gnss_position_jump_reject_m_ > 1e-6 && innovation_xy > gnss_position_jump_reject_m_) {
      alpha_xy *= gnss_position_jump_reject_m_ / innovation_xy;
    }

    if (meas.yaw_valid) {
      const double innovation_yaw = std::fabs(wrapAngle(meas_yaw - pred_yaw));
      if (gnss_yaw_jump_reject_rad_ > 1e-6 && innovation_yaw > gnss_yaw_jump_reject_rad_) {
        alpha_yaw *= gnss_yaw_jump_reject_rad_ / innovation_yaw;
      }
    }

    if (fix_state == GnssFixState::BAD) {
      alpha_xy *= gnss_no_fix_ratio_xy_;
      alpha_yaw *= gnss_no_fix_ratio_yaw_;
    } else if (fix_state == GnssFixState::GOOD) {
      alpha_xy = std::max(alpha_xy, gnss_min_alpha_xy_);
      if (meas.yaw_valid) {
        alpha_yaw = std::max(alpha_yaw, gnss_min_alpha_yaw_);
      }
    }

    if (fix_state == GnssFixState::GOOD) {
      const double odom_need_xy =
        1.0 - qualityFromCovariance(odom_cov_xy_since_anchor, odom_cov_ref_xy_, odom_max_cov_xy_);
      alpha_xy = clamp01(alpha_xy + odom_alpha_boost_xy_ * odom_need_xy);
      if (meas.yaw_valid) {
        const double odom_need_yaw =
          1.0 - qualityFromCovariance(odom_cov_yaw_since_anchor, odom_cov_ref_yaw_, odom_max_cov_yaw_);
        alpha_yaw = clamp01(alpha_yaw + odom_alpha_boost_yaw_ * odom_need_yaw);
      }
    }

    alpha_xy = clamp01(alpha_xy);
    alpha_yaw = clamp01(alpha_yaw);
  }

  if (!has_anchor) {
    if (meas.source == "gnss" && gnss_init_require_fix_ && fix_state == GnssFixState::BAD) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "GNSS received but fix state is bad. Waiting for a valid fix before creating anchor.");
      std::lock_guard<std::mutex> lk(mtx_);
      pending_measurement_ = meas;
      last_alpha_xy_ = alpha_xy;
      last_alpha_yaw_ = alpha_yaw;
      last_meas_cov_xy_ = meas.cov_xy;
      last_meas_cov_yaw_ = meas.cov_yaw;
      last_measurement_source_ = meas.source;
      last_gnss_fix_state_ = fix_state;
      return;
    }

    Eigen::Isometry3d T_map_base_new = Eigen::Isometry3d::Identity();
    T_map_base_new.translation() = meas_t;
    const double init_yaw = meas.yaw_valid ? meas_yaw : yawFromIso(od.T_odom_base);
    T_map_base_new.linear() = Eigen::AngleAxisd(init_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    AnchorState new_anchor;
    new_anchor.stamp = meas.stamp;
    new_anchor.T_map_odom = T_map_base_new * od.T_odom_base.inverse();
    new_anchor.cov_xy = std::isfinite(meas.cov_xy) && meas.cov_xy >= 0.0 ? meas.cov_xy : anchor_cov_xy_default_;
    new_anchor.cov_yaw = (meas.yaw_valid && std::isfinite(meas.cov_yaw) && meas.cov_yaw >= 0.0) ? meas.cov_yaw : anchor_cov_yaw_default_;
    new_anchor.odom_cov_xy_ref = od.cov_xy_total;
    new_anchor.odom_cov_yaw_ref = od.cov_yaw_total;
    new_anchor.source = meas.source + "_init";

    {
      std::lock_guard<std::mutex> lk(mtx_);
      anchor_ = new_anchor;
      pending_measurement_.reset();
      last_anchor_stamp_ = meas.stamp;
      last_alpha_xy_ = 1.0;
      last_alpha_yaw_ = meas.yaw_valid ? 1.0 : 0.0;
      last_meas_cov_xy_ = meas.cov_xy;
      last_meas_cov_yaw_ = meas.cov_yaw;
      last_measurement_source_ = meas.source;
      last_gnss_fix_state_ = fix_state;
    }
    return;
  }

  if (meas.source == "gnss" && alpha_xy <= 1e-6 && alpha_yaw <= 1e-6) {
    std::lock_guard<std::mutex> lk(mtx_);
    pending_measurement_.reset();
    last_alpha_xy_ = alpha_xy;
    last_alpha_yaw_ = alpha_yaw;
    last_meas_cov_xy_ = meas.cov_xy;
    last_meas_cov_yaw_ = meas.cov_yaw;
    last_measurement_source_ = meas.source;
    last_gnss_fix_state_ = fix_state;
    return;
  }

  Eigen::Vector3d upd_t = pred_t;
  upd_t.x() += alpha_xy * (meas_t.x() - pred_t.x());
  upd_t.y() += alpha_xy * (meas_t.y() - pred_t.y());
  upd_t.z() += alpha_xy * (meas_t.z() - pred_t.z());

  double upd_yaw = pred_yaw;
  if (meas.yaw_valid) {
    upd_yaw = wrapAngle(pred_yaw + alpha_yaw * wrapAngle(meas_yaw - pred_yaw));
  }

  Eigen::Isometry3d T_map_base_upd = Eigen::Isometry3d::Identity();
  T_map_base_upd.translation() = upd_t;
  T_map_base_upd.linear() = Eigen::AngleAxisd(upd_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const double meas_cov_xy =
    (std::isfinite(meas.cov_xy) && meas.cov_xy >= 0.0) ? meas.cov_xy : anchor_cov_xy_default_;
  const double meas_cov_yaw =
    (meas.yaw_valid && std::isfinite(meas.cov_yaw) && meas.cov_yaw >= 0.0) ? meas.cov_yaw : current_anchor.cov_yaw;

  AnchorState new_anchor = current_anchor;
  new_anchor.stamp = meas.stamp;
  new_anchor.T_map_odom = T_map_base_upd * od.T_odom_base.inverse();
  new_anchor.odom_cov_xy_ref = od.cov_xy_total;
  new_anchor.odom_cov_yaw_ref = od.cov_yaw_total;
  if (meas.source == "gnss") {
    new_anchor.cov_xy = (1.0 - alpha_xy) * current_anchor.cov_xy + alpha_xy * meas_cov_xy;
    new_anchor.cov_yaw = (1.0 - alpha_yaw) * current_anchor.cov_yaw + alpha_yaw * meas_cov_yaw;
  } else {
    new_anchor.cov_xy = meas_cov_xy;
    new_anchor.cov_yaw = meas.yaw_valid ? meas_cov_yaw : current_anchor.cov_yaw;
  }
  new_anchor.source = meas.source;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    anchor_ = new_anchor;
    pending_measurement_.reset();
    last_anchor_stamp_ = meas.stamp;
    last_alpha_xy_ = alpha_xy;
    last_alpha_yaw_ = alpha_yaw;
    last_meas_cov_xy_ = meas.cov_xy;
    last_meas_cov_yaw_ = meas.cov_yaw;
    last_measurement_source_ = meas.source;
    last_gnss_fix_state_ = fix_state;
  }
}

void MapOdomFusionNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  OdomSample s;
  s.stamp = rclcpp::Time(msg->header.stamp);
  s.T_odom_base = poseMsgToIso(msg->pose.pose);
  s.twist = msg->twist.twist;
  auto sanitize_cov = [](double v) {
      if (!std::isfinite(v) || v < 0.0) {
        return 0.0;
      }
      return v;
    };
  s.cov_xy_total = 0.5 * (sanitize_cov(msg->pose.covariance[0]) + sanitize_cov(msg->pose.covariance[7]));
  s.cov_yaw_total = sanitize_cov(msg->pose.covariance[35]);

  {
    std::lock_guard<std::mutex> lk(mtx_);
    odom_buf_.push_back(s);
    while (!odom_buf_.empty() && (s.stamp - odom_buf_.front().stamp).seconds() > odom_buffer_sec_) {
      odom_buf_.pop_front();
    }
  }

  resolvePendingMeasurement(s.stamp);
  publishFused(s.stamp, s);
}

void MapOdomFusionNode::onGnssInput(const pure_gnss_msgs::msg::GnssFusionInput::SharedPtr msg)
{
  const rclcpp::Time stamp =
    (rclcpp::Time(msg->header.stamp).nanoseconds() == 0) ? now() : rclcpp::Time(msg->header.stamp);

  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!gnss_status_.valid || stamp >= gnss_status_.stamp) {
      gnss_status_.stamp = stamp;
      gnss_status_.status = static_cast<int>(msg->fix_status);
      gnss_status_.valid = true;
    }
  }

  if (!msg->has_odom) {
    return;
  }

  const auto & odom = msg->odom;
  if (odom.header.frame_id != map_frame_) {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_rejected_gnss_reason_ = "header_frame_mismatch";
      last_rejected_gnss_stamp_ = stamp;
      last_rejected_gnss_header_frame_ = odom.header.frame_id;
      last_rejected_gnss_child_frame_ = odom.child_frame_id;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring gnss input: odom.header.frame_id='%s' but map_frame='%s'. GNSS odom must already be map->base.",
      odom.header.frame_id.c_str(), map_frame_.c_str());
    return;
  }

  if (odom.child_frame_id != base_frame_) {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_rejected_gnss_reason_ = "child_frame_mismatch";
      last_rejected_gnss_stamp_ = stamp;
      last_rejected_gnss_header_frame_ = odom.header.frame_id;
      last_rejected_gnss_child_frame_ = odom.child_frame_id;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring gnss input: odom.child_frame_id='%s' but base_frame='%s'. GNSS odom must already be map->base.",
      odom.child_frame_id.c_str(), base_frame_.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_rejected_gnss_reason_ = "none";
    last_rejected_gnss_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_rejected_gnss_header_frame_.clear();
    last_rejected_gnss_child_frame_.clear();
  }

  handleMeasurement(measurementFromGnssInput(*msg));
}

void MapOdomFusionNode::onLegacyAnchorPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  handleMeasurement(measurementFromPose(*msg, "legacy_anchor"));
}

void MapOdomFusionNode::onInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  handleMeasurement(measurementFromPose(*msg, "initialpose"));
}

void MapOdomFusionNode::publishFused(
  const rclcpp::Time & stamp,
  const std::optional<OdomSample> & od_in)
{
  AnchorState anchor;
  rclcpp::Time anchor_stamp(0, 0, RCL_ROS_TIME);
  bool has_anchor = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (anchor_) {
      anchor = *anchor_;
      anchor_stamp = last_anchor_stamp_;
      has_anchor = true;
    }
  }

  if (!has_anchor) {
    return;
  }

  auto od = od_in ? od_in : interpolateOdom(stamp);
  if (!od) {
    od = latestOdom(stamp);
  }
  if (!od) {
    return;
  }

  const Eigen::Isometry3d T_map_base = anchor.T_map_odom * od->T_odom_base;
  const double anchor_age = std::max(0.0, (stamp - anchor_stamp).seconds());
  const GnssFixState fix_state = gnssFixState(stamp);
  const double drift_scale = (fix_state == GnssFixState::BAD) ? no_fix_cov_drift_scale_ : 1.0;
  const double odom_cov_xy_since_anchor = std::max(0.0, od->cov_xy_total - anchor.odom_cov_xy_ref);
  const double odom_cov_yaw_since_anchor = std::max(0.0, od->cov_yaw_total - anchor.odom_cov_yaw_ref);
  const double cov_xy = anchor.cov_xy + odom_cov_xy_since_anchor + cov_xy_drift_per_sec_ * drift_scale * anchor_age;
  const double cov_yaw = anchor.cov_yaw + odom_cov_yaw_since_anchor + cov_yaw_drift_per_sec_ * drift_scale * anchor_age;

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = map_frame_;
  pose.pose.pose = isoToPoseMsg(T_map_base);
  std::fill(pose.pose.covariance.begin(), pose.pose.covariance.end(), 0.0);
  pose.pose.covariance[0] = cov_xy;
  pose.pose.covariance[7] = cov_xy;
  pose.pose.covariance[14] = 1.0e6;
  pose.pose.covariance[21] = 1.0e6;
  pose.pose.covariance[28] = 1.0e6;
  pose.pose.covariance[35] = cov_yaw;
  pub_pose_->publish(pose);

  nav_msgs::msg::Odometry odom;
  odom.header = pose.header;
  odom.child_frame_id = base_frame_;
  odom.pose = pose.pose;
  odom.twist.twist = od->twist;
  odom.twist.covariance = {};
  odom.twist.covariance[0] = 1.0e-3;
  odom.twist.covariance[35] = std::max(1.0e-5, 0.25 * cov_yaw);
  pub_odom_->publish(odom);

  if (publish_tf_ && tf_br_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header = pose.header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = pose.pose.pose.position.x;
    tf.transform.translation.y = pose.pose.pose.position.y;
    tf.transform.translation.z = pose.pose.pose.position.z;
    tf.transform.rotation = pose.pose.pose.orientation;
    tf_br_->sendTransform(tf);
  }
}

void MapOdomFusionNode::onPublishTimer()
{
  publishFused(now());
}

void MapOdomFusionNode::publishDiag(uint8_t level, const std::string & msg)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.level = level;
  st.name = "localization/gnss_map_odom_fusion";
  st.message = msg;
  st.hardware_id = "none";

  auto add = [&](const std::string & k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = v;
      st.values.push_back(kv);
    };

  std::optional<AnchorState> anchor;
  std::optional<AbsolutePoseMeasurement> pending_measurement;
  std::size_t odom_size = 0;
  rclcpp::Time latest_odom_stamp(0, 0, RCL_ROS_TIME);
  bool has_latest_odom = false;
  bool pending = false;
  double last_alpha_xy = 0.0;
  double last_alpha_yaw = 0.0;
  double last_meas_cov_xy = 0.0;
  double last_meas_cov_yaw = 0.0;
  std::string last_measurement_source = "none";
  GnssFixState last_fix_state = GnssFixState::UNKNOWN;
  GnssStatusState gnss_status;
  rclcpp::Time anchor_stamp(0, 0, RCL_ROS_TIME);
  std::string last_rejected_gnss_reason = "none";
  rclcpp::Time last_rejected_gnss_stamp(0, 0, RCL_ROS_TIME);
  std::string last_rejected_gnss_header_frame;
  std::string last_rejected_gnss_child_frame;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (anchor_) {
      anchor = *anchor_;
      anchor_stamp = last_anchor_stamp_;
    }
    if (!odom_buf_.empty()) {
      latest_odom_stamp = odom_buf_.back().stamp;
      has_latest_odom = true;
    }
    odom_size = odom_buf_.size();
    pending = pending_measurement_.has_value();
    pending_measurement = pending_measurement_;
    last_alpha_xy = last_alpha_xy_;
    last_alpha_yaw = last_alpha_yaw_;
    last_meas_cov_xy = last_meas_cov_xy_;
    last_meas_cov_yaw = last_meas_cov_yaw_;
    last_measurement_source = last_measurement_source_;
    last_fix_state = last_gnss_fix_state_;
    gnss_status = gnss_status_;
    last_rejected_gnss_reason = last_rejected_gnss_reason_;
    last_rejected_gnss_stamp = last_rejected_gnss_stamp_;
    last_rejected_gnss_header_frame = last_rejected_gnss_header_frame_;
    last_rejected_gnss_child_frame = last_rejected_gnss_child_frame_;
  }

  add("anchor.valid", anchor ? "true" : "false");
  add("pending_measurement", pending ? "true" : "false");
  add("odom_buf.size", std::to_string(odom_size));
  add("has_latest_odom", has_latest_odom ? "true" : "false");
  if (has_latest_odom) {
    add("latest_odom_age_sec", std::to_string(std::fabs((now() - latest_odom_stamp).seconds())));
  }
  if (pending_measurement) {
    add("pending_measurement.source", pending_measurement->source);
    if (has_latest_odom) {
      add(
        "pending_vs_latest_odom_dt_sec",
        std::to_string(std::fabs((pending_measurement->stamp - latest_odom_stamp).seconds())));
    }
  }
  add("last_measurement.source", last_measurement_source);
  add("last_alpha_xy", std::to_string(last_alpha_xy));
  add("last_alpha_yaw", std::to_string(last_alpha_yaw));
  add("last_meas_cov_xy", std::to_string(last_meas_cov_xy));
  add("last_meas_cov_yaw", std::to_string(last_meas_cov_yaw));

  auto fixStateToString = [](GnssFixState s) {
    if (s == GnssFixState::GOOD) {
      return std::string("good");
    }
    if (s == GnssFixState::BAD) {
      return std::string("bad");
    }
    return std::string("unknown");
  };
  const GnssFixState live_fix_state = gnssFixState(now());
  add("gnss.fix_state", fixStateToString(last_fix_state));
  add("gnss.live_fix_state", fixStateToString(live_fix_state));
  add(
    "gnss.next_gain_reduction",
    live_fix_state == GnssFixState::BAD ? "true" : "false");
  add("gnss.last_rejected_reason", last_rejected_gnss_reason);
  if (last_rejected_gnss_stamp.nanoseconds() != 0) {
    add("gnss.last_rejected_age_sec", std::to_string(std::fabs((now() - last_rejected_gnss_stamp).seconds())));
  }
  if (!last_rejected_gnss_header_frame.empty()) {
    add("gnss.last_rejected_header_frame", last_rejected_gnss_header_frame);
  }
  if (!last_rejected_gnss_child_frame.empty()) {
    add("gnss.last_rejected_child_frame", last_rejected_gnss_child_frame);
  }
  add("gnss.status.valid", gnss_status.valid ? "true" : "false");
  if (gnss_status.valid) {
    add("gnss.status.code", std::to_string(gnss_status.status));
    add("gnss.status_age_sec", std::to_string(std::fabs((now() - gnss_status.stamp).seconds())));
  }

  if (anchor) {
    add("anchor.source", anchor->source);
    add("anchor_age_sec", std::to_string(std::max(0.0, (now() - anchor_stamp).seconds())));

    auto od = latestOdom(now());
    if (od) {
      const Eigen::Isometry3d T_map_base = anchor->T_map_odom * od->T_odom_base;
      const double odom_cov_xy_since_anchor = std::max(0.0, od->cov_xy_total - anchor->odom_cov_xy_ref);
      const double odom_cov_yaw_since_anchor = std::max(0.0, od->cov_yaw_total - anchor->odom_cov_yaw_ref);
      add("x", std::to_string(T_map_base.translation().x()));
      add("y", std::to_string(T_map_base.translation().y()));
      add("yaw", std::to_string(yawFromIso(T_map_base)));
      add("odom_cov_xy_since_anchor", std::to_string(odom_cov_xy_since_anchor));
      add("odom_cov_yaw_since_anchor", std::to_string(odom_cov_yaw_since_anchor));
    }
  }

  arr.status.push_back(st);
  pub_diag_->publish(arr);
}

void MapOdomFusionNode::onHeartbeat()
{
  bool has_anchor = false;
  bool pending = false;
  bool has_latest_odom = false;
  rclcpp::Time latest_odom_stamp(0, 0, RCL_ROS_TIME);
  rclcpp::Time pending_stamp(0, 0, RCL_ROS_TIME);
  std::string last_rejected_gnss_reason = "none";
  rclcpp::Time last_rejected_gnss_stamp(0, 0, RCL_ROS_TIME);
  {
    std::lock_guard<std::mutex> lk(mtx_);
    has_anchor = anchor_.has_value();
    pending = pending_measurement_.has_value();
    if (!odom_buf_.empty()) {
      has_latest_odom = true;
      latest_odom_stamp = odom_buf_.back().stamp;
    }
    if (pending_measurement_) {
      pending_stamp = pending_measurement_->stamp;
    }
    last_rejected_gnss_reason = last_rejected_gnss_reason_;
    last_rejected_gnss_stamp = last_rejected_gnss_stamp_;
  }

  GnssFixState last_fused_fix_state = GnssFixState::UNKNOWN;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_fused_fix_state = last_gnss_fix_state_;
  }
  const GnssFixState live_fix_state = gnssFixState(now());
  const bool recent_gnss_reject =
    last_rejected_gnss_reason != "none" &&
    last_rejected_gnss_stamp.nanoseconds() != 0 &&
    std::fabs((now() - last_rejected_gnss_stamp).seconds()) <= 5.0;
  if (recent_gnss_reject) {
    publishDiag(DiagnosticStatus::WARN, "rejecting gnss input frame mismatch");
  } else if (pending) {
    const bool severe_stamp_gap = has_latest_odom &&
      std::fabs((pending_stamp - latest_odom_stamp).seconds()) > std::max(60.0, odom_timeout_sec_ * 10.0);
    if (severe_stamp_gap) {
      publishDiag(DiagnosticStatus::WARN, "measurement/odom time-base mismatch (check use_sim_time or message stamps)");
    } else {
      publishDiag(DiagnosticStatus::WARN, "waiting measurement/odom sync");
    }
  } else if (!has_anchor) {
    publishDiag(DiagnosticStatus::WARN, "no anchor yet");
  } else if (last_fused_fix_state == GnssFixState::BAD) {
    publishDiag(DiagnosticStatus::WARN, "last fused GNSS update used reduced gain (no-fix/stale at measurement time)");
  } else if (live_fix_state == GnssFixState::BAD) {
    publishDiag(DiagnosticStatus::WARN, "live GNSS status is no-fix/stale; next GNSS update will use reduced gain");
  } else {
    publishDiag(DiagnosticStatus::OK, "fused pose OK");
  }
}

}  // namespace pure_gnss_map_odom_fusion
