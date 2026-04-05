#include "pure_imu_undistortion/imu_undistorter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace pure_imu_undistortion
{

double ImuUndistorter::toSec(const rclcpp::Time & t)
{
  return static_cast<double>(t.nanoseconds()) * 1e-9;
}

rclcpp::Time ImuUndistorter::fromSec(const rclcpp::Clock & clock, double sec)
{
  const int64_t ns = static_cast<int64_t>(sec * 1e9);
  return rclcpp::Time(ns, clock.get_clock_type());
}

uint8_t ImuUndistorter::toDiagLevel(const std::string & level)
{
  if (level == "OK" || level == "ok") {
    return diagnostic_msgs::msg::DiagnosticStatus::OK;
  }
  if (level == "WARN" || level == "warn") {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }
  if (level == "ERROR" || level == "error") {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  if (level == "STALE" || level == "stale") {
    return diagnostic_msgs::msg::DiagnosticStatus::STALE;
  }
  // デフォルトはWARN（運用上無難）
  return diagnostic_msgs::msg::DiagnosticStatus::WARN;
}

ImuUndistorter::ImuUndistorter(const rclcpp::NodeOptions & options)
: rclcpp::Node("imu_undistorter", options)
{
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "");
  scan_frame_ = declare_parameter<std::string>("scan_frame", "");

  points_in_topic_ = declare_parameter<std::string>("points_in_topic", "/points_raw");
  points_out_topic_ = declare_parameter<std::string>("points_out_topic", "/points_undistorted");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
  twist_topic_ = declare_parameter<std::string>("twist_topic", ""); // empty => disabled

  time_fields_ = declare_parameter<std::vector<std::string>>(
    "time_fields", std::vector<std::string>{"time","t","timestamp","offset_time","time_stamp"});

  prefer_relative_time_ = declare_parameter<bool>("prefer_relative_time", true);
  time_scale_ = declare_parameter<double>("time_scale", 0.0); // 0 => auto
  fallback_scan_period_ = declare_parameter<double>("fallback_scan_period", 0.1);
  cloud_stamp_is_start_ = declare_parameter<bool>("cloud_stamp_is_start", true);
  reference_time_ = declare_parameter<std::string>("reference_time", "start"); // start|end

  imu_buffer_sec_ = declare_parameter<double>("imu_buffer_sec", 2.0);
  twist_buffer_sec_ = declare_parameter<double>("twist_buffer_sec", 2.0);
  max_imu_gap_sec_ = declare_parameter<double>("max_imu_gap_sec", 0.02);
  max_time_offset_sec_ = declare_parameter<double>("max_time_offset_sec", 0.2);

  use_translation_ = declare_parameter<bool>("use_translation", false);
  use_twist_speed_ = declare_parameter<bool>("use_twist_speed", true);
  default_speed_mps_ = declare_parameter<double>("default_speed_mps", 0.0);
  max_speed_mps_ = declare_parameter<double>("max_speed_mps", 40.0);

  publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", true);
  diag_throttle_ms_ = declare_parameter<int>("diag_throttle_ms", 1000);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pub_points_ = create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic_, rclcpp::SensorDataQoS());
  pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ImuUndistorter::onImu, this, std::placeholders::_1));

  if (!twist_topic_.empty()) {
    sub_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuUndistorter::onTwist, this, std::placeholders::_1));
  }

  sub_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    points_in_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ImuUndistorter::onPoints, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "pure_imu_undistortion started. translation=%s twist=%s",
    use_translation_ ? "on" : "off",
    (!twist_topic_.empty()) ? "on" : "off");
}

void ImuUndistorter::pruneBuffers(const rclcpp::Time & nowt)
{
  const double now_sec = toSec(nowt);

  while (!imu_buf_.empty()) {
    const double t = toSec(imu_buf_.front().stamp);
    if (now_sec - t > imu_buffer_sec_) imu_buf_.pop_front();
    else break;
  }
  while (!twist_buf_.empty()) {
    const double t = toSec(twist_buf_.front().stamp);
    if (now_sec - t > twist_buffer_sec_) twist_buf_.pop_front();
    else break;
  }
}

void ImuUndistorter::onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  ImuSample s;
  s.stamp = msg->header.stamp;
  s.gyro = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  std::lock_guard<std::mutex> lk(mtx_);
  imu_buf_.push_back(s);
  pruneBuffers(msg->header.stamp);
}

void ImuUndistorter::onTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  double v = msg->twist.linear.x;
  if (!std::isfinite(v)) v = 0.0;
  v = std::max(-max_speed_mps_, std::min(max_speed_mps_, v));

  TwistSample s;
  s.stamp = msg->header.stamp;
  s.speed_mps = v;

  std::lock_guard<std::mutex> lk(mtx_);
  twist_buf_.push_back(s);
  pruneBuffers(msg->header.stamp);
}

bool ImuUndistorter::ensureStaticTf(const std::string & scan_frame, const std::string & imu_frame)
{
  // base <- scan
  if (!has_T_base_scan_) {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        base_frame_, scan_frame, tf2::TimePointZero, tf2::durationFromSec(0.2));

      Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                           tf.transform.rotation.y, tf.transform.rotation.z);
      if (q.norm() > 1e-12) q.normalize();

      T_base_scan_ = Eigen::Isometry3d::Identity();
      T_base_scan_.translation() = Eigen::Vector3d(tf.transform.translation.x,
                                                   tf.transform.translation.y,
                                                   tf.transform.translation.z);
      T_base_scan_.linear() = q.toRotationMatrix();

      has_T_base_scan_ = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed (base<-scan): %s", ex.what());
      return false;
    }
  }

  // base <- imu (optional but recommended)
  if (!has_T_base_imu_) {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        base_frame_, imu_frame, tf2::TimePointZero, tf2::durationFromSec(0.2));

      Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                           tf.transform.rotation.y, tf.transform.rotation.z);
      if (q.norm() > 1e-12) q.normalize();

      T_base_imu_ = Eigen::Isometry3d::Identity();
      T_base_imu_.translation() = Eigen::Vector3d(tf.transform.translation.x,
                                                  tf.transform.translation.y,
                                                  tf.transform.translation.z);
      T_base_imu_.linear() = q.toRotationMatrix();

      has_T_base_imu_ = true;
    } catch (const tf2::TransformException & ex) {
      // IMU frame変換が無い場合は「IMUがbase_frameで出ている」想定で進める
      // ただし frame_id が base と違う場合は回転がズレるので WARN
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed (base<-imu). Will assume IMU in base frame. err=%s", ex.what());
      has_T_base_imu_ = false;
    }
  }

  return true;
}

bool ImuUndistorter::findTimeField(const sensor_msgs::msg::PointCloud2 & msg,
                                   std::string & out_name,
                                   uint8_t & out_datatype,
                                   uint32_t & out_offset) const
{
  for (size_t i = 0; i < time_fields_.size(); ++i) {
    const std::string & name = time_fields_[i];
    for (size_t j = 0; j < msg.fields.size(); ++j) {
      const auto & f = msg.fields[j];
      if (f.name == name) {
        out_name = f.name;
        out_datatype = f.datatype;
        out_offset = f.offset;
        return true;
      }
    }
  }
  return false;
}

bool ImuUndistorter::readFieldAsDouble(const uint8_t * ptr, uint8_t datatype, double & v) const
{
  // sensor_msgs::msg::PointField datatype
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:   { int8_t  t; std::memcpy(&t, ptr, 1); v = static_cast<double>(t); return true; }
    case sensor_msgs::msg::PointField::UINT8:  { uint8_t t; std::memcpy(&t, ptr, 1); v = static_cast<double>(t); return true; }
    case sensor_msgs::msg::PointField::INT16:  { int16_t t; std::memcpy(&t, ptr, 2); v = static_cast<double>(t); return true; }
    case sensor_msgs::msg::PointField::UINT16: { uint16_t t; std::memcpy(&t, ptr, 2); v = static_cast<double>(t); return true; }
    case sensor_msgs::msg::PointField::INT32:  { int32_t t; std::memcpy(&t, ptr, 4); v = static_cast<double>(t); return true; }
    case sensor_msgs::msg::PointField::UINT32: { uint32_t t; std::memcpy(&t, ptr, 4); v = static_cast<double>(t); return true; }
    case sensor_msgs::msg::PointField::FLOAT32:{ float   t; std::memcpy(&t, ptr, 4); v = static_cast<double>(t); return true; }
    case sensor_msgs::msg::PointField::FLOAT64:{ double  t; std::memcpy(&t, ptr, 8); v = t; return true; }
    default: return false;
  }
}

double ImuUndistorter::estimateTimeScale(double raw_range, double scan_period) const
{
  // raw_range * scale ~= scan_period となる候補を探す（ns/us/ms/s を吸収）
  const double candidates[] = {1.0, 1e-3, 1e-6, 1e-9};

  double best = 1.0;
  double best_score = 1e100;

  for (size_t i = 0; i < 4; ++i) {
    const double s = candidates[i];
    const double r = raw_range * s;

    if (!std::isfinite(r)) continue;
    if (r <= 1e-6) continue;
    if (r > max_time_offset_sec_) continue;

    const double score = std::fabs(r - scan_period);
    if (score < best_score) {
      best_score = score;
      best = s;
    }
  }
  return best;
}

ImuUndistorter::PointTimeInfo ImuUndistorter::preparePointTimeInfo(const sensor_msgs::msg::PointCloud2 & msg) const
{
  PointTimeInfo ti;

  // scan period
  double scan_period = fallback_scan_period_;
  if (scan_period <= 1e-4) scan_period = 0.1;
  ti.scan_period = scan_period;

  // scan start time t0
  const rclcpp::Time stamp = msg.header.stamp;
  const double stamp_sec = toSec(stamp);
  const double t0 = cloud_stamp_is_start_ ? stamp_sec : (stamp_sec - scan_period);
  ti.t0_sec = t0;

  // reference time
  if (reference_time_ == "end") ti.t_ref_sec = t0 + scan_period;
  else ti.t_ref_sec = t0;

  // time field detection
  std::string name;
  uint8_t datatype = 0;
  uint32_t offset = 0;
  const bool has = findTimeField(msg, name, datatype, offset);

  ti.has_time_field = has;
  if (!has) {
    ti.used_linear_fallback = true;
    ti.interpreted_as_relative = true;
    ti.time_scale = 1.0;
    return ti;
  }

  ti.field_name = name;
  ti.datatype = datatype;
  ti.offset = offset;

  // decide scale + relative/absolute by sampling a few points
  // sample raw values to estimate range
  const int point_count = static_cast<int>(msg.width * msg.height);
  if (point_count <= 1) {
    ti.used_linear_fallback = true;
    return ti;
  }

  const int stride = static_cast<int>(msg.point_step);
  const uint8_t * data = msg.data.data();

  const int sample_n = std::min(40, point_count);
  double vmin = std::numeric_limits<double>::infinity();
  double vmax = -std::numeric_limits<double>::infinity();

  for (int k = 0; k < sample_n; ++k) {
    const int idx = (k * point_count) / sample_n;
    const uint8_t * p = data + idx * stride + offset;

    double raw = 0.0;
    if (!readFieldAsDouble(p, datatype, raw)) continue;
    if (!std::isfinite(raw)) continue;
    vmin = std::min(vmin, raw);
    vmax = std::max(vmax, raw);
  }

  if (!std::isfinite(vmin) || !std::isfinite(vmax) || vmax <= vmin) {
    // fallback
    ti.used_linear_fallback = true;
    return ti;
  }

  const double raw_range = vmax - vmin;

  // choose scale
  double scale = (time_scale_ > 0.0) ? time_scale_ : estimateTimeScale(raw_range, scan_period);
  ti.time_scale = scale;

  // decide relative vs absolute
  // relative: values mostly within [0, scan_period] after scale
  const double range_sec = raw_range * scale;
  bool relative_like = (range_sec > 1e-6) && (range_sec <= max_time_offset_sec_);

  // absolute: values are large epoch-like (e.g. ns since epoch), after scale it's huge
  // If scale=1e-9 and raw is epoch_ns, raw_range may still be small but vmin itself is huge.
  // We also check magnitude.
  const double mag_sec = std::fabs(vmin * scale);
  bool absolute_like = (mag_sec > 1e6);  // ~11 days in seconds, rough heuristic

  if (prefer_relative_time_) {
    ti.interpreted_as_relative = relative_like && !absolute_like;
  } else {
    ti.interpreted_as_relative = relative_like && !absolute_like;
  }

  return ti;
}

bool ImuUndistorter::computePointDtSec(const sensor_msgs::msg::PointCloud2 & msg,
                                       const PointTimeInfo & ti,
                                       int point_index,
                                       int point_count,
                                       const uint8_t * point_ptr,
                                       double & out_dt) const
{
  if (point_count <= 1) {
    out_dt = 0.0;
    return true;
  }

  if (!ti.has_time_field) {
    // linear fallback
    const double s = ti.scan_period;
    const double alpha = static_cast<double>(point_index) / static_cast<double>(point_count - 1);
    out_dt = std::max(0.0, std::min(s, alpha * s));
    return true;
  }

  // read raw
  double raw = 0.0;
  if (!readFieldAsDouble(point_ptr + ti.offset, ti.datatype, raw)) {
    return false;
  }
  if (!std::isfinite(raw)) return false;

  const double t_raw_sec = raw * ti.time_scale;

  double dt = 0.0;
  if (ti.interpreted_as_relative) {
    dt = t_raw_sec;
  } else {
    // absolute time in sec
    const double t_abs = t_raw_sec;
    dt = t_abs - ti.t0_sec;
  }

  // clamp
  if (!std::isfinite(dt)) return false;
  dt = std::max(-max_time_offset_sec_, std::min(max_time_offset_sec_, dt));
  // keep within [0, scan_period] as much as possible
  dt = std::max(0.0, std::min(ti.scan_period, dt));
  out_dt = dt;
  return true;
}

static Eigen::Quaterniond deltaQFromGyro(const Eigen::Vector3d & w_rad_s, double dt)
{
  // small-angle expmap
  const double th = w_rad_s.norm() * dt;
  if (th < 1e-12) {
    return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  }
  const Eigen::Vector3d axis = w_rad_s.normalized();
  const double half = 0.5 * th;
  const double s = std::sin(half);
  return Eigen::Quaterniond(std::cos(half), axis.x()*s, axis.y()*s, axis.z()*s);
}

bool ImuUndistorter::buildBaseTrajectory(double t0_sec, double t1_sec,
                                         std::vector<PoseSample> & out_traj,
                                         std::string & out_reason)
{
  out_traj.clear();
  out_reason.clear();

  // copy imu buffer snapshot
  std::deque<ImuSample> imu;
  std::deque<TwistSample> twist;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    imu = imu_buf_;
    twist = twist_buf_;
  }

  if (imu.size() < 2) {
    out_reason = "imu buffer too small";
    return false;
  }

  // collect samples within [t0, t1]
  std::vector<ImuSample> sel;
  sel.reserve(imu.size());

  for (const auto & s : imu) {
    const double ts = toSec(s.stamp);
    if (ts < t0_sec - 0.2) continue;
    if (ts > t1_sec + 0.2) break;
    sel.push_back(s);
  }

  if (sel.size() < 2) {
    out_reason = "no imu in time window";
    return false;
  }

  // Sort by stamp (just in case)
  std::sort(sel.begin(), sel.end(),
            [](const ImuSample & a, const ImuSample & b) { return a.stamp < b.stamp; });

  // Build trajectory timestamps: include endpoints + each IMU stamp in window
  std::vector<double> ts;
  ts.reserve(sel.size() + 2);
  ts.push_back(t0_sec);
  for (size_t i = 0; i < sel.size(); ++i) {
    const double t = toSec(sel[i].stamp);
    if (t > t0_sec && t < t1_sec) ts.push_back(t);
  }
  ts.push_back(t1_sec);
  std::sort(ts.begin(), ts.end());
  ts.erase(std::unique(ts.begin(), ts.end(),
                       [](double a, double b){ return std::fabs(a-b) < 1e-9; }),
           ts.end());

  // Integrate gyro in base frame.
  // If we have base<-imu TF, convert imu gyro to base by rotation.
  const Eigen::Matrix3d R_BI =
    has_T_base_imu_ ? Eigen::Matrix3d(T_base_imu_.linear()) : Eigen::Matrix3d::Identity();

  Eigen::Quaterniond q_WB(1,0,0,0); // W defined as base at t0
  Eigen::Vector3d p_WB(0,0,0);

  out_traj.reserve(ts.size());
  out_traj.push_back(PoseSample{ts.front(), q_WB, p_WB});

  // helper: get gyro at time t by nearest previous sample (zero-order hold)
  size_t imu_idx = 0;

  auto gyroAt = [&](double t_sec)->Eigen::Vector3d {
    while (imu_idx + 1 < sel.size() && toSec(sel[imu_idx + 1].stamp) <= t_sec) imu_idx++;
    Eigen::Vector3d w_I = sel[imu_idx].gyro;
    Eigen::Vector3d w_B = R_BI * w_I;
    return w_B;
  };

  // helper: get speed at time t (zero-order hold)
  size_t tw_idx = 0;
  auto speedAt = [&](double t_sec)->double {
    if (!use_translation_) return 0.0;
    if (!use_twist_speed_) return std::max(-max_speed_mps_, std::min(max_speed_mps_, default_speed_mps_));
    if (twist.empty()) return std::max(-max_speed_mps_, std::min(max_speed_mps_, default_speed_mps_));

    while (tw_idx + 1 < twist.size() && toSec(twist[tw_idx + 1].stamp) <= t_sec) tw_idx++;
    double v = twist[tw_idx].speed_mps;
    if (!std::isfinite(v)) v = 0.0;
    v = std::max(-max_speed_mps_, std::min(max_speed_mps_, v));
    return v;
  };

  for (size_t k = 1; k < ts.size(); ++k) {
    const double t_prev = ts[k-1];
    const double t_now  = ts[k];
    const double dt = t_now - t_prev;

    if (dt <= 0.0) continue;
    if (dt > max_imu_gap_sec_ * 10.0) {
      // too sparse; still allow but warn by reason
      // (trajectory will be less accurate)
    }

    const Eigen::Vector3d w_B = gyroAt(0.5*(t_prev + t_now));
    const Eigen::Quaterniond dq = deltaQFromGyro(w_B, dt);
    q_WB = (q_WB * dq).normalized();

    if (use_translation_) {
      const double v = speedAt(0.5*(t_prev + t_now));
      // move along base x axis in world: p += R * (v*dt,0,0)
      const Eigen::Vector3d vB(v * dt, 0.0, 0.0);
      const Eigen::Vector3d vW = q_WB.toRotationMatrix() * vB;
      p_WB += vW;
    }

    out_traj.push_back(PoseSample{t_now, q_WB, p_WB});
  }

  return true;
}

bool ImuUndistorter::orientationAt(const std::vector<PoseSample> & traj, double t_sec, Eigen::Quaterniond & q_WB) const
{
  if (traj.empty()) return false;
  if (t_sec <= traj.front().t_sec) { q_WB = traj.front().q_WB; return true; }
  if (t_sec >= traj.back().t_sec) { q_WB = traj.back().q_WB; return true; }

  // find segment
  size_t hi = 1;
  while (hi < traj.size() && traj[hi].t_sec < t_sec) hi++;
  if (hi >= traj.size()) { q_WB = traj.back().q_WB; return true; }
  const size_t lo = hi - 1;

  const double t0 = traj[lo].t_sec;
  const double t1 = traj[hi].t_sec;
  const double a = (t_sec - t0) / std::max(1e-9, (t1 - t0));

  q_WB = traj[lo].q_WB.slerp(a, traj[hi].q_WB).normalized();
  return true;
}

bool ImuUndistorter::positionAt(const std::vector<PoseSample> & traj, double t_sec, Eigen::Vector3d & p_WB) const
{
  if (!use_translation_) { p_WB = Eigen::Vector3d(0,0,0); return true; }
  if (traj.empty()) return false;
  if (t_sec <= traj.front().t_sec) { p_WB = traj.front().p_WB; return true; }
  if (t_sec >= traj.back().t_sec) { p_WB = traj.back().p_WB; return true; }

  size_t hi = 1;
  while (hi < traj.size() && traj[hi].t_sec < t_sec) hi++;
  if (hi >= traj.size()) { p_WB = traj.back().p_WB; return true; }
  const size_t lo = hi - 1;

  const double t0 = traj[lo].t_sec;
  const double t1 = traj[hi].t_sec;
  const double a = (t_sec - t0) / std::max(1e-9, (t1 - t0));

  p_WB = (1.0 - a) * traj[lo].p_WB + a * traj[hi].p_WB;
  return true;
}

bool ImuUndistorter::findXYZOffsets(const sensor_msgs::msg::PointCloud2 & msg,
                                    uint32_t & off_x, uint32_t & off_y, uint32_t & off_z,
                                    uint8_t & dt_x, uint8_t & dt_y, uint8_t & dt_z) const
{
  bool fx=false, fy=false, fz=false;
  for (size_t i = 0; i < msg.fields.size(); ++i) {
    const auto & f = msg.fields[i];
    if (f.name == "x") { off_x = f.offset; dt_x = f.datatype; fx = true; }
    if (f.name == "y") { off_y = f.offset; dt_y = f.datatype; fy = true; }
    if (f.name == "z") { off_z = f.offset; dt_z = f.datatype; fz = true; }
  }
  // xyz must be float32 ideally
  return fx && fy && fz;
}

static bool readFloat32(const uint8_t * p, float & v)
{
  std::memcpy(&v, p, 4);
  return std::isfinite(v);
}

static void writeFloat32(uint8_t * p, float v)
{
  std::memcpy(p, &v, 4);
}

bool ImuUndistorter::deskewPointCloud(const sensor_msgs::msg::PointCloud2 & in,
                                      sensor_msgs::msg::PointCloud2 & out,
                                      std::string & out_reason)
{
  out_reason.clear();

  if (in.data.empty()) {
    out_reason = "empty cloud";
    return false;
  }

  // xyz offsets
  uint32_t off_x=0, off_y=0, off_z=0;
  uint8_t dt_x=0, dt_y=0, dt_z=0;
  if (!findXYZOffsets(in, off_x, off_y, off_z, dt_x, dt_y, dt_z)) {
    out_reason = "xyz fields not found";
    return false;
  }
  if (dt_x != sensor_msgs::msg::PointField::FLOAT32 ||
      dt_y != sensor_msgs::msg::PointField::FLOAT32 ||
      dt_z != sensor_msgs::msg::PointField::FLOAT32) {
    out_reason = "xyz datatype not float32 (not supported in this minimal implementation)";
    return false;
  }

  // time info
  const PointTimeInfo ti = preparePointTimeInfo(in);

  const int point_count = static_cast<int>(in.width * in.height);
  if (point_count <= 0) {
    out_reason = "invalid point count";
    return false;
  }

  // Build base trajectory [t0, t1]
  const double t0 = ti.t0_sec;
  const double t1 = ti.t0_sec + ti.scan_period;

  std::vector<PoseSample> traj;
  std::string reason_traj;
  if (!buildBaseTrajectory(t0, t1, traj, reason_traj)) {
    out_reason = "trajectory build failed: " + reason_traj;
    return false;
  }

  // Reference pose
  Eigen::Quaterniond q_ref;
  Eigen::Vector3d p_ref;
  if (!orientationAt(traj, ti.t_ref_sec, q_ref)) {
    out_reason = "no orientation at ref";
    return false;
  }
  if (!positionAt(traj, ti.t_ref_sec, p_ref)) {
    out_reason = "no position at ref";
    return false;
  }

  // output copy
  out = in;
  out.header.frame_id = in.header.frame_id; // keep same
  out.data = in.data;                       // will modify xyz

  const int step = static_cast<int>(in.point_step);
  const uint8_t * in_data = in.data.data();
  uint8_t * out_data = out.data.data();

  // Precompute extrinsic
  const Eigen::Isometry3d T_SB = T_base_scan_.inverse(); // scan <- base

  // W frame defined as base at t0. So T_WB(t0)=I.
  for (int i = 0; i < point_count; ++i) {
    const uint8_t * p_in = in_data + i * step;
    uint8_t * p_out = out_data + i * step;

    float x=0, y=0, z=0;
    if (!readFloat32(p_in + off_x, x) ||
        !readFloat32(p_in + off_y, y) ||
        !readFloat32(p_in + off_z, z)) {
      continue;
    }

    // compute dt
    double dt = 0.0;
    if (!computePointDtSec(in, ti, i, point_count, p_in, dt)) {
      // fallback to linear
      const double alpha = static_cast<double>(i) / static_cast<double>(std::max(1, point_count-1));
      dt = std::max(0.0, std::min(ti.scan_period, alpha * ti.scan_period));
    }

    const double t_i = ti.t0_sec + dt;

    Eigen::Quaterniond q_i;
    Eigen::Vector3d p_i;
    if (!orientationAt(traj, t_i, q_i)) continue;
    if (!positionAt(traj, t_i, p_i)) continue;

    // Relative transform from base_i to base_ref:
    // p_base_ref = (T_WB(ref))^-1 * T_WB(i) * p_base_i
    const Eigen::Matrix3d R_ref = q_ref.toRotationMatrix();
    const Eigen::Matrix3d R_i   = q_i.toRotationMatrix();

    const Eigen::Matrix3d R_rel = R_ref.transpose() * R_i;
    const Eigen::Vector3d t_rel = R_ref.transpose() * (p_i - p_ref);

    // point in scan_i -> base_i
    Eigen::Vector3d pS(x, y, z);
    Eigen::Vector3d pB = (T_base_scan_ * pS.homogeneous()).head<3>();

    // transform to base_ref
    Eigen::Vector3d pB_ref = R_rel * pB + t_rel;

    // base_ref -> scan_ref (extrinsic static)
    Eigen::Vector3d pS_ref = (T_SB * pB_ref.homogeneous()).head<3>();

    writeFloat32(p_out + off_x, static_cast<float>(pS_ref.x()));
    writeFloat32(p_out + off_y, static_cast<float>(pS_ref.y()));
    writeFloat32(p_out + off_z, static_cast<float>(pS_ref.z()));
  }

  return true;
}

void ImuUndistorter::publishDiag(const rclcpp::Time & stamp,
                                 const std::string & level,
                                 const std::string & msg,
                                 const PointTimeInfo & ti) const
{
  if (!publish_diagnostics_) return;

  diagnostic_msgs::msg::DiagnosticArray da;
  da.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "localization/imu_undistortion";
  st.hardware_id = "none";
  st.level = toDiagLevel(level);
  st.message = msg;

  auto addKV = [&](const std::string & k, const std::string & v){
    diagnostic_msgs::msg::KeyValue kv; kv.key=k; kv.value=v; st.values.push_back(kv);
  };
  const double msg_age_ms = (this->now() - stamp).seconds() * 1000.0;
  addKV("msg_age_ms", std::to_string(msg_age_ms));

  addKV("has_time_field", ti.has_time_field ? "true" : "false");
  addKV("time_field_name", ti.has_time_field ? ti.field_name : "");
  addKV("used_linear_fallback", ti.used_linear_fallback ? "true" : "false");
  addKV("scan_period", std::to_string(ti.scan_period));
  addKV("time_scale", std::to_string(ti.time_scale));
  addKV("interpreted_as_relative", ti.interpreted_as_relative ? "true" : "false");
  addKV("reference_time", reference_time_);
  addKV("use_translation", use_translation_ ? "true" : "false");
  addKV("use_twist_speed", use_twist_speed_ ? "true" : "false");

  da.status.push_back(st);
  pub_diag_->publish(da);
}

void ImuUndistorter::onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const std::string scan_frame = scan_frame_.empty() ? msg->header.frame_id : scan_frame_;
  const std::string imu_frame  = imu_frame_.empty()  ? "" : imu_frame_;

  std::string imu_frame_used = imu_frame;
  if (imu_frame_used.empty()) {
    // IMU frame is taken from incoming IMU messages; TF will be tried on first call anyway.
    // Here we just keep empty; ensureStaticTf will handle base<-imu lookup lazily.
    imu_frame_used = base_frame_;
  }

  // Ensure TF (base<-scan). base<-imu is optional.
  if (!ensureStaticTf(scan_frame, imu_frame_used)) {
    PointTimeInfo ti = preparePointTimeInfo(*msg);
    publishDiag(msg->header.stamp, "WARN", "TF not ready", ti);
    return;
  }

  sensor_msgs::msg::PointCloud2 out;
  std::string reason;
  const bool ok = deskewPointCloud(*msg, out, reason);

  PointTimeInfo ti = preparePointTimeInfo(*msg);
  if (!ok) {
    publishDiag(msg->header.stamp, "WARN", "deskew failed: " + reason, ti);
    return;
  }

  pub_points_->publish(out);

  // Diagnostics: OK/WARN based on fallback usage
  if (ti.used_linear_fallback) {
    publishDiag(msg->header.stamp, "WARN", "deskew OK (linear fallback: no per-point time)", ti);
  } else {
    publishDiag(msg->header.stamp, "OK", "deskew OK (time field)", ti);
  }
}

}  // namespace pure_imu_undistortion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pure_imu_undistortion::ImuUndistorter)
