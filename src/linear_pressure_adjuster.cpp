#include "safety_emergency_executor/linear_pressure_adjuster.hpp"

#include <algorithm>
#include <cmath>

namespace safety_emergency_executor
{

void LinearPressureAdjuster::configure(rclcpp::Node & node)
{
  node_ = &node;

  params_.enable_imu = node.declare_parameter<bool>(
    "auto_pressure.enable_imu", params_.enable_imu);
  params_.enable_localization = node.declare_parameter<bool>(
    "auto_pressure.enable_localization", params_.enable_localization);
  params_.fallback_mode = node.declare_parameter<std::string>(
    "auto_pressure.fallback_mode", params_.fallback_mode);

  params_.slip_threshold = node.declare_parameter<double>(
    "auto_pressure.slip_threshold", params_.slip_threshold);
  params_.slip_clear_threshold = node.declare_parameter<double>(
    "auto_pressure.slip_hysteresis", params_.slip_clear_threshold);
  params_.pressure_min = node.declare_parameter<int>(
    "auto_pressure.pressure_min", params_.pressure_min);
  params_.pressure_max = node.declare_parameter<int>(
    "auto_pressure.pressure_max", params_.pressure_max);
  params_.pressure_increment = node.declare_parameter<double>(
    "auto_pressure.rate_limit_up", params_.pressure_increment);
  params_.pressure_decrement = node.declare_parameter<double>(
    "auto_pressure.rate_limit_down", params_.pressure_decrement);

  params_.static_vel_threshold = node.declare_parameter<double>(
    "auto_pressure.static_vel_threshold", params_.static_vel_threshold);
  params_.bias_calibration_samples = node.declare_parameter<int>(
    "auto_pressure.bias_calibration_samples", params_.bias_calibration_samples);
  params_.imu_trust_duration = node.declare_parameter<double>(
    "auto_pressure.imu_trust_duration", params_.imu_trust_duration);
  params_.imu_decay_rate = node.declare_parameter<double>(
    "auto_pressure.imu_decay_rate", params_.imu_decay_rate);
  params_.weight_imu = node.declare_parameter<double>(
    "auto_pressure.weight_imu", params_.weight_imu);
  params_.loc_jump_threshold = node.declare_parameter<double>(
    "auto_pressure.loc_jump_threshold", params_.loc_jump_threshold);
  params_.loc_recovery_rate = node.declare_parameter<double>(
    "auto_pressure.loc_recovery_rate", params_.loc_recovery_rate);
}

void LinearPressureAdjuster::checkLocalizationHealth(double loc_lin_x, double loc_ang_z)
{
  if (!loc_initialized_) {
    last_loc_lin_ = loc_lin_x;
    last_loc_ang_ = loc_ang_z;
    loc_initialized_ = true;
    return;
  }

  const double lin_jump = std::abs(loc_lin_x - last_loc_lin_);
  const double ang_jump = std::abs(loc_ang_z - last_loc_ang_);

  if (lin_jump > params_.loc_jump_threshold || ang_jump > params_.loc_jump_threshold) {
    loc_trust_ = 0.0;
    if (node_ != nullptr) {
      RCLCPP_WARN(
        node_->get_logger(), "Localization jump lin=%.3f ang=%.3f", lin_jump, ang_jump);
    }
  } else {
    loc_trust_ = std::min(1.0, loc_trust_ + params_.loc_recovery_rate);
  }

  last_loc_lin_ = loc_lin_x;
  last_loc_ang_ = loc_ang_z;
}

void LinearPressureAdjuster::updateImu(const sensor_msgs::msg::Imu & imu, double wheel_vel)
{
  const rclcpp::Time current_time = imu.header.stamp;

  if (!imu_time_initialized_) {
    last_imu_time_ = current_time;
    imu_time_initialized_ = true;
    return;
  }

  const double dt = (current_time - last_imu_time_).seconds();
  last_imu_time_ = current_time;
  if (dt <= 0.0 || dt > 0.5) {
    return;
  }

  constexpr double kGravity = 9.81;
  const double a_x = imu.linear_acceleration.x * kGravity;
  const bool currently_static = std::abs(wheel_vel) < params_.static_vel_threshold;

  if (currently_static) {
    static_frame_count_++;
    if (static_frame_count_ > 10) {
      is_static_ = true;
      bias_buffer_[bias_buffer_idx_] = a_x;
      bias_buffer_idx_ = (bias_buffer_idx_ + 1) % BIAS_BUFFER_SIZE;
      bias_sample_count_++;

      if (bias_sample_count_ >= static_cast<size_t>(params_.bias_calibration_samples)) {
        const size_t count = std::min(bias_sample_count_, BIAS_BUFFER_SIZE);
        double sum = 0.0;
        for (size_t idx = 0; idx < count; ++idx) {
          sum += bias_buffer_[idx];
        }
        const double new_bias = sum / static_cast<double>(count);
        if (bias_calibrated_) {
          acc_bias_ = 0.9 * acc_bias_ + 0.1 * new_bias;
        } else {
          acc_bias_ = new_bias;
          bias_calibrated_ = true;
        }
      }

      imu_vel_ = 0.0;
      imu_integration_time_ = 0.0;
    }
    return;
  }

  static_frame_count_ = 0;
  is_static_ = false;
  if (!bias_calibrated_) {
    return;
  }

  imu_integration_time_ += dt;
  if (imu_integration_time_ > params_.imu_trust_duration) {
    imu_vel_ = 0.7 * imu_vel_ + 0.3 * wheel_vel;
    imu_integration_time_ = 0.0;
  } else {
    const double corrected_acc = a_x - acc_bias_;
    imu_vel_ += corrected_acc * dt;
    imu_vel_ *= params_.imu_decay_rate;
    imu_vel_ = std::clamp(imu_vel_, -3.0, 3.0);
  }

  if (std::abs(wheel_vel) > 0.1 && wheel_vel * imu_vel_ < 0) {
    imu_vel_ = wheel_vel * 0.5;
    imu_integration_time_ = 0.0;
  }
}

int LinearPressureAdjuster::applyLinearAdjustment(int base_press, bool slip_detected)
{
  if (!pressure_initialized_) {
    current_pressure_ = base_press;
    pressure_initialized_ = true;
  }

  if (slip_detected) {
    current_pressure_ += params_.pressure_increment;
  } else if (current_pressure_ > base_press) {
    current_pressure_ -= params_.pressure_decrement;
    current_pressure_ = std::max(current_pressure_, base_press);
  }

  current_pressure_ = std::clamp(
    current_pressure_, params_.pressure_min, params_.pressure_max);
  return current_pressure_;
}

bool LinearPressureAdjuster::update(
  const nav_msgs::msg::Odometry & wheel_odom,
  const nav_msgs::msg::Odometry & loc_odom,
  int base_press,
  int & out_press)
{
  const double wheel_lin_x = wheel_odom.twist.twist.linear.x;
  const double wheel_ang_z = wheel_odom.twist.twist.angular.z;
  const double loc_lin_x = loc_odom.twist.twist.linear.x;
  const double loc_ang_z = loc_odom.twist.twist.angular.z;

  checkLocalizationHealth(loc_lin_x, loc_ang_z);

  double loc_slip = computeLocSlipRatio(wheel_lin_x, wheel_ang_z, loc_lin_x, loc_ang_z);
  loc_slip *= loc_trust_;

  double imu_slip = computeImuSlipRatio(wheel_lin_x);
  const double imu_trust = 1.0 - std::min(1.0, imu_integration_time_ / params_.imu_trust_duration);
  imu_slip *= imu_trust;

  const double slip_metric = std::max(loc_slip, imu_slip);
  last_slip_ratio_ = slip_metric;

  if (slip_metric > params_.slip_threshold) {
    slip_active_ = true;
  } else if (slip_metric < params_.slip_clear_threshold) {
    slip_active_ = false;
  }

  out_press = applyLinearAdjustment(base_press, slip_active_);
  return slip_active_;
}

bool LinearPressureAdjuster::updateImuOnly(
  const nav_msgs::msg::Odometry & wheel_odom,
  int base_press,
  int & out_press)
{
  const double wheel_lin_x = wheel_odom.twist.twist.linear.x;
  const double imu_slip = computeImuSlipRatio(wheel_lin_x);
  last_slip_ratio_ = imu_slip;

  if (imu_slip > params_.slip_threshold) {
    slip_active_ = true;
  } else if (imu_slip < params_.slip_clear_threshold) {
    slip_active_ = false;
  }

  out_press = applyLinearAdjustment(base_press, slip_active_);
  return slip_active_;
}

double LinearPressureAdjuster::computeLocSlipRatio(
  double wheel_lin_x, double wheel_ang_z,
  double loc_lin_x, double loc_ang_z) const
{
  double slip = 0.0;

  const double wheel_speed = std::abs(wheel_lin_x);
  const double loc_speed = std::abs(loc_lin_x);
  if (wheel_speed > 0.05) {
    const double speed_diff = wheel_speed - loc_speed;
    if (speed_diff > 0.01) {
      slip = std::max(slip, speed_diff / wheel_speed);
    }
  }

  const double wheel_angular = std::abs(wheel_ang_z);
  const double loc_angular = std::abs(loc_ang_z);
  if (wheel_angular > 0.05) {
    const double angular_diff = wheel_angular - loc_angular;
    if (angular_diff > 0.01) {
      slip = std::max(slip, angular_diff / wheel_angular);
    }
  }

  return slip;
}

double LinearPressureAdjuster::computeImuSlipRatio(double wheel_lin_x) const
{
  if (!bias_calibrated_) {
    return 0.0;
  }

  const double wheel_speed = std::abs(wheel_lin_x);
  const double imu_speed = std::abs(imu_vel_);
  if (wheel_speed > 0.05) {
    const double speed_diff = wheel_speed - imu_speed;
    if (speed_diff > 0.01) {
      return params_.weight_imu * (speed_diff / wheel_speed);
    }
  }

  return 0.0;
}

}  // namespace safety_emergency_executor
