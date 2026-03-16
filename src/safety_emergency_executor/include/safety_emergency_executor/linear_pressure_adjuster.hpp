#ifndef SAFETY_EMERGENCY_EXECUTOR__LINEAR_PRESSURE_ADJUSTER_HPP_
#define SAFETY_EMERGENCY_EXECUTOR__LINEAR_PRESSURE_ADJUSTER_HPP_

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace safety_emergency_executor
{

class LinearPressureAdjuster
{
public:
  struct Params
  {
    bool enable_imu{true};
    bool enable_localization{true};
    std::string fallback_mode{"dual_sensor"};
    double slip_threshold{0.08};
    double slip_clear_threshold{0.03};
    int pressure_min{800};
    int pressure_max{2500};
    int pressure_increment{30};
    int pressure_decrement{20};
    double static_vel_threshold{0.02};
    int bias_calibration_samples{50};
    double imu_trust_duration{2.0};
    double imu_decay_rate{0.98};
    double weight_imu{0.8};
    double loc_jump_threshold{0.5};
    double loc_recovery_rate{0.1};
  };

  LinearPressureAdjuster() = default;

  void configure(rclcpp::Node & node);
  void updateImu(const sensor_msgs::msg::Imu & imu, double wheel_vel);

  bool update(
    const nav_msgs::msg::Odometry & wheel_odom,
    const nav_msgs::msg::Odometry & loc_odom,
    int base_press,
    int & out_press);

  bool updateImuOnly(
    const nav_msgs::msg::Odometry & wheel_odom,
    int base_press,
    int & out_press);

  bool isImuEnabled() const {return params_.enable_imu;}
  bool isLocalizationEnabled() const {return params_.enable_localization;}
  std::string getFallbackMode() const {return params_.fallback_mode;}

private:
  double computeLocSlipRatio(
    double wheel_lin_x, double wheel_ang_z,
    double loc_lin_x, double loc_ang_z) const;
  double computeImuSlipRatio(double wheel_lin_x) const;
  void checkLocalizationHealth(double loc_lin_x, double loc_ang_z);
  int applyLinearAdjustment(int base_press, bool slip_detected);

  Params params_;
  rclcpp::Node * node_{nullptr};
  bool slip_active_{false};
  int current_pressure_{0};
  bool pressure_initialized_{false};
  double last_slip_ratio_{0.0};
  double imu_vel_{0.0};
  double acc_bias_{0.0};
  bool bias_calibrated_{false};
  rclcpp::Time last_imu_time_;
  bool imu_time_initialized_{false};
  double imu_integration_time_{0.0};
  static constexpr size_t BIAS_BUFFER_SIZE = 100;
  std::array<double, BIAS_BUFFER_SIZE> bias_buffer_{};
  size_t bias_buffer_idx_{0};
  size_t bias_sample_count_{0};
  bool is_static_{false};
  int static_frame_count_{0};
  double loc_trust_{1.0};
  double last_loc_lin_{0.0};
  double last_loc_ang_{0.0};
  bool loc_initialized_{false};
};

}  // namespace safety_emergency_executor

#endif  // SAFETY_EMERGENCY_EXECUTOR__LINEAR_PRESSURE_ADJUSTER_HPP_
