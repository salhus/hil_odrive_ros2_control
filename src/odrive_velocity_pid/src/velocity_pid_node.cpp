#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class VelocityPidNode : public rclcpp::Node
{
public:
  VelocityPidNode()
  : Node("velocity_pid_node"),
    integral_(0.0),
    prev_error_(0.0),
    last_measured_vel_(0.0),
    filtered_vel_(0.0),
    joint_index_(-1),
    saturated_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    this->declare_parameter<std::string>("command_topic", "/motor_effort_controller/commands");
    this->declare_parameter<std::string>("joint_name", "motor_joint");

    this->declare_parameter<double>("amplitude_rad_s", 1.0);
    this->declare_parameter<double>("omega_rad_s", 1.0);

    this->declare_parameter<double>("kp", 1.0);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("kd", 0.0);
    this->declare_parameter<double>("kff", 0.0);

    this->declare_parameter<double>("torque_limit_nm", 10.0);
    this->declare_parameter<double>("integral_limit", 5.0);
    this->declare_parameter<double>("deadband_rad_s", 0.0);
    this->declare_parameter<double>("rate_hz", 100.0);
    this->declare_parameter<double>("filter_alpha", 0.3);
    this->declare_parameter<bool>("invert_output", false);

    // Read parameters
    joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
    command_topic_     = this->get_parameter("command_topic").as_string();
    joint_name_        = this->get_parameter("joint_name").as_string();

    amplitude_rad_s_ = this->get_parameter("amplitude_rad_s").as_double();
    omega_rad_s_     = this->get_parameter("omega_rad_s").as_double();

    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    kff_ = this->get_parameter("kff").as_double();

    torque_limit_nm_ = this->get_parameter("torque_limit_nm").as_double();
    integral_limit_  = this->get_parameter("integral_limit").as_double();
    deadband_rad_s_  = this->get_parameter("deadband_rad_s").as_double();
    rate_hz_         = this->get_parameter("rate_hz").as_double();
    filter_alpha_    = this->get_parameter("filter_alpha").as_double();
    invert_output_   = this->get_parameter("invert_output").as_bool();

    // Validate parameters
    if (rate_hz_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "rate_hz must be positive; defaulting to 100 Hz");
      rate_hz_ = 100.0;
    }
    if (torque_limit_nm_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "torque_limit_nm must be positive; defaulting to 10.0");
      torque_limit_nm_ = 10.0;
    }
    if (integral_limit_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "integral_limit must be positive; defaulting to 5.0");
      integral_limit_ = 5.0;
    }
    if (filter_alpha_ < 0.0 || filter_alpha_ >= 1.0) {
      RCLCPP_WARN(this->get_logger(), "filter_alpha must be in [0.0, 1.0); defaulting to 0.3");
      filter_alpha_ = 0.3;
    }

    dt_ = 1.0 / rate_hz_;

    // Subscriber
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_state_callback(msg);
      });

    // Publisher
    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);

    // Control loop timer
    const auto period_ms = std::chrono::duration<double>(dt_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period_ms),
      [this]() { control_loop(); });

    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "VelocityPidNode started: joint='%s', rate=%.1f Hz, kp=%.3f ki=%.3f kd=%.3f kff=%.3f",
      joint_name_.c_str(), rate_hz_, kp_, ki_, kd_, kff_);
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Find joint index once (or re-check if it changes)
    if (joint_index_ < 0 ||
        static_cast<size_t>(joint_index_) >= msg->name.size() ||
        msg->name[static_cast<size_t>(joint_index_)] != joint_name_)
    {
      joint_index_ = -1;
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == joint_name_) {
          joint_index_ = static_cast<int>(i);
          break;
        }
      }
      if (joint_index_ < 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Joint '%s' not found in /joint_states", joint_name_.c_str());
        return;
      }
    }

    const size_t idx = static_cast<size_t>(joint_index_);
    if (idx < msg->velocity.size()) {
      double raw = msg->velocity[idx];
      filtered_vel_ = filter_alpha_ * filtered_vel_ + (1.0 - filter_alpha_) * raw;
      last_measured_vel_ = filtered_vel_;
    }
  }

  void control_loop()
  {
    if (joint_index_ < 0) {
      // Not yet initialized — publish zero torque
      publish_torque(0.0);
      return;
    }

    // Elapsed time from node start
    const double t = (this->now() - start_time_).seconds();

    // Desired velocity (sine trajectory)
    const double desired_vel = amplitude_rad_s_ * std::sin(omega_rad_s_ * t);

    // Error
    double error = desired_vel - last_measured_vel_;

    // Deadband
    if (std::abs(error) < deadband_rad_s_) {
      error = 0.0;
    }

    // Derivative
    const double derivative = (error - prev_error_) / dt_;

    // Anti-windup: only integrate when not saturated
    if (!saturated_) {
      integral_ += error * dt_;
      // Integral clamp
      integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
    }

    // Feedforward: desired acceleration = d/dt[A·sin(ω·t)] = A·ω·cos(ω·t)
    const double desired_accel = amplitude_rad_s_ * omega_rad_s_ * std::cos(omega_rad_s_ * t);
    const double feedforward = kff_ * desired_accel;

    // PID output with feedforward
    double output = feedforward + kp_ * error + ki_ * integral_ + kd_ * derivative;

    // Torque saturation
    const bool was_saturated = saturated_;
    output = std::clamp(output, -torque_limit_nm_, torque_limit_nm_);
    saturated_ = (std::abs(output) >= torque_limit_nm_);

    // If we just left saturation, clear any built-up integral overshoot
    if (was_saturated && !saturated_) {
      integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
    }

    prev_error_ = error;
    // Apply sign inversion if needed (to match motor/encoder convention)
    if (invert_output_) {
      output = -output;
    }

    publish_torque(output);
  }

  void publish_torque(double torque)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {torque};
    torque_pub_->publish(msg);
  }
  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string joint_state_topic_;
  std::string command_topic_;
  std::string joint_name_;
  double amplitude_rad_s_;
  double omega_rad_s_;
  double kp_;
  double ki_;
  double kd_;
  double kff_;
  double torque_limit_nm_;
  double integral_limit_;
  double deadband_rad_s_;
  double rate_hz_;
  double dt_;
  double filter_alpha_;
  bool invert_output_;

  // State
  double integral_;
  double prev_error_;
  double last_measured_vel_;
  double filtered_vel_;
  int joint_index_;
  bool saturated_;
  rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityPidNode>());
  rclcpp::shutdown();
  return 0;
}
