#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class VelocityPidNode : public rclcpp::Node
{
public:
  VelocityPidNode()
  : Node("velocity_pid_node"),
    integral_(0.0),
    prev_measured_vel_(0.0),
    last_measured_vel_(0.0),
    filtered_vel_(0.0),
    filter_initialized_(false),
    joint_index_(-1),
    saturated_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    this->declare_parameter<std::string>("command_topic", "/motor_effort_controller/commands");
    this->declare_parameter<std::string>("joint_name", "motor_joint");

    this->declare_parameter<double>("amplitude_rad_s", 15.0);
    this->declare_parameter<double>("omega_rad_s", 3.14);

    this->declare_parameter<double>("kp", 0.03);
    this->declare_parameter<double>("ki", 0.1);
    this->declare_parameter<double>("kd", 0.0);
    this->declare_parameter<double>("kff", 0.015);
    this->declare_parameter<double>("kaff", 0.003);

    this->declare_parameter<double>("torque_limit_nm", 0.5);
    this->declare_parameter<double>("integral_limit", 0.3);
    this->declare_parameter<double>("deadband_rad_s", 0.0);
    this->declare_parameter<double>("rate_hz", 100.0);
    this->declare_parameter<double>("filter_alpha", 0.7);
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
    kaff_ = this->get_parameter("kaff").as_double();

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
      RCLCPP_WARN(this->get_logger(), "torque_limit_nm must be positive; defaulting to 0.5");
      torque_limit_nm_ = 0.5;
    }
    if (integral_limit_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "integral_limit must be positive; defaulting to 0.3");
      integral_limit_ = 0.3;
    }
    if (filter_alpha_ < 0.0 || filter_alpha_ >= 1.0) {
      RCLCPP_WARN(this->get_logger(), "filter_alpha must be in [0.0, 1.0); defaulting to 0.7");
      filter_alpha_ = 0.7;
    }

    dt_ = 1.0 / rate_hz_;

    last_loop_time_ = this->now();

    // Subscriber
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_state_callback(msg);
      });

    // Publisher
    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
    desired_vel_pub_   = this->create_publisher<std_msgs::msg::Float64>("~/desired_velocity", 10);
    measured_vel_pub_  = this->create_publisher<std_msgs::msg::Float64>("~/measured_velocity", 10);
    velocity_error_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/velocity_error", 10);

    // Control loop timer
    const auto period_ms = std::chrono::duration<double>(dt_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period_ms),
      [this]() { control_loop(); });

    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "VelocityPidNode started: joint='%s', rate=%.1f Hz, kp=%.3f ki=%.3f kd=%.3f kff=%.3f kaff=%.3f",
      joint_name_.c_str(), rate_hz_, kp_, ki_, kd_, kff_, kaff_);

    // Register parameter change callback for runtime reconfiguration
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return on_set_parameters(params);
      });
  }

  ~VelocityPidNode()
  {
    if (timer_) {
	timer_->cancel();
    }
    if (torque_pub_) {
	publish_torque(0.0);
	RCLCPP_INFO(this->get_logger(), "Shutdown: 0 torque");
    }
  }
	
private:
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Check for non-reconfigurable parameters first
    for (const auto & param : parameters) {
      const auto & name = param.get_name();
      if (name == "joint_state_topic" || name == "command_topic" ||
          name == "joint_name" || name == "rate_hz")
      {
        result.successful = false;
        result.reason = "Parameter '" + name +
          "' cannot be changed at runtime (requires node restart).";
        return result;
      }
    }

    // Validate all parameters before applying any
    for (const auto & param : parameters) {
      const auto & name = param.get_name();
      if (name == "torque_limit_nm" && param.as_double() <= 0.0) {
        result.successful = false;
        result.reason = "torque_limit_nm must be positive.";
        return result;
      }
      if (name == "integral_limit" && param.as_double() <= 0.0) {
        result.successful = false;
        result.reason = "integral_limit must be positive.";
        return result;
      }
      if (name == "filter_alpha" &&
          (param.as_double() < 0.0 || param.as_double() >= 1.0))
      {
        result.successful = false;
        result.reason = "filter_alpha must be in [0.0, 1.0).";
        return result;
      }
    }

    // Apply validated parameters
    bool pid_gains_changed = false;
    for (const auto & param : parameters) {
      const auto & name = param.get_name();
      if (name == "kp") {
        kp_ = param.as_double();
        pid_gains_changed = true;
        RCLCPP_INFO(this->get_logger(), "Parameter updated: kp = %.4f", kp_);
      } else if (name == "ki") {
        ki_ = param.as_double();
        pid_gains_changed = true;
        RCLCPP_INFO(this->get_logger(), "Parameter updated: ki = %.4f", ki_);
      } else if (name == "kd") {
        kd_ = param.as_double();
        pid_gains_changed = true;
        RCLCPP_INFO(this->get_logger(), "Parameter updated: kd = %.4f", kd_);
      } else if (name == "kff") {
        kff_ = param.as_double();
        // Feedforward gains do not affect the integral term, so no reset needed.
        RCLCPP_INFO(this->get_logger(), "Parameter updated: kff = %.4f", kff_);
      } else if (name == "kaff") {
        kaff_ = param.as_double();
        // Feedforward gains do not affect the integral term, so no reset needed.
        RCLCPP_INFO(this->get_logger(), "Parameter updated: kaff = %.4f", kaff_);
      } else if (name == "amplitude_rad_s") {
        amplitude_rad_s_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Parameter updated: amplitude_rad_s = %.4f", amplitude_rad_s_);
      } else if (name == "omega_rad_s") {
        omega_rad_s_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Parameter updated: omega_rad_s = %.4f", omega_rad_s_);
      } else if (name == "torque_limit_nm") {
        torque_limit_nm_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Parameter updated: torque_limit_nm = %.4f", torque_limit_nm_);
      } else if (name == "integral_limit") {
        integral_limit_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Parameter updated: integral_limit = %.4f", integral_limit_);
      } else if (name == "deadband_rad_s") {
        deadband_rad_s_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Parameter updated: deadband_rad_s = %.4f", deadband_rad_s_);
      } else if (name == "filter_alpha") {
        filter_alpha_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Parameter updated: filter_alpha = %.4f", filter_alpha_);
      } else if (name == "invert_output") {
        invert_output_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Parameter updated: invert_output = %s",
          invert_output_ ? "true" : "false");
      }
    }

    if (pid_gains_changed) {
      integral_ = 0.0;
      RCLCPP_INFO(this->get_logger(),
        "PID gains changed — integral reset to 0. New gains: kp=%.4f ki=%.4f kd=%.4f",
        kp_, ki_, kd_);
    }

    return result;
  }

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
      // When invert_output is true the motor spins in the direction the encoder
      // reports as negative, so negate the measurement here to keep the PID
      // in a negative-feedback loop rather than a positive one.
      if (invert_output_) {
        raw = -raw;
      }
      if (!filter_initialized_) {
        filtered_vel_ = raw;
        filter_initialized_ = true;
      } else {
        filtered_vel_ = filter_alpha_ * filtered_vel_ + (1.0 - filter_alpha_) * raw;
      }
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

    // Measure actual elapsed time since last iteration (Bug 3 fix)
    const auto now = this->now();
    const double actual_dt = (now - last_loop_time_).seconds();
    last_loop_time_ = now;
    const double dt = (actual_dt > 0.0) ? actual_dt : dt_;

    // Elapsed time from node start
    const double t = (now - start_time_).seconds();

    // Desired velocity (sine trajectory)
    const double desired_vel = amplitude_rad_s_ * std::sin(omega_rad_s_ * t);

    // Error
    double error = desired_vel - last_measured_vel_;

    // Deadband
    if (std::abs(error) < deadband_rad_s_) {
      error = 0.0;
    }

    // Derivative-on-measurement (Bug 2 fix): avoids derivative kick from setpoint changes
    const double derivative = -(last_measured_vel_ - prev_measured_vel_) / dt;

    // Directional anti-windup: allow integration that UNWINDS the integral
    // (error opposes integral sign), but freeze integration that would
    // wind up further in the saturation direction.
    if (!saturated_ || (error * integral_ <= 0.0)) {
      integral_ += error * dt;
      integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
    }

    // Feedforward: velocity-FF + acceleration-FF (Bug 1 fix)
    // desired_vel = A·sin(ω·t), desired_accel = d/dt[A·sin(ω·t)] = A·ω·cos(ω·t)
    const double desired_accel = amplitude_rad_s_ * omega_rad_s_ * std::cos(omega_rad_s_ * t);
    const double feedforward = (kff_ * desired_vel) + (kaff_ * desired_accel);

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

    prev_measured_vel_ = last_measured_vel_;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
      "[PID] des=%.3f meas=%.3f err=%.3f out=%.3f sat=%d",
      desired_vel, last_measured_vel_, error, output, static_cast<int>(saturated_));

    // Publishing the data to topic. Really don't need the function. 
    publish_torque(output);

    std_msgs::msg::Float64 desired_msg;
    desired_msg.data = desired_vel;
    desired_vel_pub_->publish(desired_msg);

    std_msgs::msg::Float64 measured_msg;
    measured_msg.data = last_measured_vel_;
    measured_vel_pub_->publish(measured_msg);

    std_msgs::msg::Float64 error_msg;
    error_msg.data = error;
    velocity_error_pub_->publish(error_msg);
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
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr desired_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr measured_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_error_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

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
  double kaff_;
  double torque_limit_nm_;
  double integral_limit_;
  double deadband_rad_s_;
  double rate_hz_;
  double dt_;
  double filter_alpha_;
  bool invert_output_;

  // State
  double integral_;
  double prev_measured_vel_;
  double last_measured_vel_;
  double filtered_vel_;
  bool filter_initialized_;
  int joint_index_;
  bool saturated_;
  rclcpp::Time start_time_;
  rclcpp::Time last_loop_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityPidNode>());
  rclcpp::shutdown();
  return 0;
}
