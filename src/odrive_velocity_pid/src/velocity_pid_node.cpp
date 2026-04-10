// velocity_pid_node.cpp
//
// VelocityPidNode — ROS 2 node that runs a PID velocity controller for an ODrive motor.
//
// Overview:
//   - Subscribes to /joint_states and extracts the velocity of the configured joint.
//   - Applies an exponential moving average (EMA) filter to the raw velocity measurement.
//   - Drives the joint along a sine-wave reference trajectory (amplitude_rad_s * sin(omega_rad_s * t)).
//   - Computes a PID control output with velocity feedforward and acceleration feedforward.
//   - Implements derivative-on-measurement (avoids derivative kicks from setpoint steps).
//   - Implements directional anti-windup (freezes integration in the saturation direction).
//   - Publishes the torque command as a Float64MultiArray to the effort controller topic.
//   - Publishes diagnostics (desired velocity, measured velocity, tracking error) on ~/... topics.
//   - Supports full runtime reconfiguration of all tunable parameters via ROS 2 param infrastructure
//     (rqt_reconfigure, ros2 param set, YAML launch configs).
//   - Follows the official ROS 2 two-callback pattern: validation in add_on_set_parameters_callback
//     and member-variable mutation in add_post_set_parameters_callback.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <set>
#include <string>
#include <variant>
#include <vector>

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
    saturated_(false),
    last_measured_pos_(0.0)
  {
    // ── String parameters (not runtime-reconfigurable, handled before the table) ──────────────
    // These three parameters determine which topics and joint to use.  Changing them at runtime
    // would require re-creating subscriptions/publishers and re-initialising the control loop,
    // so they are intentionally excluded from the param_defs_ table and the immutable set.
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    this->declare_parameter<std::string>("command_topic", "/motor_effort_controller/commands");
    this->declare_parameter<std::string>("joint_name", "motor_joint");

    joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
    command_topic_     = this->get_parameter("command_topic").as_string();
    joint_name_        = this->get_parameter("joint_name").as_string();

    // ── Build the parameter definition table ─────────────────────────────────────────────────
    // param_defs_ is the single source of truth for every tunable numeric/bool parameter.
    // Each entry describes the parameter name, default value (which also encodes its type via
    // the variant), a pointer-to-member so the table loop can assign the value, an optional
    // validator (empty ValidatorFn = always valid), and a flag that controls whether changing
    // this parameter should reset the PID integrator.
    //
    // Adding a new parameter requires only one line here — declare, read, validate, apply at
    // runtime, and logging are all handled automatically by the loops below.
    param_defs_ = {
      // name              default               member ptr                       validator                          resets_integral
      {"amplitude_rad_s",  0.00,  &VelocityPidNode::amplitude_rad_s_,  {},                                         false},
      {"omega_rad_s",      1.25,  &VelocityPidNode::omega_rad_s_,      {},                                         false},
      {"kp",               0.3,   &VelocityPidNode::kp_,               {},                                         true},
      {"ki",               2.5,   &VelocityPidNode::ki_,               {},                                         true},
      {"kd",               0.00,  &VelocityPidNode::kd_,               {},                                         true},
      {"kff",              0.25,  &VelocityPidNode::kff_,              {},                                         false},
      {"kaff",             0.50,  &VelocityPidNode::kaff_,             {},                                         false},
      {"torque_limit_nm",  5.00,  &VelocityPidNode::torque_limit_nm_,  positive_validator("torque_limit_nm"),      false},
      {"integral_limit",   0.30,  &VelocityPidNode::integral_limit_,   positive_validator("integral_limit"),       false},
      {"deadband_rad_s",   0.00,  &VelocityPidNode::deadband_rad_s_,   {},                                         false},
      {"rate_hz",         100.0,  &VelocityPidNode::rate_hz_,          positive_validator("rate_hz"),              false},
      {"filter_alpha",     0.90,  &VelocityPidNode::filter_alpha_,     unit_range_validator("filter_alpha"),       false},
      {"invert_output",   false,  &VelocityPidNode::invert_output_,    {},                                         false},
      {"kp_pos",           0.0,   &VelocityPidNode::kp_pos_,           {},                                         false},
      {"position_setpoint", 0.0,  &VelocityPidNode::position_setpoint_, {},                                        false},
    };

    // ── Declare, read, and validate each parameter from the table ────────────────────────────
    // std::visit dispatches on the variant alternative (double vs. bool) so a single loop
    // handles both types without duplicated code paths.
    for (auto & def : param_defs_) {
      // Declare the parameter with its correct type and default value.
      std::visit([&](auto default_val) {
        using T = decltype(default_val);
        this->declare_parameter<T>(def.name, default_val);

        // Read the (possibly overridden by launch/YAML) value into the member variable.
        auto & member = std::get<T VelocityPidNode::*>(def.member);
        if constexpr (std::is_same_v<T, double>) {
          this->*member = this->get_parameter(def.name).as_double();
        } else {
          this->*member = this->get_parameter(def.name).as_bool();
        }
      }, def.default_value);

      // Run startup validation — if invalid, warn and fall back to the table default.
      if (def.validator) {
        auto vr = def.validator(get_member_value(def));
        if (!vr.successful) {
          RCLCPP_WARN(this->get_logger(), "%s; using default", vr.reason.c_str());
          std::visit([&](auto default_val) {
            using T = decltype(default_val);
            auto & member = std::get<T VelocityPidNode::*>(def.member);
            this->*member = default_val;
          }, def.default_value);
        }
      }
    }

    // Compute the nominal timer period from the (validated) rate.
    dt_ = 1.0 / rate_hz_;
    last_loop_time_ = this->now();

    // ── Subscriber ───────────────────────────────────────────────────────────────────────────
    // Listens to joint states; the callback extracts and filters the velocity for joint_name_.
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_state_callback(msg);
      });

    // ── Publishers ───────────────────────────────────────────────────────────────────────────
    // torque_pub_: effort command consumed by the hardware interface / ODrive driver.
    // The ~/... topics expose diagnostics for rqt_plot, RViz, and rosbag recording.
    torque_pub_             = this->create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
    desired_vel_pub_        = this->create_publisher<std_msgs::msg::Float64>("~/desired_velocity", 10);
    measured_vel_pub_       = this->create_publisher<std_msgs::msg::Float64>("~/measured_velocity", 10);
    velocity_error_pub_     = this->create_publisher<std_msgs::msg::Float64>("~/velocity_error", 10);
    measured_position_pub_  = this->create_publisher<std_msgs::msg::Float64>("~/measured_position", 10);
    position_error_pub_     = this->create_publisher<std_msgs::msg::Float64>("~/position_error", 10);

    // ── Control loop timer ───────────────────────────────────────────────────────────────────
    // Fires at the nominal rate; actual elapsed time is measured inside control_loop() to
    // compensate for timer jitter without relying on a fixed dt_.
    const auto timer_period = std::chrono::duration<double>(dt_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      [this]() { control_loop(); });

    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "VelocityPidNode started: joint='%s', rate=%.1f Hz, "
      "kp=%.3f ki=%.3f kd=%.3f kff=%.3f kaff=%.3f",
      joint_name_.c_str(), rate_hz_, kp_, ki_, kd_, kff_, kaff_);

    // ── Register parameter callbacks ─────────────────────────────────────────────────────────
    // Two separate callbacks follow the official ROS 2 pattern from ros2/demos
    // (demo_nodes_cpp/src/parameters/set_parameters_callback.cpp):
    //   1. on_validate_parameters  — validation only, no side-effects
    //   2. on_apply_parameters     — safe to mutate member variables (called after storage)
    on_set_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return on_validate_parameters(params);
      });
    post_set_handle_ = this->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        on_apply_parameters(params);
      });
  }

  // ── Destructor ───────────────────────────────────────────────────────────────────────────────
  // Cancel the timer first so no further control iterations are started, then publish a zero
  // torque command so the motor comes to a safe, unpowered state rather than holding last output.
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
  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //  Parameter table infrastructure
  // ══════════════════════════════════════════════════════════════════════════════════════════════

  // MemberPtr holds a pointer-to-member for either a double or bool field of this class.
  // ParamValue is the corresponding runtime variant used to pass values to validators.
  // ValidatorFn is a callable that inspects a ParamValue and returns a SetParametersResult.
  using MemberPtr   = std::variant<double VelocityPidNode::*, bool VelocityPidNode::*>;
  using ParamValue  = std::variant<double, bool>;
  using ValidatorFn = std::function<rcl_interfaces::msg::SetParametersResult(const ParamValue &)>;

  // ParamDef — describes one tunable parameter in its entirety.
  //
  //   name            : The ROS parameter name (used with declare_parameter / get_parameter).
  //   default_value   : ParamValue variant; the alternative held also determines the type
  //                     (double vs. bool) used when declaring and reading the parameter.
  //   member          : MemberPtr to the corresponding class field so loops can assign values
  //                     without knowing the type at the call site.
  //   validator       : Optional ValidatorFn; empty means always valid.  Shared between the
  //                     constructor startup check and the runtime on_validate_parameters callback.
  //   resets_integral : When true, changing this parameter triggers an integral reset in
  //                     on_apply_parameters to avoid integral windup carry-over after a gain change.
  struct ParamDef
  {
    std::string  name;
    ParamValue   default_value;
    MemberPtr    member;
    ValidatorFn  validator;
    bool         resets_integral;
  };

  // ── Reusable validator factories ──────────────────────────────────────────────────────────────
  // Each factory returns a ValidatorFn closure that captures only the parameter name string.
  // The same closure is stored in param_defs_ and called identically from the constructor and
  // from on_validate_parameters, eliminating duplicated validation logic.

  // positive_validator — rejects any value <= 0.0 (used for torque_limit_nm, integral_limit, rate_hz).
  static ValidatorFn positive_validator(const std::string & param_name)
  {
    return [param_name](const ParamValue & v) {
      rcl_interfaces::msg::SetParametersResult r;
      r.successful = (std::get<double>(v) > 0.0);
      if (!r.successful) {
        r.reason = param_name + " must be positive.";
      }
      return r;
    };
  }

  // unit_range_validator — rejects values outside [0.0, 1.0) — i.e., greater than or equal to
  // 0.0 and strictly less than 1.0 (used for filter_alpha).
  static ValidatorFn unit_range_validator(const std::string & param_name)
  {
    return [param_name](const ParamValue & v) {
      rcl_interfaces::msg::SetParametersResult r;
      const double val = std::get<double>(v);
      r.successful = (val >= 0.0 && val < 1.0);
      if (!r.successful) {
        r.reason = param_name + " must be in [0.0, 1.0).";
      }
      return r;
    };
  }

  // get_member_value — reads the current value of the member field described by def into a
  // ParamValue variant.  std::visit dispatches on the MemberPtr alternative (double vs. bool)
  // so the correct member type is deduced automatically.
  ParamValue get_member_value(const ParamDef & def) const
  {
    return std::visit([this](auto member_ptr) -> ParamValue {
      return this->*member_ptr;
    }, def.member);
  }

  // ── Immutable parameters ──────────────────────────────────────────────────────────────────────
  // These parameters cannot be changed at runtime because doing so would require reconstructing
  // the subscription, publishers, or timer — operations that are not safe in a running node.
  // Using a set gives an O(log n) membership test and makes the list easy to extend.
  static inline const std::set<std::string> kImmutableParams = {
    "joint_state_topic", "command_topic", "joint_name", "rate_hz"
  };

  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //  on_validate_parameters — registered with add_on_set_parameters_callback
  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //
  // This callback is called by the ROS 2 parameter infrastructure BEFORE the new values are
  // stored.  Its sole responsibility is to accept or reject the proposed change.
  //
  // IMPORTANT: No member variables must be mutated here.  Per the official ROS 2 documentation
  // and the ros2/demos pattern (demo_nodes_cpp/src/parameters/set_parameters_callback.cpp), the
  // on_set callback is validation-only.  If member variables were mutated here and a *later*
  // parameter in the same batch then failed validation, the member state would be out of sync
  // with the parameter store — a subtle but real bug in the original code.
  rcl_interfaces::msg::SetParametersResult on_validate_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Reject any attempt to change a parameter that cannot be reconfigured at runtime.
    for (const auto & param : parameters) {
      if (kImmutableParams.count(param.get_name())) {
        result.successful = false;
        result.reason = "Parameter '" + param.get_name() +
          "' cannot be changed at runtime (requires node restart).";
        return result;
      }
    }

    // Run each incoming value through the validator stored in param_defs_ (if any).
    // All parameters in the batch are validated before any are applied; if one fails,
    // the entire batch is rejected and no side-effects occur.
    for (const auto & param : parameters) {
      for (const auto & def : param_defs_) {
        if (def.name == param.get_name() && def.validator) {
          const ParamValue incoming =
            (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            ? ParamValue{param.as_double()}
            : ParamValue{param.as_bool()};
          auto vr = def.validator(incoming);
          if (!vr.successful) {
            return vr;
          }
        }
      }
    }

    return result;
  }

  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //  on_apply_parameters — registered with add_post_set_parameters_callback
  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //
  // This callback is called AFTER the new parameter values have been successfully stored by the
  // ROS 2 parameter infrastructure (i.e., after on_validate_parameters returned successful).
  // It is safe to mutate member variables here because:
  //   (a) validation has already passed for all parameters in the batch, and
  //   (b) the parameter store already reflects the new values.
  //
  // For PID gains (resets_integral == true), the integrator is reset to prevent carry-over
  // windup from a previous operating point.
  void on_apply_parameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    bool pid_gains_changed = false;

    for (const auto & param : parameters) {
      for (auto & def : param_defs_) {
        if (def.name != param.get_name()) {
          continue;
        }

        // Use std::visit to assign the value to the correct member type (double or bool)
        // and log it without a type-specific if/else at the call site.
        std::visit([&](auto member_ptr) {
          using T = std::remove_reference_t<decltype(this->*member_ptr)>;
          if constexpr (std::is_same_v<T, double>) {
            this->*member_ptr = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter updated: %s = %.4f",
              def.name.c_str(), param.as_double());
          } else {
            this->*member_ptr = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Parameter updated: %s = %s",
              def.name.c_str(), param.as_bool() ? "true" : "false");
          }
        }, def.member);

        if (def.resets_integral) {
          pid_gains_changed = true;
        }
        break;
      }
    }

    // If any PID gain changed, reset the integrator so accumulated error from the old
    // operating point does not bias the controller under the new gains.
    if (pid_gains_changed) {
      integral_ = 0.0;
      RCLCPP_INFO(this->get_logger(),
        "PID gains changed — integral reset to 0. New gains: kp=%.4f ki=%.4f kd=%.4f",
        kp_, ki_, kd_);
    }
  }

  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //  Joint-state callback
  // ══════════════════════════════════════════════════════════════════════════════════════════════

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Cache the joint index on first lookup and re-validate on each message in case the joint
    // order in the message changes (which can happen during bring-up).
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

      // When invert_output is true the motor spins in the direction the encoder reports
      // as negative, so negate the measurement to keep the PID in a negative-feedback
      // loop rather than a positive-feedback loop.
      if (invert_output_) {
        raw = -raw;
      }

      // Exponential moving average (EMA) filter:
      //   filtered = alpha * filtered_prev + (1 - alpha) * raw
      // A higher filter_alpha gives more smoothing at the cost of added lag.
      // On the first sample, initialise the filter to the raw value to avoid a step response.
      if (!filter_initialized_) {
        filtered_vel_ = raw;
        filter_initialized_ = true;
      } else {
        filtered_vel_ = filter_alpha_ * filtered_vel_ + (1.0 - filter_alpha_) * raw;
      }
      last_measured_vel_ = filtered_vel_;
    }

    // Read position estimate (already smooth from ODrive encoder; no filtering needed).
    if (idx < msg->position.size()) {
      double raw_pos = msg->position[idx];
      if (invert_output_) {
        raw_pos = -raw_pos;
      }
      last_measured_pos_ = raw_pos;
    }
  }

  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //  Control loop
  // ══════════════════════════════════════════════════════════════════════════════════════════════

  void control_loop()
  {
    if (joint_index_ < 0) {
      // Joint not yet found — publish zero torque as a safety measure.
      publish_torque(0.0);
      return;
    }

    // Measure actual elapsed time since the last iteration to compensate for timer jitter.
    // Fall back to the nominal dt_ only if the wall-clock delta is non-positive (e.g., during
    // simulation time resets).
    const auto now = this->now();
    const double actual_dt = (now - last_loop_time_).seconds();
    last_loop_time_ = now;
    const double dt = (actual_dt > 0.0) ? actual_dt : dt_;

    // Elapsed time from node start — used to evaluate the sine reference trajectory.
    const double t = (now - start_time_).seconds();

    // Sine-wave reference trajectory: desired_vel = A · sin(ω · t)
    const double desired_vel = amplitude_rad_s_ * std::sin(omega_rad_s_ * t);

    // Tracking error; clamped to zero inside the deadband to suppress small oscillations.
    double error = desired_vel - last_measured_vel_;
    if (std::abs(error) < deadband_rad_s_) {
      error = 0.0;
    }

    // Derivative-on-measurement: use the negative rate of change of the measured velocity
    // rather than the rate of change of the error.  This avoids a "derivative kick" that
    // would otherwise occur whenever the setpoint changes step-wise.
    const double derivative = -(last_measured_vel_ - prev_measured_vel_) / dt;

    // Directional anti-windup: allow integration only when it would unwind the accumulator
    // (error opposes the sign of integral_), and freeze it when the output is saturated and
    // further integration would deepen the windup.
    if (!saturated_ || (error * integral_ <= 0.0)) {
      integral_ += error * dt;
      integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
    }

    // Feedforward terms:
    //   velocity FF:     kff  * desired_vel    (proportional to reference velocity)
    //   acceleration FF: kaff * desired_accel  (proportional to d/dt[desired_vel])
    // For a sine reference: desired_accel = A · ω · cos(ω · t)
    const double desired_accel = amplitude_rad_s_ * omega_rad_s_ * std::cos(omega_rad_s_ * t);
    const double feedforward   = kff_ * desired_vel + kaff_ * desired_accel;

    // Position feedforward: spring-like restoring force toward position_setpoint_.
    // Acts as a stabilising anchor that prevents the position from drifting over time.
    // kp_pos = 0.0 (default) disables this term entirely for backward compatibility.
    const double position_error = last_measured_pos_ - position_setpoint_;
    const double position_ff    = -kp_pos_ * position_error;

    // Full PID + feedforward output.
    double output = feedforward + position_ff + kp_ * error + ki_ * integral_ + kd_ * derivative;

    // Torque saturation: clamp the output and track whether we are currently saturated.
    const bool was_saturated = saturated_;
    output    = std::clamp(output, -torque_limit_nm_, torque_limit_nm_);
    saturated_ = (std::abs(output) >= torque_limit_nm_);

    // On the first cycle after leaving saturation, re-clamp the integral to remove any
    // overshoot that accumulated while the anti-windup was active.
    if (was_saturated && !saturated_) {
      integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
    }

    prev_measured_vel_ = last_measured_vel_;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
      "[PID] des=%.3f meas=%.3f err=%.3f out=%.3f sat=%d",
      desired_vel, last_measured_vel_, error, output, static_cast<int>(saturated_));

    publish_torque(output);

    // Publish diagnostic Float64 topics with a small lambda to avoid repeating the
    // message-construction boilerplate three times.
    auto pub_float64 = [](auto & publisher, double val) {
      std_msgs::msg::Float64 m;
      m.data = val;
      publisher->publish(m);
    };
    pub_float64(desired_vel_pub_,       desired_vel);
    pub_float64(measured_vel_pub_,      last_measured_vel_);
    pub_float64(velocity_error_pub_,    error);
    pub_float64(measured_position_pub_, last_measured_pos_);
    pub_float64(position_error_pub_,    position_error);
  }

  void publish_torque(double torque)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {torque};
    torque_pub_->publish(msg);
  }

  // ── ROS interfaces ────────────────────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr desired_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr measured_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr measured_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_error_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // Callback handles must be stored as members; if they go out of scope the callbacks are
  // automatically deregistered by rclcpp.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_handle_;

  // ── Parameters ────────────────────────────────────────────────────────────────────────────────
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
  double kp_pos_;
  double position_setpoint_;

  // ── State ─────────────────────────────────────────────────────────────────────────────────────
  double integral_;
  double prev_measured_vel_;
  double last_measured_vel_;
  double filtered_vel_;
  bool filter_initialized_;
  int joint_index_;
  bool saturated_;
  double last_measured_pos_;
  rclcpp::Time start_time_;
  rclcpp::Time last_loop_time_;

  // ── Parameter table ───────────────────────────────────────────────────────────────────────────
  // Populated in the constructor; used by the declare/read/validate loop and both param callbacks.
  std::vector<ParamDef> param_defs_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityPidNode>());
  rclcpp::shutdown();
  return 0;
}
