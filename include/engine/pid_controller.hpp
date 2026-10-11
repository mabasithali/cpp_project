#pragma once

#include <algorithm>
#include <concepts>
#include <limits>
#include <utility>

namespace engine {

/**
 * @brief Configuration parameters for the PID controller.
 *
 * @tparam T Floating-point type (e.g., float, double).
 */
template <std::floating_point T = double>
struct PidGains {
  T kp{T(0)};  ///< Proportional gain
  T ki{T(0)};  ///< Integral gain
  T kd{T(0)};  ///< Derivative gain
};

/**
 * @brief Output limits and anti-windup configuration.
 *
 * @tparam T Floating-point type (e.g., float, double).
 */
template <std::floating_point T = double>
struct PidLimits {
  T output_min{-std::numeric_limits<T>::infinity()};    ///< Minimum controller output
  T output_max{std::numeric_limits<T>::infinity()};     ///< Maximum controller output
  T integral_min{-std::numeric_limits<T>::infinity()};  ///< Minimum integral accumulator limit
  T integral_max{std::numeric_limits<T>::infinity()};   ///< Maximum integral accumulator limit
};

/**
 * @brief A production-ready discrete PID controller in C++20.
 *
 * Features proportional, integral (with clamping anti-windup), and derivative terms,
 * configurable sampling time dt, bumpless reset capability, and modern C++ design principles
 * (constexpr, [[nodiscard]], explicit constructors, and template support).
 *
 * @tparam T Floating-point type (default: double).
 */
template <std::floating_point T = double>
class PidControllerTemplate {
 public:
  /**
   * @brief Construct a new PidControllerTemplate with gains, limits, and sampling time.
   *
   * @param gains Proportional, integral, and derivative gains.
   * @param limits Output and integral clamping limits.
   * @param dt Sampling time interval (> 0).
   */
  explicit constexpr PidControllerTemplate(PidGains<T> gains = {}, PidLimits<T> limits = {},
                                           T dt = T(0.01))
      : gains_(gains), limits_(limits), dt_(dt) {
    reset();
  }

  /**
   * @brief Compute the control output given a target setpoint and current measurement.
   *
   * @param setpoint Desired target value.
   * @param measurement Current measured process value.
   * @return [[nodiscard]] Computed control effort.
   */
  [[nodiscard]] constexpr T compute(T setpoint, T measurement) {
    const T error = setpoint - measurement;

    // Proportional term
    const T p_term = gains_.kp * error;

    // Integral term with trapezoidal integration (or rectangular) and anti-windup clamping
    // We update the integral state conditionally or clamp the accumulator.
    // Standard clamping anti-windup:
    // Only accumulate integral if we are not saturated in the direction of the error,
    // or clamp the integral term directly within [integral_min, integral_max].
    integral_ += gains_.ki * error * dt_;
    integral_ = std::clamp(integral_, limits_.integral_min, limits_.integral_max);
    const T i_term = integral_;

    // Derivative term (on measurement error or measurement itself to avoid derivative kick)
    // Using derivative on error: (error - prev_error_) / dt_
    T d_term = T(0);
    if (dt_ > T(0)) {
      const T derivative = (error - prev_error_) / dt_;
      d_term = gains_.kd * derivative;
    }
    prev_error_ = error;

    // Sum terms
    const T raw_output = p_term + i_term + d_term;

    // Output clamping
    const T output = std::clamp(raw_output, limits_.output_min, limits_.output_max);

    // Back-calculation / clamping anti-windup refinement:
    // If output is clamped and the sign of error matches the sign of output saturation,
    // we can prevent further windup (or the direct accumulator clamp above handles standard
    // bounds).

    return output;
  }

  /**
   * @brief Reset the internal states (integral accumulator, previous error) to zero.
   */
  constexpr void reset() noexcept {
    integral_ = T(0);
    prev_error_ = T(0);
  }

  // Getters and Setters

  [[nodiscard]] constexpr const PidGains<T>& gains() const noexcept { return gains_; }
  constexpr void set_gains(const PidGains<T>& gains) noexcept { gains_ = gains; }

  [[nodiscard]] constexpr const PidLimits<T>& limits() const noexcept { return limits_; }
  constexpr void set_limits(const PidLimits<T>& limits) noexcept { limits_ = limits; }

  [[nodiscard]] constexpr T dt() const noexcept { return dt_; }
  constexpr void set_dt(T dt) noexcept {
    if (dt > T(0)) {
      dt_ = dt;
    }
  }

  [[nodiscard]] constexpr T integral_state() const noexcept { return integral_; }
  constexpr void set_integral_state(T integral) noexcept {
    integral_ = std::clamp(integral, limits_.integral_min, limits_.integral_max);
  }

 private:
  PidGains<T> gains_{};
  PidLimits<T> limits_{};
  T dt_{T(0.01)};

  T integral_{T(0)};
  T prev_error_{T(0)};
};

// Convenient default type alias for standard usage (double precision)
using PidController = PidControllerTemplate<double>;

}  // namespace engine
