#ifndef SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include <cmath>
#include <vector> // For using std::vector
#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"

namespace swerve_drive_controller
{
class Odometry
{
public:
  explicit Odometry(size_t velocity_rolling_window_size = 10, size_t number_of_modules = 4);

  void init(const rclcpp::Time & time);

  bool update(const std::vector<std::pair<double, double>>& module_states, const rclcpp::Time & time);

  bool updateFromVelocity(const std::vector<std::pair<double, double>>& module_velocities, const rclcpp::Time & time);

  void updateOpenLoop(double linear, double angular, const rclcpp::Time & time);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinear() const { return linear_; }
  double getAngular() const { return angular_; }

  void setWheelParams(const std::vector<std::tuple<double, double, double>>& wheel_params);

  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

  void integrateRungeKutta2(double linear, double angular);
  void integrateExact(double linear, double angular);
  void resetAccumulators();

  rclcpp::Time timestamp_;

  // Current pose:
  double x_;        // [m]
  double y_;        // [m]
  double heading_;  // [rad]

  // Current velocity:
  double linear_;   // [m/s]
  double angular_;  // [rad/s]

  // Wheel kinematic parameters [m]:
  std::vector<std::tuple<double, double, double>> wheel_params_; // wheel_pos_x, wheel_pos_y, wheel_radius
  
  // Previous module states [position, direction]:
  std::vector<std::pair<double, double>> old_module_states_;

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_
