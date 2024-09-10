// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Enrique Fernández
 */


// TODO(RISHIKESAVAN) make it follow everywhere swerve_drive_controller or swervebot_controller?
#include "swerve_drive_controller/odometry.hpp"

namespace swerve_drive_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size, size_t number_of_modules)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheel_params_(), // Initialize to an empty vector
  old_module_states_(), // Initialize to an empty vector
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size) 
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::update(const std::vector<std::pair<double, double>>& module_states, const rclcpp::Time & time)
{
    // We cannot estimate the speed with very small time intervals:
    const double dt = time.seconds() - timestamp_.seconds();
    if (dt < 0.0001)
    {
        return false;  // Interval too small to integrate with
    }

    // Vector to store estimated velocities and directions of all modules
    std::vector<std::pair<double, double>> modules_est_vel;

    // Iterate over each module (wheel)
    for (size_t i = 0; i < wheel_params_.size(); ++i)
    {
        if (i >= module_states.size()) 
        {
            // Handle case where module_states doesn't have corresponding entry
            break;
        }

        // Extract wheel parameters for the current module
        double wheel_pos_x, wheel_pos_y, wheel_radius;
        std::tie(wheel_pos_x, wheel_pos_y, wheel_radius) = wheel_params_[i];

        // Extract the current state for the module
        double current_pos, current_dir;
        std::tie(current_pos, current_dir) = module_states[i];

        // Compute current position of the wheel/module
        double cur_pos = current_pos * wheel_radius;

        // Compute estimated velocity for the current module
        double old_pos = old_module_states_[i].first * wheel_radius;
        double est_vel = cur_pos - old_pos;

        // Update the estimated velocities vector
        modules_est_vel.emplace_back(est_vel, current_dir);

        // Update old position for the current module
        old_module_states_[i] = { cur_pos, current_dir };
    }

    // Pass the estimated velocities and directions for further processing
    updateFromVelocity(modules_est_vel,time);

    return true;
}

bool Odometry::updateFromVelocity(const std::vector<std::pair<double, double>>& module_velocities, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();

  // TODO(RISHIKESAVAN): UPDATE FROM THE SWERVEDRIVE KINEMATICS
  // // Compute linear and angular swerve:
  // const double linear = (left_vel + right_vel) * 0.5;
  // // Now there is a bug about scout angular velocity
  // const double angular = (right_vel - left_vel) / wheel_separation_;

  // Integrate odometry:
  integrateExact(linear, angular);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  linear_accumulator_.accumulate(linear / dt);
  angular_accumulator_.accumulate(angular / dt);

  linear_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::updateOpenLoop(double linear, double angular, const rclcpp::Time & time)
{
  /// Save last linear and angular velocity:
  linear_ = linear;
  angular_ = angular;

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrateExact(linear * dt, angular * dt);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(const std::vector<std::tuple<double, double, double>>& wheel_params)
{
    // Clear existing wheel parameters
    wheel_params_.clear();

    // Set the new wheel parameters
    wheel_params_ = wheel_params;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  heading_ += angular;
}

void Odometry::integrateExact(double linear, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
  }
}

void Odometry::resetAccumulators()
{
  linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace swerve_drive_controller
