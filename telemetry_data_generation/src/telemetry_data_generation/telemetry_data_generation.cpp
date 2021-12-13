// Copyright 2021 Agustin Alba Chicar.
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
#include "telemetry_data_generation/telemetry_data_generation.hpp"

#include <cmath>
#include <cstdint>

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

namespace telemetry_data_generation
{

geometry_msgs::msg::PoseStamped ComputeNextPose(double t)
{
  using std::atan2;
  using std::cos;
  using std::sin;

  geometry_msgs::msg::PoseStamped result;

  // Position
  result.pose.position.x = 10. * sin(2. * M_PI * 0.1 * t);
  result.pose.position.y = 10. * cos(2. * M_PI * 0.1 * t);
  result.pose.position.z = 0.1 * t;
  // Euler angles
  const double roll = 0.;
  const double pitch = atan2(0.1, 10. * 2. * M_PI * 0.1);
  double yaw = std::fmod(2. * M_PI * 0.1 * t + M_PI, 2. * M_PI);
  yaw = yaw > 0. ? yaw + M_PI : yaw - M_PI;
  // Euler angles -> quaternion
  // See https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  result.pose.orientation.x = sin(roll / 2.) * cos(pitch / 2.) * cos(yaw / 2.) - cos(roll / 2.) *
    sin(pitch / 2.) * sin(yaw / 2.);
  result.pose.orientation.y = cos(roll / 2.) * sin(pitch / 2.) * cos(yaw / 2.) + sin(roll / 2.) *
    cos(pitch / 2.) * sin(yaw / 2.);
  result.pose.orientation.z = cos(roll / 2.) * cos(pitch / 2.) * sin(yaw / 2.) - sin(roll / 2.) *
    sin(pitch / 2.) * cos(yaw / 2.);
  result.pose.orientation.w = cos(roll / 2.) * cos(pitch / 2.) * cos(yaw / 2.) + sin(roll / 2.) *
    sin(pitch / 2.) * sin(yaw / 2.);
  // Header
  const auto now = rclcpp::Clock().now();
  result.header.stamp.sec = now.nanoseconds() / 1000000000u;
  result.header.stamp.nanosec = now.nanoseconds() - result.header.stamp.sec * 1000000000u;

  return result;
}

}  // namespace telemetry_data_generation
