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
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace quat2euler
{
namespace
{

// Gets the quaternion from a pose stamped message and republishes it as point
// stamped. For reference: x->roll, y->pitch, z->yaw.
class Quaternion2EulerAnglesRepublisher : public rclcpp::Node
{
public:
  Quaternion2EulerAnglesRepublisher()
  : Node("telemetry_data_publisher")
  {
    using std::placeholders::_1;
    publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("euler_angles", 10);
    subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", 10, std::bind(&Quaternion2EulerAnglesRepublisher::PoseHandler, this, _1));
  }

private:
  void PoseHandler(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    geometry_msgs::msg::PointStamped point;
    point.header = pose->header;
    point.point = Convert(pose->pose.orientation);
    publisher_->publish(point);
  }

  // See https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  static geometry_msgs::msg::Point Convert(const geometry_msgs::msg::Quaternion & q)
  {
    // roll (x-axis rotation)
    const double sinr_cosp = 2. * (q.w * q.x + q.y * q.z);
    const double cosr_cosp = 1 - 2. * (q.x * q.x + q.y * q.y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    const double sinp = 2. * (q.w * q.y - q.z * q.x);
    // use 90 degrees if out of range
    const double pitch = std::abs(sinp) >= 1. ? std::copysign(M_PI / 2, sinp) : std::asin(sinp);

    // yaw (z-axis rotation)
    const double siny_cosp = 2. * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1. - 2. * (q.y * q.y + q.z * q.z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    geometry_msgs::msg::Point euler_angles;
    euler_angles.x = roll;
    euler_angles.y = pitch;
    euler_angles.z = yaw;
    return euler_angles;
  }

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
};

int Main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Quaternion2EulerAnglesRepublisher>());
  rclcpp::shutdown();
  return 0;
}

}  // namespace
}  // namespace quat2euler

int main(int argc, char ** argv) {return quat2euler::Main(argc, argv);}
