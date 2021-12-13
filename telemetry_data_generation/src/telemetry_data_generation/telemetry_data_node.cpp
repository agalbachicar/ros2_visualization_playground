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

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace telemetry_data_generation
{
namespace
{

using namespace std::chrono_literals;

class DataPublisher : public rclcpp::Node
{
public:
  DataPublisher()
  : Node("telemetry_data_publisher"), start_time_(rclcpp::Clock().now())
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    timer_ = this->create_wall_timer(1ms, std::bind(&DataPublisher::TimerCallback, this));
  }

private:
  void TimerCallback()
  {
    const double t = (rclcpp::Clock().now() - start_time_).seconds();
    auto message = ComputeNextPose(t);
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Time start_time_;
};

int Main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataPublisher>());
  rclcpp::shutdown();
  return 0;
}

}  // namespace
}  // namespace telemetry_data_generation

int main(int argc, char ** argv) {return telemetry_data_generation::Main(argc, argv);}
