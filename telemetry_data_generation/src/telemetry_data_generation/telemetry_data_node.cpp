#include "telemetry_data_generation/telemetry_data_generation.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

namespace telemetry_data_generation {
namespace {

using namespace std::chrono_literals;

class DataPublisher : public rclcpp::Node {
 public:
  DataPublisher() : Node("telemetry_data_publisher"), start_time_(rclcpp::Clock().now()) {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&DataPublisher::TimerCallback, this));
  }

 private:
  void TimerCallback() {
    const double t = (rclcpp::Clock().now() - start_time_).seconds();
    auto message = ComputeNextPose(t);
    RCLCPP_INFO(this->get_logger(), "Publishing data at %f", t);
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Time start_time_;
};

int Main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataPublisher>());
  rclcpp::shutdown();
  return 0;
}

}  // namespace
}  // telemetry_data_generation

int main(int argc, char** argv) { return telemetry_data_generation::Main(argc, argv); }