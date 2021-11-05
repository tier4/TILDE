#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pathnode/sub_timing_advertise_node.hpp"
#include "pathnode/timing_advertise_publisher.hpp"

using namespace std::chrono_literals;

namespace pathnode_sample
{
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class P2Goal : public pathnode::SubTimingAdvertiseNode
{
public:
  explicit P2Goal(const rclcpp::NodeOptions & options)
      : SubTimingAdvertiseNode("talker", options)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto sub_callback =
        [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void
        {
          (void) msg;
          RCLCPP_INFO(this->get_logger(), "recieved");
        };
    sub_pc_ = this->create_timing_advertise_subscription<sensor_msgs::msg::PointCloud2>("in", qos, sub_callback);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
};

}  // namespace pathnode_sample

RCLCPP_COMPONENTS_REGISTER_NODE(pathnode_sample::P2Goal)
