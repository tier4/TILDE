#include <string>
#include <iostream>
#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "pathnode/path_node.hpp"
#include "path_info_msg/msg/path_info.hpp"

using namespace std::chrono_literals;

namespace pathnode_sample
{

// TODO: define per path
const std::string IS_FIRST = "is_first";
const std::string PATH_DEADLINE_SEC = "path_deadline_sec";
const std::string PATH_DEADLINE_NS = "path_deadline_ns";

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class RelayWithPath : public pathnode::PathNode
{
public:
  explicit RelayWithPath(const rclcpp::NodeOptions & options)
      : PathNode("relay", options)
  {
    /* newly introduced variables for path */
    const std::string path_name = "sample_path";
    declare_parameter<bool>(IS_FIRST, false);
    // 	whole seconds (valid values are >= 0)
    declare_parameter<int64_t>(PATH_DEADLINE_SEC, (int64_t) 0);
    // nanoseconds (valid values are [0, 999999999])
    declare_parameter<int64_t>(PATH_DEADLINE_NS, (int64_t)(10 * 1000 * 1000));
    /* from here */


    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::String>("out", qos);

    auto callback =
        [this, &path_name](const std_msgs::msg::String &msg) -> void
      {
        std::cout << "sample_relay_with_path callback" << std::endl;
        if(exceeds_deadline(path_name)) {
          // do_error
          return;
        }

        pub_->publish(msg);
      };


    // TODO check range
    auto deadline_tv_sec = get_parameter(PATH_DEADLINE_SEC).get_value<int64_t>();
    auto deadline_tv_nsec = get_parameter(PATH_DEADLINE_NS).get_value<int64_t>();

    pathnode::PathNodeSubscriptionOptions path_node_options;
    path_node_options.path_name_ = path_name;
    path_node_options.path_deadline_duration_ = rclcpp::Duration(deadline_tv_sec,
                                                                 deadline_tv_nsec);

    path_node_options.publish_topic_names_.push_back(pub_->get_topic_name());

    // Create a publisher with a custom Quality of Service profile.
    sub_ = this->create_path_node_subscription<std_msgs::msg::String>(
        path_node_options,
        "in", qos, callback);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace pathnode_sample

RCLCPP_COMPONENTS_REGISTER_NODE(pathnode_sample::RelayWithPath)
