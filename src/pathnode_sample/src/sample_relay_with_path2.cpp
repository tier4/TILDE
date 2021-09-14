#include <string>
#include <iostream>
#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>
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
const std::string PATH_VALID_MIN_SEC = "path_valid_min_sec";
const std::string PATH_VALID_MIN_NS  = "path_valid_min_ns";
const std::string PATH_VALID_MAX_SEC = "path_valid_max_sec";
const std::string PATH_VALID_MAX_NS  = "path_valid_max_ns";
const std::string WAIT_MSEC          = "wait_msec";

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class RelayWithPath2 : public pathnode::PathNode
{
public:
  explicit RelayWithPath2(const rclcpp::NodeOptions & options)
      : PathNode("relay", options), path_name_("sample_path")
  {
    /* newly introduced variables for path */
    declare_parameter<bool>(IS_FIRST, false);
    // 	whole seconds (valid values are >= 0)
    declare_parameter<int64_t>(PATH_VALID_MIN_SEC, (int64_t) 0);
    // nanoseconds (valid values are [0, 999999999])
    declare_parameter<int64_t>(PATH_VALID_MIN_NS, (int64_t)( 1 * 1000 * 1000));
    // 	whole seconds (valid values are >= 0)
    declare_parameter<int64_t>(PATH_VALID_MAX_SEC, (int64_t) 0);
    // nanoseconds (valid values are [0, 999999999])
    declare_parameter<int64_t>(PATH_VALID_MAX_NS, (int64_t)(10 * 1000 * 1000));
    /* from here */

    // for dummy sleep
    declare_parameter<int64_t>(WAIT_MSEC, (int64_t) 0);

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::String>("out", qos);

    auto wait_msec = get_parameter(WAIT_MSEC).get_value<int64_t>();

    // now we can define unique_ptr version
    auto callback_unique_ptr =
        [this, wait_msec](std_msgs::msg::String::UniquePtr msg) -> void
      {
        std::cout << "relay: " << msg->data << std::endl;
        pub_->publish(std::move(msg));
      };

    auto callback_const =
        [this, wait_msec](const std_msgs::msg::String &msg) -> void
      {
        std::cout << "relay: " << msg.data << std::endl;
        pub_->publish(msg);
        std::cout << "published" << std::endl;
      };


    // TODO check range
    auto valid_min_tv_sec  = get_parameter(PATH_VALID_MIN_SEC).get_value<int64_t>();
    auto valid_min_tv_nsec = get_parameter(PATH_VALID_MIN_NS).get_value<int64_t>();
    auto valid_max_tv_sec  = get_parameter(PATH_VALID_MAX_SEC).get_value<int64_t>();
    auto valid_max_tv_nsec = get_parameter(PATH_VALID_MAX_NS).get_value<int64_t>();

    pathnode::PathNodeSubscriptionOptions path_node_options;
    path_node_options.path_name_ = path_name_;
    path_node_options.is_first_ = get_parameter(IS_FIRST).get_value<bool>();
    path_node_options.valid_min_ = rclcpp::Duration(valid_min_tv_sec, valid_min_tv_nsec);
    path_node_options.valid_max_ = rclcpp::Duration(valid_max_tv_sec, valid_max_tv_nsec);

    // baggy
    /*
    sub_ = this->create_path_node_subscription<std_msgs::msg::String>(
        path_node_options,
        "in", qos, callback);
    */

    sub_ = this->create_path_node_subscription<std_msgs::msg::String>(
        path_node_options,
        "in", qos, callback_unique_ptr);
  }

 private:
  std::string path_name_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace pathnode_sample

RCLCPP_COMPONENTS_REGISTER_NODE(pathnode_sample::RelayWithPath2)
