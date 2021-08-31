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

using namespace std::chrono_literals;

namespace pathnode_sample
{

// TODO: define per path
const std::string IS_FIRST = "is_first";


// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class RelayWithPath : public pathnode::PathNode
{
public:
  explicit RelayWithPath(const rclcpp::NodeOptions & options)
      : PathNode("relay", options)
  {
    declare_parameter(IS_FIRST, false);

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void
      {
        auto is_first = get_parameter(IS_FIRST).get_value<bool>();
        if(is_first) {
          // TODO: set now
          auto msg = std::make_unique<std_msgs::msg::String>();
          msg->data = "1234";
          path_info_pub_->publish(std::move(msg));
        }

        /********************
         * original routine
         ********************/
        // if you want to check deadline in user code, write these lines
        if(exceeds_deadline(pathname_)) {
          // do_error
          return;
        }

        pub_->publish(std::move(msg));
        /********************
         * original routine from here
         ********************/
      };

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    sub_ = this->create_subscription<std_msgs::msg::String>("in", qos, callback);
    pub_ = this->create_publisher<std_msgs::msg::String>("out", qos);

    pathname_ = "path";  // actually, topics are mapped to path

    auto path_info_callback =
        [this](std_msgs::msg::String::UniquePtr msg) -> void
        {
          this->path_dead_line_ = std::stoi(msg->data);
        };
    path_info_sub_ = this->create_subscription<std_msgs::msg::String>(
        pathname_ + "_info", qos, path_info_callback);
    path_info_pub_ = this->create_publisher<std_msgs::msg::String>(pathname_ + "_info", qos);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  std::string pathname_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_info_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_info_sub_;

  // TODO: have multiple times
  rclcpp::Time path_start_time_;
  // TODO: int is right?
  int path_dead_line_;

  bool exceeds_deadline(const std::string& path)
  {
    (void)path;
    std::cout << "check now and " << std::to_string(path_dead_line_) << std::endl;
    return false;
  }
};

}  // namespace pathnode_sample

RCLCPP_COMPONENTS_REGISTER_NODE(pathnode_sample::RelayWithPath)
