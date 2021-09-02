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
      : PathNode("relay", options),
        CLOCK_TYPE(RCL_SYSTEM_TIME),
        path_start_time_(0, 0, CLOCK_TYPE), path_deadline_duration_(0, 0)
  {
    declare_parameter<bool>(IS_FIRST, false);
    // 	whole seconds (valid values are >= 0)
    declare_parameter<int64_t>(PATH_DEADLINE_SEC, (int64_t) 0);
    // nanoseconds (valid values are [0, 999999999])
    declare_parameter<int64_t>(PATH_DEADLINE_NS, (int64_t)(10 * 1000 * 1000));

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void
      {
        const auto is_first = get_parameter(IS_FIRST).get_value<bool>();

        // TODO: set now
        auto m = std::make_unique<path_info_msg::msg::PathInfo>();
        if(is_first) {
          this->path_start_time_ = now();

          // TODO check range
          auto deadline_tv_sec = get_parameter(PATH_DEADLINE_SEC).get_value<int64_t>();
          auto deadline_tv_ns = get_parameter(PATH_DEADLINE_NS).get_value<int64_t>();

          this->path_deadline_duration_ = rclcpp::Duration(deadline_tv_sec, deadline_tv_ns);

          m->path_start = this->path_start_time_;
          m->path_deadline_duration = this->path_deadline_duration_;
        } else {
          // TODO validate path_info
          m->path_start = path_start_time_;
          m->path_deadline_duration = path_deadline_duration_;
        }
        m->topic_name = std::string(pub_->get_topic_name());
        path_info_pub_->publish(std::move(m));

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

    path_info_pub_ = this->create_publisher<path_info_msg::msg::PathInfo>(pathname_ + "_info", qos);
    auto path_info_callback =
        [this](path_info_msg::msg::PathInfo::UniquePtr msg) -> void
        {
          if(msg->topic_name != sub_->get_topic_name()) return;

          this->path_start_time_ = rclcpp::Time(msg->path_start, CLOCK_TYPE);
          this->path_deadline_duration_ = msg->path_deadline_duration;
          std::cout << "get path_info: "
                    << " topic: "  << msg->topic_name
                    << " path_start: " << this->path_start_time_.nanoseconds() << std::endl;
        };
    path_info_sub_ = this->create_subscription<path_info_msg::msg::PathInfo>(
        pathname_ + "_info", qos, path_info_callback);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  // TODO: have multiple path
  std::string pathname_;
  rclcpp::Publisher<path_info_msg::msg::PathInfo>::SharedPtr path_info_pub_;
  rclcpp::Subscription<path_info_msg::msg::PathInfo>::SharedPtr path_info_sub_;

  // TODO: have multiple times
  rcl_clock_type_t CLOCK_TYPE;
  rclcpp::Time path_start_time_;
  rclcpp::Duration path_deadline_duration_;

  bool exceeds_deadline(const std::string& path) const
  {
    (void)path;
    std::cout << std::fixed
              << "now: " << now_ns()
              << " path_start: " << path_start_time_.nanoseconds()
              << " deadline: " << std::to_string(path_deadline_duration_.nanoseconds()) << std::endl;

    // TODO: verify path_info.valid_ns
    if(path_start_time_ + path_deadline_duration_ <= now()) {
      std::cout << "path overrun!" << std::endl;
      return true;
    }

    return false;
  }

  rclcpp::Time now() const
  {
    rclcpp::Clock ros_clock(CLOCK_TYPE);
    return ros_clock.now();
  }

  double now_ns() const
  {
    return now().nanoseconds();
  }
};

}  // namespace pathnode_sample

RCLCPP_COMPONENTS_REGISTER_NODE(pathnode_sample::RelayWithPath)
