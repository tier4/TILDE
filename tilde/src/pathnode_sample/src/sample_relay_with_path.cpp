// Copyright 2021 Research Institute of Systems Planning, Inc.
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

// TODO(y-okumura-isp): define per path
const char IS_FIRST[] = "is_first";
const char PATH_VALID_MIN_SEC[] = "path_valid_min_sec";
const char PATH_VALID_MIN_NS[] = "path_valid_min_ns";
const char PATH_VALID_MAX_SEC[] = "path_valid_max_sec";
const char PATH_VALID_MAX_NS[] = "path_valid_max_ns";
const char WAIT_MSEC[] = "wait_msec";

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class RelayWithPath : public pathnode::PathNode
{
public:
  explicit RelayWithPath(const rclcpp::NodeOptions & options)
  : PathNode("relay", options), path_name_("sample_path")
  {
    /* newly introduced variables for path */
    declare_parameter<bool>(IS_FIRST, false);
    //  whole seconds (valid values are >= 0)
    declare_parameter<int64_t>(PATH_VALID_MIN_SEC, (int64_t) 0);
    // nanoseconds (valid values are [0, 999999999])
    declare_parameter<int64_t>(PATH_VALID_MIN_NS, (int64_t)( 1 * 1000 * 1000));
    //  whole seconds (valid values are >= 0)
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

    auto callback =
      [this, wait_msec](const std_msgs::msg::String & msg) -> void
      {
        this->on_pathed_subscription(this->path_name_);

        rclcpp::Time path_start_time(0, 0, CLOCK_TYPE);

        if (!pop_path_start_time(path_name_, path_start_time)) {
          std::cout << "cannot find path_start_time" << std::endl;
          // do what you want: error handling or continue routine without deadline detection
          return;
        }

        auto valid_min = get_path_valid_min(path_name_);
        auto valid_max = get_path_valid_max(path_name_);

        auto deadline_exceeds =
          [this, &path_start_time, &valid_min, &valid_max]() -> bool
          {
            auto nw = this->now();
            std::cout << "path_start_time: " << path_start_time.nanoseconds() << " " <<
              "valid_min: " << valid_min.nanoseconds() << " " <<
              "nw: " << nw.nanoseconds() << " " <<
              "valid_max: " << valid_max.nanoseconds() << " " <<
              "min: " << (path_start_time - valid_min).nanoseconds() << " " <<
              "max: " << (path_start_time + valid_max).nanoseconds() << std::endl;
            return !(path_start_time - valid_min < nw && nw < path_start_time + valid_max);
          };

        // you can call deadline_exceeds many times in your routine
        if (deadline_exceeds()) {
          std::cout << "deadline exceeds!" << std::endl;
          return;
        }

        // usual procedure
        std::chrono::duration<int64_t, std::milli> dur(wait_msec);
        std::this_thread::sleep_for(dur);
        std::cout << "relay: " << msg.data << std::endl;
        pub_->publish(msg);
      };


    // TODO(y-okumura-isp) check range
    auto valid_min_tv_sec = get_parameter(PATH_VALID_MIN_SEC).get_value<int64_t>();
    auto valid_min_tv_nsec = get_parameter(PATH_VALID_MIN_NS).get_value<int64_t>();
    auto valid_max_tv_sec = get_parameter(PATH_VALID_MAX_SEC).get_value<int64_t>();
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

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "in", qos, callback);
    setup_path(path_node_options);
  }

private:
  std::string path_name_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace pathnode_sample

RCLCPP_COMPONENTS_REGISTER_NODE(pathnode_sample::RelayWithPath)
