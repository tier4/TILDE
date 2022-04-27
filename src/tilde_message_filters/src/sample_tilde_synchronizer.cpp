// Copyright 2022 Research Institute of Systems Planning, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "tilde/tilde_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "message_filters/sync_policies/exact_time.h"

#include "tilde_message_filters/tilde_subscriber.h"
#include "tilde_message_filters/tilde_synchronizer.h"

typedef sensor_msgs::msg::PointCloud2 Msg;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2 const> MsgConstPtr;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2> MsgPtr;

namespace sample_tilde_message_filter
{
// 2 means the callback argc to simplify the template programming
class SampleTildeSynchronizer2 : public tilde::TildeNode
{
  using SyncPolicy = message_filters::sync_policies::ExactTime<Msg, Msg>;
  using Sync = tilde_message_filters::TildeSynchronizer<SyncPolicy>;
  using Subscriber = message_filters::Subscriber<Msg>;
  using Publisher = tilde::TildePublisher<Msg>::SharedPtr;

public:
  explicit SampleTildeSynchronizer2(const rclcpp::NodeOptions & options)
  : TildeNode("sample_tilde_sync2", options)
  {
    std::cout << "hee" << std::endl;

    rclcpp::QoS qos(rclcpp::KeepLast(7));
    auto rmw_qos = qos.get_rmw_qos_profile();

    sub_pc1_.subscribe(this, "in1", rmw_qos);
    sub_pc2_.subscribe(this, "in2", rmw_qos);

    sync_ptr_ = std::make_shared<Sync>(
        this,
        SyncPolicy(5),
        sub_pc1_,
        sub_pc2_);

    pub_ = create_tilde_publisher<Msg>("out2", 1);

    // registerCallback(const C& callback) version:
    // <- (const C&) can bind rvalue
    sync_ptr_->registerCallback(
        std::bind(&SampleTildeSynchronizer2::sub_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    std::cout << "hoo" << std::endl;
  }

private:
  Subscriber sub_pc1_, sub_pc2_;
  std::shared_ptr<Sync> sync_ptr_;
  Publisher pub_;

  void sub_callback(const MsgConstPtr &msg1,
                    const MsgConstPtr &msg2)
  {
    (void) msg2;
    std::cout << "sub_callback" << std::endl;
    pub_->publish(*msg1);
  }
};

// 3 means the callback argc to simplify the template programming
class SampleTildeSynchronizer3 : public tilde::TildeNode
{
  using SyncPolicy = message_filters::sync_policies::ExactTime<Msg, Msg, Msg>;
  using Sync = tilde_message_filters::TildeSynchronizer<SyncPolicy>;
  using Subscriber = message_filters::Subscriber<Msg>;

public:
  explicit SampleTildeSynchronizer3(const rclcpp::NodeOptions & options)
  : TildeNode("sample_tilde_sync3", options)
  {
    std::cout << "hee" << std::endl;

    rclcpp::QoS qos(rclcpp::KeepLast(7));
    auto rmw_qos = qos.get_rmw_qos_profile();

    sub_pc1_.subscribe(this, "in1", rmw_qos);
    sub_pc2_.subscribe(this, "in2", rmw_qos);
    sub_pc3_.subscribe(this, "in3", rmw_qos);

    sync_ptr_ = std::make_shared<Sync>(
        this,
        SyncPolicy(5),
        sub_pc1_,
        sub_pc2_,
        sub_pc3_);

    // registerCallback(const C& callback) version:
    // <- (const C&) can bind rvalue
    sync_ptr_->registerCallback(
        std::bind(&SampleTildeSynchronizer3::sub_callback, this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3
                  ));
    std::cout << "hoo" << std::endl;
  }

private:
  Subscriber sub_pc1_, sub_pc2_, sub_pc3_;
  std::shared_ptr<Sync> sync_ptr_;

  void sub_callback(const MsgConstPtr &msg1,
                    const MsgConstPtr &msg2,
                    const MsgConstPtr &msg3)
  {
    (void) msg1;
    (void) msg2;
    (void) msg3;
    std::cout << "sub_callback" << std::endl;
  }
};

}  // sample_tilde_message_filter

RCLCPP_COMPONENTS_REGISTER_NODE(sample_tilde_message_filter::SampleTildeSynchronizer2)
RCLCPP_COMPONENTS_REGISTER_NODE(sample_tilde_message_filter::SampleTildeSynchronizer3)
