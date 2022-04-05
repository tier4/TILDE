#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "tilde/tilde_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "message_filters/sync_policies/exact_time.h"

#include "tilde_message_filters/tilde_subscriber.h"
#include "tilde_message_filters/tilde_time_synchronizer.h"

typedef sensor_msgs::msg::PointCloud2 Msg;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2 const> MsgConstPtr;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2> MsgPtr;

namespace sample_tilde_message_filter
{
class SampleSynchronizer : public tilde::TildeNode
{
  using SyncPolicy = message_filters::sync_policies::ExactTime<Msg, Msg>;
  using Sync = tilde_message_filters::TildeSynchronizer<SyncPolicy>;

public:
  explicit SampleSynchronizer(const rclcpp::NodeOptions & options)
  : TildeNode("sub_with_header", options)
  {
    std::cout << "hee" << std::endl;

    rclcpp::QoS qos(rclcpp::KeepLast(7));
    sub_pc1_.subscribe(this, "in1", qos.get_rmw_qos_profile());
    sub_pc2_.subscribe(this, "in2", qos.get_rmw_qos_profile());

    sync_ptr_ = std::make_shared<Sync>(SyncPolicy(5), sub_pc1_, sub_pc2_);

    // registerCallback(const C& callback) version:
    // <- (const C&) can bind rvalue
    sync_ptr_->registerCallback(
        std::bind(&SampleSynchronizer::sub_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    std::cout << "hoo" << std::endl;
  }

private:
  tilde_message_filters::TildeSubscriber<Msg> sub_pc1_, sub_pc2_;
  MsgPtr msg_;

  std::shared_ptr<Sync> sync_ptr_;

  void sub_callback(const MsgConstPtr &msg1,
                    const MsgConstPtr &msg2)
  {
    (void) msg1;
    (void) msg2;
    std::cout << "sub_callback" << std::endl;
  }

};
}  // sample_tilde_message_filter

RCLCPP_COMPONENTS_REGISTER_NODE(sample_tilde_message_filter::SampleSynchronizer)
