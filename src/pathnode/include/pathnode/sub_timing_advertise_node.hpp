#ifndef SUB_TIMING_ADVERTISE_NODE_HPP_
#define SUB_TIMING_ADVERTISE_NODE_HPP_

#include <memory>
#include <set>

#include "rclcpp/node.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/macros.hpp"

#include "rmw/types.h"

#include "path_info_msg/msg/topic_info.hpp"
#include "timing_advertise_publisher.hpp"

namespace pathnode
{
/// PoC of `every sub talks sub timing`
class SubTimingAdvertiseNode : public rclcpp::Node
{
  using TopicInfoPublisher = rclcpp::Publisher<path_info_msg::msg::TopicInfo>::SharedPtr;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(SubTimingAdvertiseNode)


  RCLCPP_PUBLIC
  explicit SubTimingAdvertiseNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_PUBLIC
  explicit SubTimingAdvertiseNode(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_PUBLIC
  virtual ~SubTimingAdvertiseNode();

  /// create custom subscription
  /**
   * This is the implementation of `first node only send path_info` strategy.
   */
  template<
    typename MessageT,
    typename CallbackT,
    typename AllocatorT = std::allocator<void>,
    typename CallbackMessageT =
    typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
    typename CallbackArgT =
    typename rclcpp::function_traits::function_traits<CallbackT>::template argument_type<0>,
    typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>,
    typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
      CallbackMessageT,
      AllocatorT
    >
  >
  std::shared_ptr<SubscriptionT>
  create_timing_advertise_subscription(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    CallbackT && callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
    rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(),
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = (
      MessageMemoryStrategyT::create_default()
    ))
  {
    using rclcpp::node_interfaces::get_node_topics_interface;
    auto node_topics_interface = get_node_topics_interface(this);
    auto resolved_topic_name = node_topics_interface->resolve_topic_name(topic_name);

    auto topic_info_name = resolved_topic_name + "_info";

    auto topic_info_pub = create_publisher<path_info_msg::msg::TopicInfo>(
        topic_info_name,
        rclcpp::QoS(1));
    topic_info_pubs_[topic_info_name] = topic_info_pub;
    seqs_[topic_info_name] = 0;

    auto main_topic_callback
        = [this, resolved_topic_name, topic_info_name, callback](CallbackArgT msg,
                                                                 const rclcpp::MessageInfo &info) -> void
          {
            auto minfo = info.get_rmw_message_info();

            auto m = std::make_unique<path_info_msg::msg::TopicInfo>();
            auto &seq = seqs_[topic_info_name];
            m->seq = seq;
            seq++;
            m->node_fqn = get_fully_qualified_name();
            m->topic_name = resolved_topic_name;
            for(size_t i=0; i<RMW_GID_STORAGE_SIZE; i++) {
              m->publisher_gid[i] = minfo.publisher_gid.data[i];
            }
            m->callback_start = now();
            topic_info_pubs_[topic_info_name]->publish(std::move(m));

            // finally, call original function
            callback(std::forward<CallbackArgT>(msg));
          };

    return create_subscription<MessageT>(
        topic_name,
        qos,
        main_topic_callback,
        options,
        msg_mem_strat);
  }

  template<
    typename MessageT,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>,
    typename TimingAdvertisePublisherT = TimingAdvertisePublisher<MessageT, AllocatorT>>
  std::shared_ptr<TimingAdvertisePublisherT>
  create_timing_advertise_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options =
    rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
  )
  {
    auto pub = create_publisher<MessageT, AllocatorT, PublisherT>(topic_name, qos, options);
    auto info_topic = std::string(pub->get_topic_name()) + "_info_pub";
    auto info_pub = create_publisher<path_info_msg::msg::TopicInfo>(info_topic, 1);
    return std::make_shared<TimingAdvertisePublisher<MessageT, AllocatorT>>(info_pub, pub, get_fully_qualified_name());
  }


 private:
  /// topic info name vs TopicInfoPublisher
  std::map<std::string, TopicInfoPublisher> topic_info_pubs_;
  /// topic info name vs seq
  std::map<std::string, int64_t> seqs_;
};

} // namespace pathnode

#endif // SUB_TIMING_ADVERTISE_NODE_HPP_
