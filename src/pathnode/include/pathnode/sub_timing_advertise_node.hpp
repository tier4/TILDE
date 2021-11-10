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
#include "path_info_msg/msg/pub_info.hpp"
#include "timing_advertise_publisher.hpp"

namespace pathnode
{
template<class> inline constexpr bool always_false_v = false;

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
    typename MessageDeleter = std::default_delete<MessageT>,
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

    auto topic_info_name = resolved_topic_name + "/info/sub";

    auto topic_info_pub = create_publisher<path_info_msg::msg::TopicInfo>(
        topic_info_name,
        rclcpp::QoS(1));
    topic_info_pubs_[topic_info_name] = topic_info_pub;
    seqs_[topic_info_name] = 0;

    auto main_topic_callback
        = [this, resolved_topic_name, topic_info_name, callback](CallbackArgT msg,
                                                                 const rclcpp::MessageInfo &info) -> void
          {
            // publish subscription timing
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

            // prepare InputInfo
            using ConstRef = const MessageT &;
            using UniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
            using SharedConstPtr = std::shared_ptr<const MessageT>;
            using ConstRefSharedConstPtr = const std::shared_ptr<const MessageT> &;
            using SharedPtr = std::shared_ptr<MessageT>;

            rclcpp::Time header_stamp;
            rclcpp::Time t = this->now();

            using S = std::decay_t<decltype(msg)>;
            if constexpr (std::is_same_v<S, ConstRef>) {
              // std::cout << "visit: ConstRef\n";
              header_stamp = Process<MessageT>::get_timestamp3(t, &msg);
            }
            else if constexpr (std::is_same_v<S, UniquePtr>) {
              // std::cout << "visit: UniquePtr\n";
              header_stamp = Process<MessageT>::get_timestamp3(t, msg.get());
            }
            else if constexpr (std::is_same_v<S, SharedConstPtr>) {
              // std::cout << "visit: SharedConstPtr\n";
              header_stamp = Process<MessageT>::get_timestamp3(t, msg.get());
            }
            else if constexpr (std::is_same_v<S, ConstRefSharedConstPtr>) {
              // std::cout << "visit: ConstRefSharedPtr\n";
              header_stamp = Process<MessageT>::get_timestamp3(t, msg.get());
            }
            else if constexpr (std::is_same_v<S, SharedPtr>) {
              // std::cout << "visit: SharedPtr\n";
              header_stamp = Process<MessageT>::get_timestamp3(t, msg.get());
            }
            else {
              static_assert(always_false_v<S>, "non-exhaustive visitor!");
            }

            auto input_info = std::make_shared<InputInfo>();

            input_info->sub_time = now();
            if(header_stamp != t) {
              input_info->has_header_stamp = true;
              input_info->header_stamp = header_stamp;
            }

            // TODO: consider race condition in multi threaded executor.
            // i.e. subA comes when subB callback which uses topicA is running
            for(auto &[topic, tap]: timing_advertise_pubs_) {
              tap->set_input_info(resolved_topic_name, input_info);
            }

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
    auto info_topic = std::string(pub->get_topic_name()) + "/info/pub";
    auto info_pub = create_publisher<TimingAdvertisePublisherBase::InfoMsg>(info_topic, rclcpp::QoS(1), options);

    auto ta_pub = std::make_shared<TimingAdvertisePublisherT>(info_pub, pub, get_fully_qualified_name());
    timing_advertise_pubs_[info_topic] = ta_pub;
    return ta_pub;
  }

private:
  /// topic info name vs TopicInfoPublisher (subscriber side)
  std::map<std::string, TopicInfoPublisher> topic_info_pubs_;
  /// topic info name vs TimingAdvertisePublisher (pub side)
  std::map<std::string, std::shared_ptr<TimingAdvertisePublisherBase>> timing_advertise_pubs_;
  /// topic info name vs seq
  std::map<std::string, int64_t> seqs_;
};

} // namespace pathnode

#endif // SUB_TIMING_ADVERTISE_NODE_HPP_
