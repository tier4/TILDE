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

#ifndef TILDE__TILDE_NODE_HPP_
#define TILDE__TILDE_NODE_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "rclcpp/node.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/macros.hpp"

#include "rmw/types.h"

#include "tilde_msg/msg/topic_info.hpp"
#include "tilde_msg/msg/pub_info.hpp"
#include "tilde_publisher.hpp"

#include "tilde/tp.h"

namespace tilde
{
template<class>
inline constexpr bool always_false_v = false;

/// PoC of `every sub talks sub timing`
class TildeNode : public rclcpp::Node
{
  using TopicInfoPublisher = rclcpp::Publisher<tilde_msg::msg::TopicInfo>::SharedPtr;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(TildeNode)


  RCLCPP_PUBLIC
  explicit TildeNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_PUBLIC
  explicit TildeNode(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_PUBLIC
  virtual ~TildeNode();

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
  create_tilde_subscription(
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

    std::cerr << &callback << std::endl;
    auto topic_info_pub = create_publisher<tilde_msg::msg::TopicInfo>(
      topic_info_name,
      rclcpp::QoS(1));
    topic_info_pubs_[topic_info_name] = topic_info_pub;
    seqs_[topic_info_name] = 0;

    auto callback_addr = &callback;

    tracepoint(
      TRACEPOINT_PROVIDER,
      tilde_subscription_init,
      callback_addr,
      get_fully_qualified_name(),
      resolved_topic_name.c_str());

    auto main_topic_callback =
      [this, resolved_topic_name, topic_info_name, callback, callback_addr](
      CallbackArgT msg,
      const rclcpp::MessageInfo & info) -> void
      {
        if (this->enable_tilde) {
          auto subtime = this->now();
          auto subtime_steady = this->steady_clock_->now();
          // publish subscription timing
          auto minfo = info.get_rmw_message_info();

          tracepoint(
            TRACEPOINT_PROVIDER,
            tilde_subscribe,
            callback_addr,
            subtime_steady.nanoseconds());

          auto m = std::make_unique<tilde_msg::msg::TopicInfo>();
          auto & seq = seqs_[topic_info_name];
          m->seq = seq;
          seq++;
          m->node_fqn = get_fully_qualified_name();
          m->topic_name = resolved_topic_name;
          for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; i++) {
            m->publisher_gid[i] = minfo.publisher_gid.data[i];
          }
          m->callback_start = now();
          topic_info_pubs_[topic_info_name]->publish(std::move(m));

          // prepare InputInfo
          using ConstRef = const MessageT &;
          using UniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
          using SharedConstPtr = std::shared_ptr<const MessageT>;
          using ConstRefSharedConstPtr = const std::shared_ptr<const MessageT>&;
          using SharedPtr = std::shared_ptr<MessageT>;

          rclcpp::Time header_stamp;
          rclcpp::Time t(0, 100, this->now().get_clock_type());

          using S = std::decay_t<decltype(msg)>;
          if constexpr (std::is_same_v<S, ConstRef>) {
            // std::cout << "visit: ConstRef\n";
            header_stamp = Process<MessageT>::get_timestamp3(t, &msg);
          } else if constexpr (std::is_same_v<S, UniquePtr>) {
            // std::cout << "visit: UniquePtr\n";
            header_stamp = Process<MessageT>::get_timestamp3(t, msg.get());
          } else if constexpr (std::is_same_v<S, SharedConstPtr>) {
            // std::cout << "visit: SharedConstPtr\n";
            header_stamp = Process<MessageT>::get_timestamp3(t, msg.get());
          } else if constexpr (std::is_same_v<S, ConstRefSharedConstPtr>) {
            // std::cout << "visit: ConstRefSharedPtr\n";
            header_stamp = Process<MessageT>::get_timestamp3(t, msg.get());
          } else if constexpr (std::is_same_v<S, SharedPtr>) {
            // std::cout << "visit: SharedPtr\n";
            header_stamp = Process<MessageT>::get_timestamp3(t, msg.get());
          } else {
            static_assert(always_false_v<S>, "non-exhaustive visitor!");
          }

          auto input_info = std::make_shared<InputInfo>();

          input_info->sub_time = subtime;
          input_info->sub_time_steady = subtime_steady;
          if (header_stamp != t) {
            input_info->has_header_stamp = true;
            input_info->header_stamp = header_stamp;
          }

          // TODO(y-okumura-isp): consider race condition in multi threaded executor.
          // i.e. subA comes when subB callback which uses topicA is running
          for (auto &[topic, tap] : timing_advertise_pubs_) {
            tap->set_input_info(resolved_topic_name, input_info);
            if (input_info->has_header_stamp) {
              tap->set_explicit_subtime(resolved_topic_name, input_info);
            }
          }
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
    typename TildePublisherT = TildePublisher<MessageT, AllocatorT>>
  std::shared_ptr<TildePublisherT>
  create_tilde_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options =
    rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
  )
  {
    auto pub = create_publisher<MessageT, AllocatorT, PublisherT>(topic_name, qos, options);
    auto info_topic = std::string(pub->get_topic_name()) + "/info/pub";
    auto info_pub = create_publisher<TildePublisherBase::InfoMsg>(
      info_topic, rclcpp::QoS(1), options);

    auto ta_pub = std::make_shared<TildePublisherT>(
      info_pub, pub, get_fully_qualified_name(),
      this->get_clock(),
      steady_clock_,
      this->enable_tilde);
    timing_advertise_pubs_[info_topic] = ta_pub;


    tracepoint(
      TRACEPOINT_PROVIDER,
      tilde_publisher_init,
      ta_pub.get(),
      get_fully_qualified_name(),
      pub->get_topic_name());

    return ta_pub;
  }

private:
  /// topic info name vs TopicInfoPublisher (subscriber side)
  std::map<std::string, TopicInfoPublisher> topic_info_pubs_;
  /// topic info name vs TildePublisher (pub side)
  std::map<std::string, std::shared_ptr<TildePublisherBase>> timing_advertise_pubs_;
  /// topic info name vs seq
  std::map<std::string, int64_t> seqs_;
  /// node clock may be simulation time
  std::shared_ptr<rclcpp::Clock> steady_clock_;

  /// whether to enable tilde
  // TODO(y-okumura-isp) enable dynamic configuration
  bool enable_tilde;
};

}  // namespace tilde

#endif  // TILDE__TILDE_NODE_HPP_
