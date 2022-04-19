#ifndef TILDE_MESSAGE_FILTERS__TILDE_SUBSCRIBER_HPP_
#define TILDE_MESSAGE_FILTERS__TILDE_SUBSCRIBER_HPP_

#include <iostream>

#include "message_filters/subscriber.h"
#include "message_filters/connection.h"

#include "tilde/tilde_node.hpp"


namespace tilde_message_filters
{
template<class>
inline constexpr bool always_false_v = false;

template<class M, class NodeType = tilde::TildeNode>
class TildeSubscriber
{
  using NodePtr = std::shared_ptr<NodeType>;
  using MConstPtr = std::shared_ptr<M const>;

public:

  TildeSubscriber(NodePtr node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscribe(node, topic, qos);
  }

  TildeSubscriber(NodeType * node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscribe(node, topic, qos);
  }

  TildeSubscriber() = default;

  void subscribe(NodePtr node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscribe(node.get(), topic, qos);
    node_raw_ = nullptr;
    node_shared_ = node;
  }

  void subscribe(NodeType * node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    node_raw_ = node;
    subscriber_.subscribe(node, topic, qos);
    subscriber_.registerCallback(&TildeSubscriber::register_message_callback, this);
  }

  void subscribe()
  {
    subscriber_.subscribe();
  }

  void unsubscribe()
  {
    subscriber_.unsubscribe();
  }

  /// get non-FQDN topic name as in message_filter::Subscriber,
  std::string getTopic() const
  {
    return subscriber_.getTopic();
  }

  // original callbacks
  template<typename C>
  message_filters::Connection registerCallback(const C& callback)
  {
    return subscriber_.registerCallback(callback);
  }

  template<typename P>
  message_filters::Connection registerCallback(const std::function<void(P)>& callback)
  {
    return subscriber_.registerCallback(callback);
  }

  template<typename P>
  message_filters::Connection registerCallback(void(*callback)(P))
  {
    return subscriber_.registerCallback(callback);
  }

  template<typename T, typename P>
  message_filters::Connection registerCallback(void(T::*callback)(P), T* t)
  {
    return subscriber_.registerCallback(callback, t);
  }

  template<typename C,
           typename CallbackArgT =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>
           >
  message_filters::Connection registerCallbackTilde(const C& callback)
  {
    std::cout << "ptn1" << std::endl;
    const auto topic = getTopicFQDN();

    auto new_callback =
        [this, callback, topic](CallbackArgT msg) -> void
        {
          std::cout << "hooked1!" << std::endl;
          auto pnode = get_node();

          // msg.get() in below codes:
          // we can use msg.get() because
          // - message_filters::simple_filter assumes that
          //   typename C sholud be compatible with std::function<void(const MConstPtr&)>
          // - MConstPtr is std::shared_ptr<M const>

          // on TildeSubscriber, default callback saves subtime
          rclcpp::Time subtime, subtime_steady;
          pnode->find_subtime(
              msg.get(), topic,
              subtime, subtime_steady);

          // update implicit input info
          pnode->register_message_as_input(
              msg.get(), topic,
              subtime, subtime_steady);

          callback(msg);
        };
    return subscriber_.registerCallback(new_callback);
  }

  template<typename P,
           typename MessageDeleter = std::default_delete<M>>
  message_filters::Connection registerCallbackTilde(const std::function<void(P)>& callback)
  {
    std::cout << "ptn2" << std::endl;
    const auto topic = getTopicFQDN();
    // As std::function use vtable, we use auto type,
    // not std::function<void(P)> new_callback = ...
    // https://stackoverflow.com/questions/25848690/should-i-use-stdfunction-or-a-function-pointer-in-c
    auto new_callback =
        [this, callback, topic](P msg) -> void
        {
          std::cout << "hooked2!" << std::endl;

          auto pnode = get_node();

          // we use msg.get() because
          // even in this pattern,
          // P looks to be const shared_ptr (see message_filters::MessageEvent).
          // TODO(y-okumura-isp): Is ConstRef also possible??

          // on TildeSubscriber, default callback saves subtime
          rclcpp::Time subtime, subtime_steady;
          pnode->find_subtime(
              msg.get(), topic,
              subtime, subtime_steady);

          // update implicit input info
          pnode->register_message_as_input(
              msg.get(), topic,
              subtime, subtime_steady);

          callback(msg);
        };
    return subscriber_.registerCallback(new_callback);
  }

  template<typename P>
  message_filters::Connection registerCallbackTilde(void(*callback)(P))
  {
    std::cout << "ptn3" << std::endl;
    const auto topic = getTopicFQDN();
    // We use lambda not original function pointer
    // because we cannot convert captured lambda to function pointer.
    // https://stackoverflow.com/questions/28746744/passing-capturing-lambda-as-function-pointer
    auto new_callback =
        [this, callback, topic](P msg) -> void
        {
          std::cout << "hooked3!" << std::endl;
         auto pnode = get_node();

          // we use msg.get() because
          // even in this pattern,
          // P looks to be const shared_ptr (see message_filters::MessageEvent).
          // TODO(y-okumura-isp): Is ConstRef also possible??

          // on TildeSubscriber, default callback saves subtime
          rclcpp::Time subtime, subtime_steady;
          pnode->find_subtime(
              msg.get(), topic,
              subtime, subtime_steady);

          // update implicit input info
          pnode->register_message_as_input(
              msg.get(), topic,
              subtime, subtime_steady);

          callback(msg);
        };
    return subscriber_.registerCallback(new_callback);
  }

  template<typename T, typename P>
  message_filters::Connection registerCallbackTilde(void(T::*callback)(P), T* t)
  {
    std::cout << "ptn4" << std::endl;
    const auto topic = getTopicFQDN();
    auto bind_callback = std::bind(callback, t, std::placeholders::_1);
    auto new_callback =
        [this, bind_callback, topic](P msg) -> void
        {
          std::cout << "hooked4!" << std::endl;
          auto pnode = get_node();

          // we use msg.get() because
          // even in this pattern,
          // P looks to be const shared_ptr (see message_filters::MessageEvent).
          // TODO(y-okumura-isp): Can the msg type be ConstRef?

          // on TildeSubscriber, default callback saves subtime
          rclcpp::Time subtime, subtime_steady;
          pnode->find_subtime(
              msg.get(), topic,
              subtime, subtime_steady);

          // update implicit input info
          pnode->register_message_as_input(
              msg.get(), topic,
              subtime, subtime_steady);

          bind_callback(msg);
        };
    // We need to wrap new_callback bacause P is sometimes non MConstPtr.
    // If P is non MConstPtr, `registerCallback(const C& callback)` is called and
    // subsequent `signal_.addCallback(Callback(callback))` fails because
    // Callback = std::function<void(const MConstPtr&)>.
    // So we wrap new_callback by std::function, and use
    // `registerCallback(const std::function<void(P)>& callback)`.
    std::function<void(P)> new_callback2 =
        std::bind(new_callback, std::placeholders::_1);
    return subscriber_.registerCallback(new_callback2);
  }

private:
  NodePtr node_shared_;
  NodeType * node_raw_ {nullptr};
  message_filters::Subscriber<M> subscriber_;

  NodeType * get_node()
  {
    if(node_shared_) {
      return node_shared_.get();
    } else {
      return node_raw_;
    }
  }

  /// get topic FQDN
  std::string getTopicFQDN()
  {
    auto topic = subscriber_.getTopic();
    using rclcpp::node_interfaces::get_node_topics_interface;
    auto node_topics_interface = get_node_topics_interface(get_node());
    auto resolved_topic_name = node_topics_interface->resolve_topic_name(topic);
    return resolved_topic_name;
  }

  /// TildeSubsriber 1st callback
  /**
   *  Save subtime for other callbacks or TildeSynchronizer
   */
  void register_message_callback(const MConstPtr msg)
  {
    auto topic = getTopicFQDN();
    auto pnode = get_node();
    auto subtime = pnode->now();
    auto subtime_steady = pnode->get_steady_time();

    pnode->register_message_as_input(
        msg.get(), topic,
        subtime, subtime_steady);
  }
};
}  // namespace tilde_message_filters

#endif // TILDE_MESSAGE_FILTERS__TILDE_SUBSCRIBER_HPP_