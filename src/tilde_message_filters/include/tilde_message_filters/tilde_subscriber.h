#ifndef TILDE_MESSAGE_FILTERS__TILDE_SUBSCRIBER_HPP_
#define TILDE_MESSAGE_FILTERS__TILDE_SUBSCRIBER_HPP_

#include <iostream>

#include "message_filters/subscriber.h"
#include "message_filters/connection.h"

#include "tilde/tilde_node.hpp"


namespace tilde_message_filters
{
template<class M, class NodeType = tilde::TildeNode>
class TildeSubscriber
{
public:
  typedef std::shared_ptr<NodeType> NodePtr;

  TildeSubscriber(NodePtr node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscriber_.subscribe(node, topic, qos);
  }

  TildeSubscriber(NodeType * node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscriber_.subscribe(node, topic, qos);
  }

  TildeSubscriber() = default;

  void subscribe(NodePtr node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscriber_.subscribe(node, topic, qos);
  }

  void subscribe(NodeType * node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscriber_.subscribe(node, topic, qos);
  }

  void subscribe(
    NodePtr node,
    const std::string& topic,
    const rmw_qos_profile_t qos,
    rclcpp::SubscriptionOptions options)
  {
    subscriber_.subscribe(node, topic, qos, options);
  }

  void subscribe(
    NodeType * node,
    const std::string& topic,
    const rmw_qos_profile_t qos,
    rclcpp::SubscriptionOptions options)
  {
    subscriber_.subscribe(node, topic, qos, options);
  }

  void subscribe()
  {
    subscriber_.subscribe();
  }

  void unsubscribe()
  {
    subscriber_.unsubscribe();
  }

  std::string getTopic() const
  {
    return subscriber_->getTopic();
  }

  template<typename C,
           typename CallbackArgT =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>
           >
  message_filters::Connection registerCallback(const C& callback)
  {
    std::cout << "ptn1" << std::endl;
    auto new_callback =
        [this, callback](CallbackArgT msg) -> void
        {
          std::cout << "hooked1!" << std::endl;
          callback(msg);
        };
    return subscriber_.registerCallback(new_callback);
  }

  template<typename P>
  message_filters::Connection registerCallback(const std::function<void(P)>& callback)
  {
    std::cout << "ptn2" << std::endl;
    auto new_callback =
        [this, callback](P msg) -> void
        {
          std::cout << "hooked2!" << std::endl;
          callback(msg);
        };
    return subscriber_.registerCallback(new_callback);
  }

  template<typename P>
  message_filters::Connection registerCallback(void(*callback)(P))
  {
    std::cout << "ptn3" << std::endl;
    auto new_callback =
        [this, callback](P msg) -> void
        {
          std::cout << "hooked3!" << std::endl;
          callback(msg);
        };
    return subscriber_.registerCallback(new_callback);
  }

  template<typename T, typename P>
  message_filters::Connection registerCallback(void(T::*callback)(P), T* t)
  {
    std::cout << "ptn4" << std::endl;
    auto f = std::bind(callback, t, std::placeholders::_1);
    auto new_callback =
        [this, f](P msg) -> void
        {
          std::cout << "hooked4!" << std::endl;
          f(msg);
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
  message_filters::Subscriber<M> subscriber_;
};
}  // namespace tilde_message_filters

#endif // TILDE_MESSAGE_FILTERS__TILDE_SUBSCRIBER_HPP_
