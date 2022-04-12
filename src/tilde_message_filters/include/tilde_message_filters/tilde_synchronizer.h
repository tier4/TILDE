#ifndef TILDE_MESSAGE_FILTERS__TILDE_SYNCHRONIZER_H
#define TILDE_MESSAGE_FILTERS__TILDE_SYNCHRONIZER_H

#include <memory>

#include "tilde/tilde_node.hpp"

#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"

namespace tilde_message_filters
{

template<class F, class M>
constexpr bool true_v = true;

template<class F, class M>
constexpr bool false_v = false;

template<class F, class M>
constexpr bool is_subscriber() {
  if constexpr (std::is_same_v<F, message_filters::Subscriber<M>>) {
      return true_v<F, M>;
  }
  return false_v<F, M>;
}


template<typename Policy>
class TildeSynchronizer
{
  using Sync = message_filters::Synchronizer<Policy>;

  typedef typename Policy::Messages Messages;
  typedef typename Policy::Events Events;
  typedef typename Policy::Signal Signal;

  typedef typename std::tuple_element<0, Messages>::type M0;
  typedef typename std::tuple_element<1, Messages>::type M1;
  typedef typename std::tuple_element<2, Messages>::type M2;
  typedef typename std::tuple_element<3, Messages>::type M3;
  typedef typename std::tuple_element<4, Messages>::type M4;
  typedef typename std::tuple_element<5, Messages>::type M5;
  typedef typename std::tuple_element<6, Messages>::type M6;
  typedef typename std::tuple_element<7, Messages>::type M7;
  typedef typename std::tuple_element<8, Messages>::type M8;

public:
  template<class F0, class F1>
  TildeSynchronizer(tilde::TildeNode *node, F0& f0, F1& f1)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(f0, f1);

    if constexpr (std::is_same_v<F0, message_filters::Subscriber<M0>>) {
       using rclcpp::node_interfaces::get_node_topics_interface;
       auto node_topics_interface = get_node_topics_interface(node);
       auto topic_name = f0.getTopic();
       auto resolved_topic_name = node_topics_interface->resolve_topic_name(topic_name);
       std::cout << "topic_name[0]: " << resolved_topic_name << std::endl;
       topic_names_[0] = resolved_topic_name;
    }
  }

  template<class F0, class F1>
  TildeSynchronizer(tilde::TildeNode *node, const Policy& policy, F0& f0, F1& f1)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(policy, f0, f1);

    if constexpr (std::is_same_v<F0, message_filters::Subscriber<M0>>) {
       using rclcpp::node_interfaces::get_node_topics_interface;
       auto node_topics_interface = get_node_topics_interface(node);
       auto topic_name = f0.getTopic();
       auto resolved_topic_name = node_topics_interface->resolve_topic_name(topic_name);
       std::cout << "topic_name[0]: " << resolved_topic_name << std::endl;
       topic_names_[0] = resolved_topic_name;
    }
  }

  template<class F0, class F1, class F2>
  TildeSynchronizer(tilde::TildeNode *node, F0& f0, F1& f1, F2& f2)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(f0, f1, f2);
  }

  template<class F0, class F1, class F2>
  TildeSynchronizer(tilde::TildeNode *node, const Policy& policy, F0& f0, F1& f1, F2& f2)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(policy, f0, f1, f2);
  }

  template<class C,
           std::size_t Arity = 2,
           typename std::enable_if<
             rclcpp::function_traits::arity_comparator<Arity, C>::value
             >::type * = nullptr,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>
           >
  message_filters::Connection registerCallback(const C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1) {
            std::cout << "hooked2" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);

            callback(msg0, msg1);
          };

    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2));
  }

  template<class C,
           std::size_t Arity = 3,
           typename std::enable_if<
             rclcpp::function_traits::arity_comparator<Arity, C>::value
             >::type * = nullptr,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>
           >
  message_filters::Connection registerCallback(const C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2) {
            std::cout << "hooked3" << std::endl;
            callback(msg0, msg1, msg2);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3));
  }

  /*
  template<class C,
           std::size_t Arity = 4,
           typename std::enable_if<
             rclcpp::function_traits::arity_comparator<Arity, C>::value
             >::type * = nullptr,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>
           >
  message_filters::Connection registerCallback(const C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT2 msg3) {
            std::cout << "hooked4" << std::endl;
            callback(msg0, msg1, msg2, msg3);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4));
  }
  */

  template<std::size_t I,
           class CallbackArgT>
  void register_ith_message_as_input(
      CallbackArgT msg)
  {
    static_assert(I < 9);

    auto subtime_steady = node_->get_steady_time();
    const auto& topic = topic_names_[I];
    if(topic.empty()) return;

    node_->register_message_as_input(
        msg.get(),
        topic,
        subtime_steady);
  }

private:
  std::shared_ptr<Sync> sync_ptr_;
  tilde::TildeNode *node_;
  // message id vs topic name.
  // 9 means the number of max messages.
  std::vector<std::string> topic_names_{9};
};



}  // tilde_message_filters


#endif // TILDE_MESSAGE_FILTERS__TILDE_SYNCHRONIZER_H
