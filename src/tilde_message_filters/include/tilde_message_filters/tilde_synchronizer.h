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

#ifndef TILDE_MESSAGE_FILTERS__TILDE_SYNCHRONIZER_H
#define TILDE_MESSAGE_FILTERS__TILDE_SYNCHRONIZER_H

#include <memory>

#include "tilde/tilde_node.hpp"

#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"

#include "tilde_message_filters/tilde_subscriber.h"

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
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
  }

  template<class F0, class F1>
  TildeSynchronizer(tilde::TildeNode *node, const Policy& policy, F0& f0, F1& f1)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(policy, f0, f1);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
  }

  template<class F0, class F1, class F2>
  TildeSynchronizer(tilde::TildeNode *node, F0& f0, F1& f1, F2& f2)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(f0, f1, f2);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
  }

  template<class F0, class F1, class F2>
  TildeSynchronizer(tilde::TildeNode *node, const Policy& policy, F0& f0, F1& f1, F2& f2)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(policy, f0, f1, f2);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
  }

  template<class F0, class F1, class F2, class F3>
  TildeSynchronizer(tilde::TildeNode *node, F0& f0, F1& f1, F2& f2, F3& f3)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(f0, f1, f2, f3);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
    init_topic_name<3, M3>(f3);
  }

  template<class F0, class F1, class F2, class F3>
  TildeSynchronizer(tilde::TildeNode *node, const Policy& policy, F0& f0, F1& f1, F2& f2, F3& f3)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(policy, f0, f1, f2, f3);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
    init_topic_name<3, M3>(f3);
  }

  template<class F0, class F1, class F2, class F3, class F4>
  TildeSynchronizer(tilde::TildeNode *node, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(f0, f1, f2, f3, f4);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
    init_topic_name<3, M3>(f3);
    init_topic_name<4, M4>(f4);
  }

  template<class F0, class F1, class F2, class F3, class F4>
  TildeSynchronizer(tilde::TildeNode *node, const Policy& policy, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(policy, f0, f1, f2, f3, f4);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
    init_topic_name<3, M3>(f3);
    init_topic_name<4, M4>(f4);
  }

  template<class F0, class F1, class F2, class F3, class F4,
           class F5>
  TildeSynchronizer(tilde::TildeNode *node, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4,
                    F5& f5)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(f0, f1, f2, f3, f4, f5);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
    init_topic_name<3, M3>(f3);
    init_topic_name<4, M4>(f4);
    init_topic_name<5, M5>(f5);
  }

  template<class F0, class F1, class F2, class F3, class F4,
           class F5>
  TildeSynchronizer(tilde::TildeNode *node, const Policy& policy, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4,
                    F5& f5)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(policy, f0, f1, f2, f3, f4, f5);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
    init_topic_name<3, M3>(f3);
    init_topic_name<4, M4>(f4);
    init_topic_name<5, M5>(f5);
  }

  template<class F0, class F1, class F2, class F3, class F4,
           class F5, class F6>
  TildeSynchronizer(tilde::TildeNode *node, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4,
                    F5& f5, F6& f6)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(f0, f1, f2, f3, f4, f5, f6);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
    init_topic_name<3, M3>(f3);
    init_topic_name<4, M4>(f4);
    init_topic_name<5, M5>(f5);
    init_topic_name<6, M6>(f6);
  }

  template<class F0, class F1, class F2, class F3, class F4,
           class F5, class F6>
  TildeSynchronizer(tilde::TildeNode *node, const Policy& policy, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4,
                    F5& f5, F6& f6)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(policy, f0, f1, f2, f3, f4, f5, f6);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
    init_topic_name<3, M3>(f3);
    init_topic_name<4, M4>(f4);
    init_topic_name<5, M5>(f5);
    init_topic_name<6, M6>(f6);
  }

  template<class F0, class F1, class F2, class F3, class F4,
           class F5, class F6, class F7>
  TildeSynchronizer(tilde::TildeNode *node, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4,
                    F5& f5, F6& f6, F7& f7)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(f0, f1, f2, f3, f4, f5, f6, f7);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
    init_topic_name<3, M3>(f3);
    init_topic_name<4, M4>(f4);
    init_topic_name<5, M5>(f5);
    init_topic_name<6, M6>(f6);
    init_topic_name<7, M7>(f7);
  }

  template<class F0, class F1, class F2, class F3, class F4,
           class F5, class F6, class F7>
  TildeSynchronizer(tilde::TildeNode *node, const Policy& policy, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4,
                    F5& f5, F6& f6, F7& f7)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(policy, f0, f1, f2, f3, f4, f5, f6, f7);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
    init_topic_name<3, M3>(f3);
    init_topic_name<4, M4>(f4);
    init_topic_name<5, M5>(f5);
    init_topic_name<6, M6>(f6);
    init_topic_name<7, M7>(f7);
  }

  template<class F0, class F1, class F2, class F3, class F4,
           class F5, class F6, class F7, class F8>
  TildeSynchronizer(tilde::TildeNode *node, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4,
                    F5& f5, F6& f6, F7& f7, F8& f8)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(f0, f1, f2, f3, f4, f5, f6, f7, f8);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
    init_topic_name<3, M3>(f3);
    init_topic_name<4, M4>(f4);
    init_topic_name<5, M5>(f5);
    init_topic_name<6, M6>(f6);
    init_topic_name<7, M7>(f7);
    init_topic_name<8, M8>(f8);
  }

  template<class F0, class F1, class F2, class F3, class F4,
           class F5, class F6, class F7, class F8>
  TildeSynchronizer(tilde::TildeNode *node, const Policy& policy, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4,
                    F5& f5, F6& f6, F7& f7, F8& f8)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(policy, f0, f1, f2, f3, f4, f5, f6, f7, f8);
    init_topic_name<0, M0>(f0);
    init_topic_name<1, M1>(f1);
    init_topic_name<2, M2>(f2);
    init_topic_name<3, M3>(f3);
    init_topic_name<4, M4>(f4);
    init_topic_name<5, M5>(f5);
    init_topic_name<6, M6>(f6);
    init_topic_name<7, M7>(f7);
    init_topic_name<8, M8>(f8);
  }

  TildeSynchronizer(tilde::TildeNode *node)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>();
  }

  TildeSynchronizer(tilde::TildeNode *node, const Policy& policy)
      : node_(node)
  {
    sync_ptr_ = std::make_shared<Sync>(policy);
  }

  // (const C& callback)
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

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);

            callback(msg0, msg1, msg2);
          };

    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3));
  }

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
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
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
            CallbackArgT3 msg3) {
            std::cout << "hooked4" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);

            callback(msg0, msg1, msg2, msg3);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4));
  }

  template<class C,
           std::size_t Arity = 5,
           typename std::enable_if<
             rclcpp::function_traits::arity_comparator<Arity, C>::value
             >::type * = nullptr,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>
           >
  message_filters::Connection registerCallback(const C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4) {
            std::cout << "hooked5" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);

            callback(msg0, msg1, msg2, msg3, msg4);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5));
  }

  template<class C,
           std::size_t Arity = 6,
           typename std::enable_if<
             rclcpp::function_traits::arity_comparator<Arity, C>::value
             >::type * = nullptr,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>,
           typename CallbackArgT5 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<5>
           >
  message_filters::Connection registerCallback(const C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4,
            CallbackArgT5 msg5) {
            std::cout << "hooked6" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);
            register_ith_message_as_input<5>(msg5);

            callback(msg0, msg1, msg2, msg3, msg4, msg5);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5,
                  std::placeholders::_6));
  }

  template<class C,
           std::size_t Arity = 7,
           typename std::enable_if<
             rclcpp::function_traits::arity_comparator<Arity, C>::value
             >::type * = nullptr,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>,
           typename CallbackArgT5 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<5>,
           typename CallbackArgT6 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<6>
           >
  message_filters::Connection registerCallback(const C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4,
            CallbackArgT5 msg5,
            CallbackArgT6 msg6) {
            std::cout << "hooked7" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);
            register_ith_message_as_input<5>(msg5);
            register_ith_message_as_input<6>(msg6);

            callback(msg0, msg1, msg2, msg3, msg4, msg5, msg6);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5,
                  std::placeholders::_6,
                  std::placeholders::_7));
  }

  template<class C,
           std::size_t Arity = 8,
           typename std::enable_if<
             rclcpp::function_traits::arity_comparator<Arity, C>::value
             >::type * = nullptr,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>,
           typename CallbackArgT5 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<5>,
           typename CallbackArgT6 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<6>,
           typename CallbackArgT7 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<7>
           >
  message_filters::Connection registerCallback(const C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4,
            CallbackArgT5 msg5,
            CallbackArgT6 msg6,
            CallbackArgT7 msg7) {
            std::cout << "hooked8" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);
            register_ith_message_as_input<5>(msg5);
            register_ith_message_as_input<6>(msg6);
            register_ith_message_as_input<7>(msg7);

            callback(msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5,
                  std::placeholders::_6,
                  std::placeholders::_7,
                  std::placeholders::_8));
  }

  template<class C,
           std::size_t Arity = 9,
           typename std::enable_if<
             rclcpp::function_traits::arity_comparator<Arity, C>::value
             >::type * = nullptr,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>,
           typename CallbackArgT5 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<5>,
           typename CallbackArgT6 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<6>,
           typename CallbackArgT7 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<7>,
           typename CallbackArgT8 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<8>
           >
  message_filters::Connection registerCallback(const C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4,
            CallbackArgT5 msg5,
            CallbackArgT6 msg6,
            CallbackArgT7 msg7,
            CallbackArgT8 msg8) {
            std::cout << "hooked9" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);
            register_ith_message_as_input<5>(msg5);
            register_ith_message_as_input<6>(msg6);
            register_ith_message_as_input<7>(msg7);
            register_ith_message_as_input<8>(msg8);

            callback(msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5,
                  std::placeholders::_6,
                  std::placeholders::_7,
                  std::placeholders::_8,
                  std::placeholders::_9));
  }

  // non-const C callback&
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
  message_filters::Connection registerCallback(C& callback)
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
  message_filters::Connection registerCallback(C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2) {
            std::cout << "hooked3" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);

            callback(msg0, msg1, msg2);
          };

    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3));
  }

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
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>
           >
  message_filters::Connection registerCallback(C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3) {
            std::cout << "hooked4" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);

            callback(msg0, msg1, msg2, msg3);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4));
  }

  template<class C,
           std::size_t Arity = 5,
           typename std::enable_if<
             rclcpp::function_traits::arity_comparator<Arity, C>::value
             >::type * = nullptr,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>
           >
  message_filters::Connection registerCallback(C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4) {
            std::cout << "hooked5" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);

            callback(msg0, msg1, msg2, msg3, msg4);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5));
  }

  template<class C,
           std::size_t Arity = 6,
           typename std::enable_if<
             rclcpp::function_traits::arity_comparator<Arity, C>::value
             >::type * = nullptr,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>,
           typename CallbackArgT5 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<5>
           >
  message_filters::Connection registerCallback(C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4,
            CallbackArgT5 msg5) {
            std::cout << "hooked6" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);
            register_ith_message_as_input<5>(msg5);

            callback(msg0, msg1, msg2, msg3, msg4, msg5);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5,
                  std::placeholders::_6));
  }

  template<class C,
           std::size_t Arity = 7,
           typename std::enable_if<
             rclcpp::function_traits::arity_comparator<Arity, C>::value
             >::type * = nullptr,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>,
           typename CallbackArgT5 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<5>,
           typename CallbackArgT6 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<6>
           >
  message_filters::Connection registerCallback(C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4,
            CallbackArgT5 msg5,
            CallbackArgT6 msg6) {
            std::cout << "hooked7" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);
            register_ith_message_as_input<5>(msg5);
            register_ith_message_as_input<6>(msg6);

            callback(msg0, msg1, msg2, msg3, msg4, msg5, msg6);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5,
                  std::placeholders::_6,
                  std::placeholders::_7));
  }

  template<class C,
           std::size_t Arity = 8,
           typename std::enable_if<
             rclcpp::function_traits::arity_comparator<Arity, C>::value
             >::type * = nullptr,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>,
           typename CallbackArgT5 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<5>,
           typename CallbackArgT6 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<6>,
           typename CallbackArgT7 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<7>
           >
  message_filters::Connection registerCallback(C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4,
            CallbackArgT5 msg5,
            CallbackArgT6 msg6,
            CallbackArgT7 msg7) {
            std::cout << "hooked8" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);
            register_ith_message_as_input<5>(msg5);
            register_ith_message_as_input<6>(msg6);
            register_ith_message_as_input<7>(msg7);

            callback(msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5,
                  std::placeholders::_6,
                  std::placeholders::_7,
                  std::placeholders::_8));
  }

  template<class C,
           std::size_t Arity = 9,
           typename std::enable_if<
             rclcpp::function_traits::arity_comparator<Arity, C>::value
             >::type * = nullptr,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>,
           typename CallbackArgT5 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<5>,
           typename CallbackArgT6 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<6>,
           typename CallbackArgT7 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<7>,
           typename CallbackArgT8 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<8>
           >
  message_filters::Connection registerCallback(C& callback)
  {
    auto new_callback_lambda
        = [this, callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4,
            CallbackArgT5 msg5,
            CallbackArgT6 msg6,
            CallbackArgT7 msg7,
            CallbackArgT8 msg8) {
            std::cout << "hooked9" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);
            register_ith_message_as_input<5>(msg5);
            register_ith_message_as_input<6>(msg6);
            register_ith_message_as_input<7>(msg7);
            register_ith_message_as_input<8>(msg8);

            callback(msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5,
                  std::placeholders::_6,
                  std::placeholders::_7,
                  std::placeholders::_8,
                  std::placeholders::_9));
  }

  // (const C& callback, T* t)
  template<typename ReturnTypeT, typename... Args, typename T,
           typename C = ReturnTypeT(Args...),
           std::size_t Arity = 2,
           typename std::enable_if<
             sizeof...(Args) == Arity,
             bool>::type = true,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>
           >
  message_filters::Connection registerCallback(ReturnTypeT(T::*callback)(Args...), T* t)
  {
    auto bind_callback = std::bind(callback, t,
                                   std::placeholders::_1,
                                   std::placeholders::_2);
    auto new_callback_lambda
        = [this, bind_callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1) -> void
          {
            std::cout << "hooked2" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);

            bind_callback(msg0, msg1);
          };

    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2));
  }

  template<typename ReturnTypeT, typename... Args, typename T,
           typename C = ReturnTypeT(Args...),
           std::size_t Arity = 3,
           typename std::enable_if<
             sizeof...(Args) == Arity,
             bool>::type = true,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>
           >
  message_filters::Connection registerCallback(ReturnTypeT(T::*callback)(Args...), T* t)
  {
    auto bind_callback = std::bind(callback, t,
                                   std::placeholders::_1,
                                   std::placeholders::_2,
                                   std::placeholders::_3);
    auto new_callback_lambda
        = [this, bind_callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2) -> void
          {
            std::cout << "hooked3" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);

            bind_callback(msg0, msg1, msg2);
          };

    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3));
  }

  template<typename ReturnTypeT, typename... Args, typename T,
           typename C = ReturnTypeT(Args...),
           std::size_t Arity = 4,
           typename std::enable_if<
             sizeof...(Args) == Arity,
             bool>::type = true,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>
           >
  message_filters::Connection registerCallback(ReturnTypeT(T::*callback)(Args...), T* t)
  {
    auto bind_callback = std::bind(callback, t,
                                   std::placeholders::_1,
                                   std::placeholders::_2,
                                   std::placeholders::_3,
                                   std::placeholders::_4);
    auto new_callback_lambda
        = [this, bind_callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3) -> void
          {
            std::cout << "hooked4" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);

            bind_callback(msg0, msg1, msg2, msg3);
          };

    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4));
  }

  template<typename ReturnTypeT, typename... Args, typename T,
           typename C = ReturnTypeT(Args...),
           std::size_t Arity = 5,
           typename std::enable_if<
             sizeof...(Args) == Arity,
             bool>::type = true,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>
           >
  message_filters::Connection registerCallback(ReturnTypeT(T::*callback)(Args...), T* t)
  {
    auto bind_callback = std::bind(callback, t,
                                   std::placeholders::_1,
                                   std::placeholders::_2,
                                   std::placeholders::_3,
                                   std::placeholders::_4,
                                   std::placeholders::_5);
    auto new_callback_lambda
        = [this, bind_callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4) -> void
          {
            std::cout << "hooked5" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);

            bind_callback(msg0, msg1, msg2, msg3, msg4);
          };

    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5));
  }

  template<typename ReturnTypeT, typename... Args, typename T,
           typename C = ReturnTypeT(Args...),
           std::size_t Arity = 6,
           typename std::enable_if<
             sizeof...(Args) == Arity,
             bool>::type = true,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>,
           typename CallbackArgT5 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<5>
           >
  message_filters::Connection registerCallback(ReturnTypeT(T::*callback)(Args...), T* t)
  {
    auto bind_callback = std::bind(callback, t,
                                   std::placeholders::_1,
                                   std::placeholders::_2,
                                   std::placeholders::_3,
                                   std::placeholders::_4,
                                   std::placeholders::_5,
                                   std::placeholders::_6);
    auto new_callback_lambda
        = [this, bind_callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4,
            CallbackArgT5 msg5) -> void
          {
            std::cout << "hooked6" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);
            register_ith_message_as_input<5>(msg5);

            bind_callback(msg0, msg1, msg2, msg3, msg4, msg5);
          };

    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5,
                  std::placeholders::_6));
  }

  template<typename ReturnTypeT, typename... Args, typename T,
           typename C = ReturnTypeT(Args...),
           std::size_t Arity = 7,
           typename std::enable_if<
             sizeof...(Args) == Arity,
             bool>::type = true,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>,
           typename CallbackArgT5 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<5>,
           typename CallbackArgT6 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<6>
           >
  message_filters::Connection registerCallback(ReturnTypeT(T::*callback)(Args...), T* t)
  {
    auto bind_callback = std::bind(callback, t,
                                   std::placeholders::_1,
                                   std::placeholders::_2,
                                   std::placeholders::_3,
                                   std::placeholders::_4,
                                   std::placeholders::_5,
                                   std::placeholders::_6,
                                   std::placeholders::_7);
    auto new_callback_lambda
        = [this, bind_callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4,
            CallbackArgT5 msg5,
            CallbackArgT6 msg6) -> void
          {
            std::cout << "hooked7" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);
            register_ith_message_as_input<5>(msg5);
            register_ith_message_as_input<6>(msg6);

            bind_callback(msg0, msg1, msg2, msg3, msg4, msg5, msg6);
          };

    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5,
                  std::placeholders::_6,
                  std::placeholders::_7));
  }

  template<typename ReturnTypeT, typename... Args, typename T,
           typename C = ReturnTypeT(Args...),
           std::size_t Arity = 8,
           typename std::enable_if<
             sizeof...(Args) == Arity,
             bool>::type = true,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>,
           typename CallbackArgT5 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<5>,
           typename CallbackArgT6 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<6>,
           typename CallbackArgT7 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<7>
           >
  message_filters::Connection registerCallback(ReturnTypeT(T::*callback)(Args...), T* t)
  {
    auto bind_callback = std::bind(callback, t,
                                   std::placeholders::_1,
                                   std::placeholders::_2,
                                   std::placeholders::_3,
                                   std::placeholders::_4,
                                   std::placeholders::_5,
                                   std::placeholders::_6,
                                   std::placeholders::_7,
                                   std::placeholders::_8);
    auto new_callback_lambda
        = [this, bind_callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4,
            CallbackArgT5 msg5,
            CallbackArgT6 msg6,
            CallbackArgT7 msg7) -> void
          {
            std::cout << "hooked8" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);
            register_ith_message_as_input<5>(msg5);
            register_ith_message_as_input<6>(msg6);
            register_ith_message_as_input<7>(msg7);

            bind_callback(msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7);
          };

    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5,
                  std::placeholders::_6,
                  std::placeholders::_7,
                  std::placeholders::_8));
  }

  template<typename ReturnTypeT, typename... Args, typename T,
           typename C = ReturnTypeT(Args...),
           std::size_t Arity = 9,
           typename std::enable_if<
             sizeof...(Args) == Arity,
             bool>::type = true,
           typename CallbackArgT0 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<0>,
           typename CallbackArgT1 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<1>,
           typename CallbackArgT2 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<2>,
           typename CallbackArgT3 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<3>,
           typename CallbackArgT4 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<4>,
           typename CallbackArgT5 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<5>,
           typename CallbackArgT6 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<6>,
           typename CallbackArgT7 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<7>,
           typename CallbackArgT8 =
           typename rclcpp::function_traits::function_traits<C>::template argument_type<8>
           >
  message_filters::Connection registerCallback(ReturnTypeT(T::*callback)(Args...), T* t)
  {
    auto bind_callback = std::bind(callback, t,
                                   std::placeholders::_1,
                                   std::placeholders::_2,
                                   std::placeholders::_3,
                                   std::placeholders::_4,
                                   std::placeholders::_5,
                                   std::placeholders::_6,
                                   std::placeholders::_7,
                                   std::placeholders::_8,
                                   std::placeholders::_9);
    auto new_callback_lambda
        = [this, bind_callback](
            CallbackArgT0 msg0,
            CallbackArgT1 msg1,
            CallbackArgT2 msg2,
            CallbackArgT3 msg3,
            CallbackArgT4 msg4,
            CallbackArgT5 msg5,
            CallbackArgT6 msg6,
            CallbackArgT7 msg7,
            CallbackArgT8 msg8) -> void
          {
            std::cout << "hooked9" << std::endl;

            register_ith_message_as_input<0>(msg0);
            register_ith_message_as_input<1>(msg1);
            register_ith_message_as_input<2>(msg2);
            register_ith_message_as_input<3>(msg3);
            register_ith_message_as_input<4>(msg4);
            register_ith_message_as_input<5>(msg5);
            register_ith_message_as_input<6>(msg6);
            register_ith_message_as_input<7>(msg7);
            register_ith_message_as_input<8>(msg8);

            bind_callback(msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8);
          };

    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5,
                  std::placeholders::_6,
                  std::placeholders::_7,
                  std::placeholders::_8,
                  std::placeholders::_9));
  }

  // (C& callback, T* t)
  // TODO(y-okumura-isp): implement me

 private:
  std::shared_ptr<Sync> sync_ptr_;
  tilde::TildeNode *node_;
  // message id vs topic name.
  // 9 means the number of max messages.
  std::vector<std::string> topic_names_{9};


  // helper function for topic name initialization.
  // I: the index of message
  // F: filter class such as Subscriber<PointCloud2>
  // M: message type such as PointCloud2
  template<std::size_t I,
           typename M,
           typename F>
  void init_topic_name(F& f) {
    // if constexpr (std::is_same_v<F, message_filters::Subscriber<M>>) {
    if constexpr (std::is_same_v<F, tilde_message_filters::TildeSubscriber<M>>) {
       using rclcpp::node_interfaces::get_node_topics_interface;
       auto node_topics_interface = get_node_topics_interface(node_);
       auto topic_name = f.getTopic();
       auto resolved_topic_name = node_topics_interface->resolve_topic_name(topic_name);
       // std::cout << "topic_name[ " << I << "]: "<< resolved_topic_name << std::endl;
       topic_names_[I] = resolved_topic_name;
    }
  }

  // helper function for node->register_message_as_input
  template<std::size_t I,
           class CallbackArgT>
  void register_ith_message_as_input(
      CallbackArgT msg)
  {
    static_assert(I < 9);

    using MessageT = typename std::tuple_element<I, Messages>::type;
    // TODO(y-okumura-isp): support custom deleter
    using MessageDeleter = std::default_delete<MessageT>;
    using ConstRef = const MessageT &;
    using UniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
    using SharedConstPtr = std::shared_ptr<const MessageT>;
    using ConstRefSharedConstPtr = const std::shared_ptr<const MessageT>&;
    using SharedPtr = std::shared_ptr<MessageT>;

    const auto& topic = topic_names_[I];
    if(topic.empty()) return;

    rclcpp::Time subtime, subtime_steady;
    if constexpr (std::is_same_v<CallbackArgT, UniquePtr> ||
                  std::is_same_v<CallbackArgT, SharedConstPtr> ||
                  std::is_same_v<CallbackArgT, ConstRefSharedConstPtr> ||
                  std::is_same_v<CallbackArgT, SharedPtr>) {
        node_->find_subtime(
            msg.get(), topic,
            subtime, subtime_steady);

        // update implicit input info
        node_->register_message_as_input(
            msg.get(), topic,
            subtime, subtime_steady);
    } else if constexpr (std::is_same_v<CallbackArgT, ConstRef>) {
        node_->find_subtime(
            &msg, topic,
            subtime, subtime_steady);

        // update implicit input info
        node_->register_message_as_input(
            &msg, topic,
            subtime, subtime_steady);
    } else {
      // todo(y-okumura-isp): implement me
    }
  }
};
}  // tilde_message_filters


#endif // TILDE_MESSAGE_FILTERS__TILDE_SYNCHRONIZER_H
