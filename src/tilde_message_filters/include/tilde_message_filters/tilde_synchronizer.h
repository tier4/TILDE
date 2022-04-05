#ifndef TILDE_MESSAGE_FILTERS__TILDE_SYNCHRONIZER_H
#define TILDE_MESSAGE_FILTERS__TILDE_SYNCHRONIZER_H

#include <memory>

#include "message_filters/time_synchronizer.h"
#include "message_filters/time_synchronizer.h"

namespace tilde_message_filters
{
template<class Policy>
class TildeSynchronizer
{
  using Sync = message_filters::Synchronizer<Policy>;

public:
  template<class F0, class F1>
  TildeSynchronizer(F0& f0, F1& f1)
  {
    sync_ptr_ = std::make_shared<Sync>(f0, f1);
  }

  template<class F0, class F1>
  TildeSynchronizer(const Policy& policy, F0& f0, F1& f1)
  {
    sync_ptr_ = std::make_shared<Sync>(policy, f0, f1);
  }

  template<class F0, class F1, class F2>
  TildeSynchronizer(F0& f0, F1& f1, F2& f2)
  {
    sync_ptr_ = std::make_shared<Sync>(f0, f1, f2);
  }

  template<class F0, class F1, class F2>
  TildeSynchronizer(const Policy& policy, F0& f0, F1& f1, F2& f2)
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

private:
  std::shared_ptr<Sync> sync_ptr_;
};



}  // tilde_message_filters


#endif // TILDE_MESSAGE_FILTERS__TILDE_SYNCHRONIZER_H
