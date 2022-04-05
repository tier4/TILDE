#ifndef TILDE_MESSAGE_FILTERS__TILDE_TIME_SYNCHRONIZER_H
#define TILDE_MESSAGE_FILTERS__TILDE_TIME_SYNCHRONIZER_H

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

  template<class C,
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
            std::cout << "hooked" << std::endl;
            callback(msg0, msg1);
          };
    return sync_ptr_->registerCallback(
        std::bind(new_callback_lambda,
                  std::placeholders::_1,
                  std::placeholders::_2));
  }

private:
  std::shared_ptr<Sync> sync_ptr_;
};



}  // tilde_message_filters


#endif // TILDE_MESSAGE_FILTERS__TILDE_TIME_SYNCHRONIZER_H
