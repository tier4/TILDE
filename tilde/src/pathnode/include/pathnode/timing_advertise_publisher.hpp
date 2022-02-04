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

#ifndef PATHNODE__TIMING_ADVERTISE_PUBLISHER_HPP_
#define PATHNODE__TIMING_ADVERTISE_PUBLISHER_HPP_

#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "rclcpp/publisher.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"

#include "path_info_msg/msg/pub_info.hpp"

#include "pathnode/tp.h"

#define TILDE_S_TO_NS(seconds) ((seconds) * (1000LL * 1000LL * 1000LL))

namespace pathnode
{

/*
template <class T>
auto get_timestamp(rclcpp::Time t, T a) -> decltype(a.header.timestamp, t)
{
  std::cout << "T a: true" << std::endl;
  return a.header.timestamp;
}
*/

class InputInfo
{
public:
  rclcpp::Time sub_time;
  rclcpp::Time sub_time_steady;
  bool has_header_stamp;
  rclcpp::Time header_stamp;

  InputInfo()
  : has_header_stamp(false) {}
};

template<typename M, typename = void>
struct HasHeader : public std::false_type {};

template<typename M>
struct HasHeader<M, decltype((void) M::header)>: std::true_type {};

template<typename M, typename Enable = void>
struct Process
{
  static rclcpp::Time get_timestamp2(rclcpp::Time t, M * m)
  {
    // std::cout << "rclcpp::Time2" << std::endl;
    (void) m;
    return t;
  }

  static rclcpp::Time get_timestamp3(rclcpp::Time t, const M * m)
  {
    // std::cout << "rclcpp::Time3" << std::endl;
    (void) m;
    return t;
  }
};

template<typename M>
struct Process<M, typename std::enable_if<HasHeader<M>::value>::type>
{
  static rclcpp::Time get_timestamp2(rclcpp::Time t, M * m)
  {
    // std::cout << "header Time2" << std::endl;
    (void) t;
    return m->header.stamp;
  }

  static rclcpp::Time get_timestamp3(rclcpp::Time t, const M * m)
  {
    // std::cout << "header Time3" << std::endl;
    (void) t;
    return m->header.stamp;
  }
};

template<class T>
auto get_timestamp(rclcpp::Time t, T * a)->decltype(rclcpp::Time(a->header.stamp), t)
{
  // std::cout << "get header timestamp" << std::endl;
  rclcpp::Time ret(a->header.stamp);
  return ret;
}

rclcpp::Time get_timestamp(rclcpp::Time t, ...);

class TimingAdvertisePublisherBase
{
public:
  using InfoMsg = path_info_msg::msg::PubInfo;

  explicit TimingAdvertisePublisherBase(
    std::shared_ptr<rclcpp::Clock> clock,
    std::shared_ptr<rclcpp::Clock> steady_clock,
    const std::string & node_fqn);

  void set_input_info(
    const std::string & sub_topic,
    const std::shared_ptr<const InputInfo> p);

  /** Set sub callback time information for explicit API.
   * It's a FW API, so your code should not call this.
   */
  void set_explicit_subtime(
    const std::string & sub_topic,
    const std::shared_ptr<const InputInfo> p);

  /**
   * assume set_explicit_subtime is already called
   */
  void add_explicit_input_info(
    const std::string & sub_topic,
    const rclcpp::Time & stamp);

  void set_input_info(path_info_msg::msg::PubInfo & info_msg);

  void set_max_sub_callback_infos_sec(size_t sec);

protected:
  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<rclcpp::Clock> steady_clock_;

  const std::string node_fqn_;
  int64_t seq_;

  std::map<std::string, std::string> sub_topics_;

private:
  // parent node subscription topic vs InputInfo
  std::map<std::string, std::shared_ptr<const InputInfo>> input_infos_;

  // explicit InputInfo
  // If this is set, FW creates PubInfo only by this info
  std::map<std::string, std::vector<InputInfo>> explicit_input_infos_;
  // topic, header stamp vs sub callback time
  std::map<std::string,
    std::map<rclcpp::Time, std::shared_ptr<const InputInfo>>> explicit_sub_time_infos_;

  // how many seconds to preserve explicit_sub_callback_infos per topic
  size_t MAX_SUB_CALLBACK_INFOS_SEC_;
};


template<typename MessageT,
  typename AllocatorT = std::allocator<void>>
class TimingAdvertisePublisher : public TimingAdvertisePublisherBase
{
private:
  using MessageAllocatorTraits = rclcpp::allocator::AllocRebind<MessageT, AllocatorT>;
  using MessageAllocator = typename MessageAllocatorTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAllocator, MessageT>;
  using PublisherT = rclcpp::Publisher<MessageT, AllocatorT>;
  using PubInfoPublisher = rclcpp::Publisher<InfoMsg>;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(TimingAdvertisePublisher)

  TimingAdvertisePublisher(
    std::shared_ptr<PubInfoPublisher> info_pub,
    std::shared_ptr<PublisherT> pub,
    const std::string & node_fqn,
    std::shared_ptr<rclcpp::Clock> clock,
    std::shared_ptr<rclcpp::Clock> steady_clock,
    bool enable)
  : TimingAdvertisePublisherBase(clock, steady_clock, node_fqn),
    info_pub_(info_pub),
    pub_(pub),
    enable_(enable)
  {
  }

  void
  publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    if (enable_) {
      publish_info(Process<MessageT>::get_timestamp2(clock_->now(), msg.get()));
    }
    pub_->publish(std::move(msg));
  }

  void
  publish(const MessageT & msg)
  {
    if (enable_) {
      publish_info(Process<MessageT>::get_timestamp3(clock_->now(), &msg));
    }
    pub_->publish(msg);
  }

  void
  publish(const rcl_serialized_message_t & serialized_msg)
  {
    std::cout << "TAP publish serialized message (not supported)" << std::endl;
    // publish_info(get_timestamp(clock_->now(), msg.get()));
    pub_->publish(serialized_msg);
  }

  void
  publish(const rclcpp::SerializedMessage & serialized_msg)
  {
    std::cout << "TAP publish SerializedMessage (not supported)" << std::endl;
    pub_->publish(serialized_msg);
  }

  void
  publish(rclcpp::LoanedMessage<MessageT, AllocatorT> && loaned_msg)
  {
    std::cout << "TAP publish LoanedMessage (not supported)" << std::endl;
    pub_->publish(loaned_msg);
  }

  // TODO(y-okumura-isp) get_allocator

  size_t
  get_subscription_count() const
  {
    return pub_->get_subscription_count();
  }

  size_t
  get_intra_process_subscription_count() const
  {
    return pub_->get_intra_process_subscription_count();
  }

  RCLCPP_PUBLIC
  const char *
  get_topic_name() const
  {
    return pub_->get_topic_name();
  }

private:
  std::shared_ptr<PubInfoPublisher> info_pub_;
  std::shared_ptr<PublisherT> pub_;
  const std::string node_fqn_;
  bool enable_;

  /**
   * t: header stamp
   * TODO: we cannot distinguish t is header.stamp or now in current implementation
   */
  void publish_info(rclcpp::Time && t)
  {
    auto msg = std::make_unique<path_info_msg::msg::PubInfo>();
    msg->header.stamp = clock_->now();
    // msg->header.frame_id  // Nothing todo

    msg->output_info.topic_name = pub_->get_topic_name();
    msg->output_info.node_fqn = node_fqn_;
    msg->output_info.seq = seq_;
    seq_++;
    msg->output_info.pub_time = clock_->now();
    msg->output_info.pub_time_steady = steady_clock_->now();
    msg->output_info.header_stamp = t;

    set_input_info(*msg);

    for (auto & input_info : msg->input_infos) {
      auto pubtime = TILDE_S_TO_NS(msg->output_info.pub_time.sec) +
        msg->output_info.pub_time.nanosec;
      auto subtime_steady = TILDE_S_TO_NS(input_info.sub_time_steady.sec) +
        input_info.sub_time_steady.nanosec;
      auto sub = &sub_topics_[input_info.topic_name];
      tracepoint(
        TRACEPOINT_PROVIDER,
        tilde_publish,
        this,
        pubtime,
        sub,
        subtime_steady
      );
    }

    info_pub_->publish(std::move(msg));
  }
};

}  // namespace pathnode

#endif  // PATHNODE__TIMING_ADVERTISE_PUBLISHER_HPP_
