#ifndef TIMING_ADVERTISE_PUBLISHER_HPP_
#define TIMING_ADVERTISE_PUBLISHER_HPP_

#include <memory>
#include <type_traits>

#include "rclcpp/publisher.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"

#include "path_info_msg/msg/topic_info.hpp"

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

template<typename M, typename = void>
struct HasHeader : public std::false_type {};

template<typename M>
struct HasHeader<M, decltype((void) M::header)>: std::true_type {};

template <typename M, typename Enable = void> struct Process {
  static rclcpp::Time get_timestamp2(rclcpp::Time t, M *m) {
    std::cout << "rclcpp::Time2" << std::endl;
    return t;
  }

  static rclcpp::Time get_timestamp3(rclcpp::Time t, const M *m) {
    std::cout << "rclcpp::Time3" << std::endl;
    return t;
  }
};

template <typename M>
struct Process<M, typename std::enable_if<HasHeader<M>::value>::type> {
  static rclcpp::Time get_timestamp2(rclcpp::Time t, M *m) {
    std::cout << "header Time2" << std::endl;
    return m->header.stamp;
  }

  static rclcpp::Time get_timestamp3(rclcpp::Time t, const M *m) {
    std::cout << "header Time3" << std::endl;
    return m->header.stamp;
  }

};

template <class T>
auto get_timestamp(rclcpp::Time t, T *a) -> decltype(rclcpp::Time(a->header.stamp), t)
{
  std::cout << "get header timestamp" << std::endl;
  rclcpp::Time ret(a->header.stamp);
  return ret;
}

rclcpp::Time get_timestamp(rclcpp::Time t, ...);

template<typename MessageT,
         typename AllocatorT = std::allocator<void>>
class TimingAdvertisePublisher
{
  using MessageAllocatorTraits = rclcpp::allocator::AllocRebind<MessageT, AllocatorT>;
  using MessageAllocator = typename MessageAllocatorTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAllocator, MessageT>;
  using PublisherT = rclcpp::Publisher<MessageT, AllocatorT>;
  using TopicInfoPublisher = rclcpp::Publisher<path_info_msg::msg::TopicInfo>;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(TimingAdvertisePublisher)

  TimingAdvertisePublisher(
      std::shared_ptr<TopicInfoPublisher> info_pub,
      std::shared_ptr<PublisherT> pub,
      const std::string &node_fqn)
      : info_pub_(info_pub), pub_(pub), seq_(0), node_fqn_(node_fqn), clock_(std::make_unique<rclcpp::Clock>())
  {
  }

  void
  publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    publish_info(Process<MessageT>::get_timestamp2(clock_->now(), msg.get()));
    pub_->publish(std::move(msg));
  }

  void
  publish(const MessageT & msg)
  {
    publish_info(Process<MessageT>::get_timestamp3(clock_->now(), &msg));
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

  // TODO get_allocator

private:
  std::shared_ptr<TopicInfoPublisher> info_pub_;
  std::shared_ptr<PublisherT> pub_;
  int64_t seq_;
  const std::string node_fqn_;
  std::unique_ptr<rclcpp::Clock> clock_;

  void publish_info(rclcpp::Time t)
  {
    auto info = std::make_unique<path_info_msg::msg::TopicInfo>();
    info->seq = seq_;
    seq_++;
    info->node_fqn = node_fqn_;
    info->topic_name = pub_->get_topic_name();

    info->callback_start = t;
    info_pub_->publish(std::move(info));
  }
};

} // namespace pathnode

#endif // TIMING_ADVERTISE_PUBLISHER_HPP_
