#ifndef TIMING_ADVERTISE_PUBLISHER_HPP_
#define TIMING_ADVERTISE_PUBLISHER_HPP_

#include "rclcpp/publisher.hpp"
#include "rclcpp/clock.hpp"

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

template <class T>
auto get_timestamp(rclcpp::Time t, const T *a) -> decltype(a->header.timestamp, t)
{
  std::cout << "get header timestamp" << std::endl;
  return a->header.timestamp;
}

auto get_timestamp(rclcpp::Time t, ...) -> rclcpp::Time
{
  std::cout << "get rclcpp::Time t" << std::endl;
  return t;
}

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
    std::cout << "TAP publish" << std::endl;
    auto info = std::make_unique<path_info_msg::msg::TopicInfo>();
    info->seq = seq_;
    seq_++;
    info->node_fqn = node_fqn_;
    info->topic_name = pub_->get_topic_name();

    info->callback_start = get_timestamp(clock_->now(), msg.get());
    info_pub_->publish(std::move(info));

    pub_->publish(std::move(msg));
  }

  void
  publish(const MessageT & msg)
  {
    std::cout << "TAP publish" << std::endl;
    pub_->publish(msg);
  }

  void
  publish(const rcl_serialized_message_t & serialized_msg)
  {
    std::cout << "TAP publish" << std::endl;
    pub_->publish(serialized_msg);
  }

  void
  publish(const rclcpp::SerializedMessage & serialized_msg)
  {
    std::cout << "TAP publish" << std::endl;
    pub_->publish(serialized_msg);
  }

  void
  publish(rclcpp::LoanedMessage<MessageT, AllocatorT> && loaned_msg)
  {
    std::cout << "TAP publish" << std::endl;
    pub_->publish(loaned_msg);
  }

  // TODO get_allocator

private:
  std::shared_ptr<TopicInfoPublisher> info_pub_;
  std::shared_ptr<PublisherT> pub_;
  int64_t seq_;
  const std::string node_fqn_;
  std::unique_ptr<rclcpp::Clock> clock_;
};

} // namespace pathnode

#endif // TIMING_ADVERTISE_PUBLISHER_HPP_
