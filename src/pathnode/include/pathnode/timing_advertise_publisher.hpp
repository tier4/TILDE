#ifndef TIMING_ADVERTISE_PUBLISHER_HPP_
#define TIMING_ADVERTISE_PUBLISHER_HPP_

#include "rclcpp/publisher.hpp"

namespace pathnode
{
template<typename MessageT,
         typename AllocatorT = std::allocator<void>>
class TimingAdvertisePublisher
{
  using MessageAllocatorTraits = rclcpp::allocator::AllocRebind<MessageT, AllocatorT>;
  using MessageAllocator = typename MessageAllocatorTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAllocator, MessageT>;
  using PublisherT = rclcpp::Publisher<MessageT, AllocatorT>;

public:
  TimingAdvertisePublisher(std::shared_ptr<PublisherT> pub)
      : pub_(pub)
  {}

  void
  publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    std::cout << "TAP publish" << std::endl;
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
  std::shared_ptr<PublisherT> pub_;
};

} // namespace pathnode

#endif // TIMING_ADVERTISE_PUBLISHER_HPP_
