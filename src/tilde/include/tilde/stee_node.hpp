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

#ifndef TILDE__STEE_NODE_HPP_
#define TILDE__STEE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tilde/stee_publisher.hpp"
#include "tilde/message_conversion.hpp"

namespace tilde
{
class SteeNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SteeNode)

  /// see corresponding rclcpp::Node constructor
  RCLCPP_PUBLIC
  explicit SteeNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// see corresponding rclcpp::Node constructor
  RCLCPP_PUBLIC
  explicit SteeNode(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_PUBLIC
  virtual ~SteeNode();

  template<
    typename MessageT,
    typename ConvertedMessageT = ConvertedMessageType<MessageT>,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>,
    typename ConvertedPublisherT = rclcpp::Publisher<ConvertedMessageT, AllocatorT>,
    typename SteePublisherT = SteePublisher<MessageT, ConvertedMessageT, AllocatorT>>
  std::shared_ptr<SteePublisherT>
  create_stee_publisher(
      const std::string & topic_name,
      const rclcpp::QoS & qos,
      const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options =
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())
  {
    auto pub = create_publisher<MessageT, AllocatorT, PublisherT>(
        topic_name, qos, options);
    auto converted_pub = create_publisher<ConvertedMessageT, AllocatorT, ConvertedPublisherT>(
        topic_name + "/stee", qos, options);
    auto stee_pub = std::make_shared<SteePublisherT>(
        pub, converted_pub,
        get_fully_qualified_name(),
        this->get_clock(),
        steady_clock_);
    return stee_pub;
  }

private:
  void init();
  std::shared_ptr<rclcpp::Clock> steady_clock_;
};

}  // namespace tilde

#endif  // TILDE__STEE_NODE_HPP_
