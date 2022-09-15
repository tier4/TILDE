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

#ifndef TILDE__TILDE_PUBLISHER_HPP_
#define TILDE__TILDE_PUBLISHER_HPP_

#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/publisher.hpp"
#include "tilde/tp.h"
#include "tilde_msg/msg/message_tracking_tag.hpp"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#define TILDE_S_TO_NS(seconds) ((seconds) * (1000LL * 1000LL * 1000LL))

namespace tilde
{

/// Internal class to hold input message information
class InputInfo
{
public:
  rclcpp::Time sub_time;
  rclcpp::Time sub_time_steady;
  bool has_header_stamp;
  rclcpp::Time header_stamp;

  InputInfo() : has_header_stamp(false) {}

  bool operator==(const InputInfo & rhs) const;
};

/// detect header field, not found case
template <typename M, typename = void>
struct HasHeader : public std::false_type
{
};

/// detect header field, found case
template <typename M>
struct HasHeader<M, decltype((void)M::header)> : std::true_type
{
};

/// has top level stamp
template <typename M, typename = void>
struct HasStamp : public std::false_type
{
};

template <typename M>
struct HasStamp<M, decltype((void)M::stamp)> : public std::true_type
{
};

/// has top level stamp and no header
template <typename M, class Enable = void>
struct HasStampWithoutHeader : public std::false_type
{
};

template <typename M>
struct HasStampWithoutHeader<
  M, typename std::enable_if<std::conjunction_v<std::negation<HasHeader<M>>, HasStamp<M>>>::type>
: public std::true_type
{
};

/// SFINAE to get header.stamp, not found case
template <typename M, typename Enable = void>
struct Process
{
  /// stamp getter for non-const pointer without header field
  /**
   * \param[in] m message
   */
  static std::optional<rclcpp::Time> get_timestamp(M * m)
  {
    (void)m;
    return {};
  }

  /// stamp getter for const pointer without header field
  /**
   * Return dummy stamp as M has no header field.
   *
   * \param[in] m message
   */
  static std::optional<rclcpp::Time> get_timestamp_from_const(const M * m)
  {
    (void)m;
    return {};
  }
};

/// SFINAEEs to get header.stamp, found case
template <typename M>
struct Process<M, typename std::enable_if<HasHeader<M>::value>::type>
{
  /// stamp getter for non-const pointer with header field
  /**
   * Return header.stamp
   *
   * \param[in] m message
   */
  static std::optional<rclcpp::Time> get_timestamp(M * m) { return m->header.stamp; }

  /// stamp getter for const pointer with header field
  /**
   * Return header.stamp
   *
   * \param[in] m message
   */
  static std::optional<rclcpp::Time> get_timestamp_from_const(const M * m)
  {
    return m->header.stamp;
  }
};

/// SFINAE to get top level stamp, found case
template <typename M>
struct Process<M, typename std::enable_if<HasStampWithoutHeader<M>::value>::type>
{
  /// stamp getter for non-const pointer with header field
  /**
   * Return header.stamp
   *
   * \param[in] m message
   */
  static std::optional<rclcpp::Time> get_timestamp(M * m) { return m->stamp; }

  /// stamp getter for const pointer with header field
  /**
   * Return header.stamp
   *
   * \param[in] m message
   */
  static std::optional<rclcpp::Time> get_timestamp_from_const(const M * m) { return m->stamp; }
};

template <class T>
auto get_timestamp(rclcpp::Time t, T * a) -> decltype(rclcpp::Time(a->header.stamp), t)
{
  // std::cout << "get header timestamp" << std::endl;
  rclcpp::Time ret(a->header.stamp);
  return ret;
}

rclcpp::Time get_timestamp(rclcpp::Time t, ...);

class TildePublisherBase
{
public:
  using InfoMsg = tilde_msg::msg::MessageTrackingTag;

  /// Constructor
  /**
   * \param[in] clock for RCL_ROS_TIME
   * \param[in] clock for RCL_STEADY_TIME
   * \param[in] node_fqn a node name
   */
  explicit TildePublisherBase(
    std::shared_ptr<rclcpp::Clock> clock, std::shared_ptr<rclcpp::Clock> steady_clock,
    const std::string & node_fqn);

  /// Set implicit input info
  /**
   * This is a TILDE internal API for connecting input and output.
   *
   * \param[in] sub_topic Subscribed topic name
   * \param[in] p InputInfo to set
   */
  void set_implicit_input_info(
    const std::string & sub_topic, const std::shared_ptr<const InputInfo> p);

  /// Explicit API helper
  /**
   * It's a TILDE internal API for connecting input and output by holding
   * "sub_topic + stamp" vs InputInfo.
   *
   * \param[in] sub_topic Subscribed topic name
   * \param[in] p InputInfo
   */
  void set_explicit_subscription_time(
    const std::string & sub_topic, const std::shared_ptr<const InputInfo> p);

  /// Explicit API
  /**
   * Declare input messages explicitly.
   * Specify all messages you use before publishing the message.
   *
   * TildePublisher gets corresponding InputInfo from topic name + stamp.
   * If not found, input_info.header_stamp in MessageTrackingTag is filled
   * but sub_time and sub_time_steady are zero cleared.
   *
   * \param[in] sub_topic used topic
   * \param[in] stamp header.stamp of the used message
   */
  void add_explicit_input_info(const std::string & sub_topic, const rclcpp::Time & stamp);

  /// Fill input info field of the argument message
  /**
   * It's a TILDE internal API.
   * Fill intput info field according to implicit and explicit info.
   * Explicit info is cleared after calling this.
   *
   * \param[out] info_msg Target message
   */
  void fill_input_info(tilde_msg::msg::MessageTrackingTag & info_msg);

  /// Set how long to hold InputInfo
  /**
   * TildePublisher holds InputInfo of every message just a seconds to
   * fill in sub_time and sub_time_steady fields of MessageTrackingTag
   * for explicit API.
   * You can adjust how many seconds to hold InputInfo.
   * The default is 2 seconds.
   *
   * \param[in] sec how many seconds to hold InputInfo
   */
  void set_max_sub_callback_infos_sec(size_t sec);

  /// Get input info
  /**
   * \param[in] topic fully qualified topic name
   * \param[in] header_stamp target header.stamp
   * \param[out] info output data
   *
   * \return true if found else false
   */
  bool get_input_info(
    const std::string & topic, const rclcpp::Time & header_stamp, InputInfo & info);

  void print_input_infos();

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
  // If this is set, FW creates MessageTrackingTag only by this info
  bool is_explicit_;
  std::map<std::string, std::vector<InputInfo>> explicit_input_infos_;
  // topic, header stamp vs sub callback time
  std::map<std::string, std::map<rclcpp::Time, std::shared_ptr<const InputInfo>>>
    explicit_sub_time_infos_;

  // how many seconds to preserve explicit_sub_callback_infos per topic
  size_t MAX_SUB_CALLBACK_INFOS_SEC_;
};

/// rclcpp::Publisher TILDE version
template <typename MessageT, typename AllocatorT = std::allocator<void>>
class TildePublisher : public TildePublisherBase
{
private:
  using MessageAllocatorTraits = rclcpp::allocator::AllocRebind<MessageT, AllocatorT>;
  using MessageAllocator = typename MessageAllocatorTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAllocator, MessageT>;
  using PublisherT = rclcpp::Publisher<MessageT, AllocatorT>;
  using MessageTrackingTagPublisher = rclcpp::Publisher<InfoMsg>;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(TildePublisher)

  /// Default constructor
  TildePublisher(
    std::shared_ptr<MessageTrackingTagPublisher> info_pub, std::shared_ptr<PublisherT> pub,
    const std::string & node_fqn, std::shared_ptr<rclcpp::Clock> clock,
    std::shared_ptr<rclcpp::Clock> steady_clock, bool enable)
  : TildePublisherBase(clock, steady_clock, node_fqn),
    info_pub_(info_pub),
    pub_(pub),
    enable_(enable)
  {
  }

  void publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    if (enable_) {
      auto stamp = Process<MessageT>::get_timestamp(msg.get());
      publish_info(stamp);
    }
    pub_->publish(std::move(msg));
  }

  void publish(const MessageT & msg)
  {
    if (enable_) {
      auto stamp = Process<MessageT>::get_timestamp_from_const(&msg);
      publish_info(stamp);
    }
    pub_->publish(msg);
  }

  /**
   * publish() variant
   * can send a main message but cannot send the corresponding MessageTrackingTag
   */
  void publish(const rcl_serialized_message_t & serialized_msg)
  {
    std::cout << "publish serialized message (not supported)" << std::endl;
    // publish_info(get_timestamp(clock_->now(), msg.get()));
    pub_->publish(serialized_msg);
  }

  /**
   * publish() variant
   * can send a main message but cannot send the corresponding MessageTrackingTag
   */
  void publish(const rclcpp::SerializedMessage & serialized_msg)
  {
    std::cout << "publish SerializedMessage (not supported)" << std::endl;
    pub_->publish(serialized_msg);
  }

  /**
   * publish() variant
   * can send a main message but cannot send the corresponding MessageTrackingTag
   */
  void publish(rclcpp::LoanedMessage<MessageT, AllocatorT> && loaned_msg)
  {
    std::cout << "publish LoanedMessage (not supported)" << std::endl;
    pub_->publish(loaned_msg);
  }

  // TODO(y-okumura-isp) get_allocator

  size_t get_subscription_count() const { return pub_->get_subscription_count(); }

  size_t get_intra_process_subscription_count() const
  {
    return pub_->get_intra_process_subscription_count();
  }

  RCLCPP_PUBLIC
  const char * get_topic_name() const { return pub_->get_topic_name(); }

private:
  std::shared_ptr<MessageTrackingTagPublisher> info_pub_;
  std::shared_ptr<PublisherT> pub_;
  const std::string node_fqn_;
  bool enable_;

  /// Publish MessageTrackingTag
  /**
   * \param has_header_stamp whether main message has header.stamp
   * \param t header stamp
   */
  void publish_info(const std::optional<rclcpp::Time> & t)
  {
    bool has_header_stamp = t ? true : false;

    auto msg = std::make_unique<tilde_msg::msg::MessageTrackingTag>();
    msg->header.stamp = clock_->now();
    // msg->header.frame_id  // Nothing todo

    msg->output_info.topic_name = pub_->get_topic_name();
    msg->output_info.node_fqn = node_fqn_;
    msg->output_info.seq = seq_;
    seq_++;
    msg->output_info.pub_time = clock_->now();
    msg->output_info.pub_time_steady = steady_clock_->now();
    msg->output_info.has_header_stamp = has_header_stamp;
    if (has_header_stamp) {
      msg->output_info.header_stamp = *t;
    }

    fill_input_info(*msg);

    for (auto & input_info : msg->input_infos) {
      auto pub_time =
        TILDE_S_TO_NS(msg->output_info.pub_time.sec) + msg->output_info.pub_time.nanosec;
      auto sub_time_steady =
        TILDE_S_TO_NS(input_info.sub_time_steady.sec) + input_info.sub_time_steady.nanosec;
      auto sub = &sub_topics_[input_info.topic_name];
      tracepoint(TRACEPOINT_PROVIDER, tilde_publish, this, pub_time, sub, sub_time_steady);
    }

    info_pub_->publish(std::move(msg));
  }
};

}  // namespace tilde

#endif  // TILDE__TILDE_PUBLISHER_HPP_
