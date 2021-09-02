#include "pathnode/path_node.hpp"

using pathnode::PathNodeSubscriptionOptions;
using pathnode::PathNode;

PathNodeSubscriptionOptions::PathNodeSubscriptionOptions(
      const std::string &path_name,
      bool is_first,
      const rclcpp::Duration &path_deadline_duration):
    path_name_(path_name), is_first_(is_first), path_deadline_duration_(0, 0)
{
  path_deadline_duration_ = path_deadline_duration;
}

PathNode::PathNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options):
    Node(node_name, options)
{
}

PathNode::PathNode(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options):
    Node(node_name, namespace_, options)
{
}

PathNode::~PathNode()
{
}
