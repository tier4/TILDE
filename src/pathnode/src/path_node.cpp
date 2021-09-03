#include "pathnode/path_node.hpp"

using pathnode::PathNodeSubscriptionOptions;
using pathnode::PathNodeInfo;
using pathnode::PathNode;

PathNodeSubscriptionOptions::PathNodeSubscriptionOptions():
    path_deadline_duration_(0, 0)
{
}

PathNodeInfo::PathNodeInfo():
    CLOCK_TYPE(RCL_SYSTEM_TIME),
    path_start_time_(0, 0, CLOCK_TYPE),
    path_deadline_duration_(0, 0)
{
}

PathNode::PathNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options):
    Node(node_name, options),
    CLOCK_TYPE(RCL_SYSTEM_TIME)
{
}

PathNode::PathNode(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options):
    Node(node_name, namespace_, options),
    CLOCK_TYPE(RCL_SYSTEM_TIME)
{
}

PathNode::~PathNode()
{
}
