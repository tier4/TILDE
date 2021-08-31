#include "pathnode/path_node.hpp"

using pathnode::PathNode;

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
