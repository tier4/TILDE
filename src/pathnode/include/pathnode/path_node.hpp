#ifndef PATH_NODE_HPP_
#define PATH_NODE_HPP_

#include "rclcpp/node.hpp"
#include "rclcpp/visibility_control.hpp"

namespace pathnode
{

class PathNode : public rclcpp::Node
{
public:
  RCLCPP_PUBLIC
  explicit PathNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_PUBLIC
  explicit PathNode(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_PUBLIC
  virtual ~PathNode();
};

} // namespace pathnode

#endif // PATH_NODE_HPP_
