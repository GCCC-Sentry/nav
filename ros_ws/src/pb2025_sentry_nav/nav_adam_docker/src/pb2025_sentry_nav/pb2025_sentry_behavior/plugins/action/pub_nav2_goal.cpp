// Copyright 2025 Lihan Chen
// Licensed under the Apache License, Version 2.0

#include "pb2025_sentry_behavior/plugins/action/pub_nav2_goal.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb2025_sentry_behavior
{

PubNav2Goal::PubNav2Goal(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, config, params)
{
}

BT::PortsList PubNav2Goal::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("x", 0.0, "Goal X coordinate"),
    BT::InputPort<double>("y", 0.0, "Goal Y coordinate"),
    BT::InputPort<double>("yaw", 0.0, "Goal Yaw angle (rad)")
  });
}

bool PubNav2Goal::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  // Read three separate double values
  double x, y, yaw;
  if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw)) {
    return false;
  }
  
  // Build PoseStamped message
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();
  
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0.0;
  
  // Convert yaw to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  msg.pose.orientation = tf2::toMsg(q);

  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubNav2Goal, "PubNav2Goal");
