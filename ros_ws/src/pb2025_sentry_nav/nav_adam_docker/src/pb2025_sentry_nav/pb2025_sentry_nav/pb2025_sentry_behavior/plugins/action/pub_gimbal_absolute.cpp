// Copyright 2025 Lihan Chen
// Licensed under the Apache License, Version 2.0

#include "pb2025_sentry_behavior/plugins/action/pub_gimbal_absolute.hpp"
#include <thread>
#include <chrono>

namespace pb2025_sentry_behavior
{

PublishGimbalAbsolute::PublishGimbalAbsolute(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosTopicPubNode<pb_rm_interfaces::msg::GimbalCmd>(name, config, params)
{
}

BT::PortsList PublishGimbalAbsolute::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("yaw", 0.0, "Expected Yaw angle (rad)"),
    BT::InputPort<double>("pitch", 0.0, "Expected Pitch angle (rad)"),
    // 修改点 1: 类型改为 int
    BT::InputPort<int>("duration", "Publish duration")
  });
}

bool PublishGimbalAbsolute::setMessage(pb_rm_interfaces::msg::GimbalCmd & msg)
{
  msg.header.stamp = node_->now();
  msg.yaw_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  msg.pitch_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;

  double pitch, yaw;
  if (!getInput("pitch", pitch) || !getInput("yaw", yaw)) {
    return false;
  }

  msg.position.pitch = pitch;
  msg.position.yaw = yaw;
  msg.velocity = pb_rm_interfaces::msg::Gimbal();

  // 修改点 2: 读取 int 并转换为 milliseconds
  auto duration_ms = getInput<int>("duration");
  if(duration_ms && duration_ms.value() > 0)
  {
      std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms.value()));
  }

  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishGimbalAbsolute, "PublishGimbalAbsolute");