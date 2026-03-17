// Copyright 2025 Lihan Chen
// Licensed under the Apache License, Version 2.0

#include "pb2025_sentry_behavior/plugins/action/pub_gimbal_velocity.hpp"
#include <thread>
#include <chrono>

namespace pb2025_sentry_behavior
{

PublishGimbalVelocity::PublishGimbalVelocity(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosTopicPubNode<pb_rm_interfaces::msg::GimbalCmd>(name, config, params)
{
}

BT::PortsList PublishGimbalVelocity::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("gimbal_vel_pitch", 0.0, "Pitch velocity (rad/s)"),
    BT::InputPort<double>("gimbal_vel_yaw", 0.0, "Yaw velocity (rad/s)"),
    BT::InputPort<double>("pitch_min", -1.57, "Minimum pitch range"),
    BT::InputPort<double>("pitch_max", 1.57, "Maximum pitch range"),
    BT::InputPort<double>("yaw_min", -3.14, "Minimum yaw range"),
    BT::InputPort<double>("yaw_max", 3.14, "Maximum yaw range"),
    // 修改点 1: 类型改为 int
    BT::InputPort<int>("duration", "Publish duration")
  });
}

bool PublishGimbalVelocity::setMessage(pb_rm_interfaces::msg::GimbalCmd & msg)
{
  msg.header.stamp = node_->now();
  msg.yaw_type = pb_rm_interfaces::msg::GimbalCmd::VELOCITY;
  msg.pitch_type = pb_rm_interfaces::msg::GimbalCmd::VELOCITY;

  double vel_pitch, vel_yaw;
  if (!getInput("gimbal_vel_pitch", vel_pitch) || !getInput("gimbal_vel_yaw", vel_yaw)) {
    return false;
  }

  double pitch_min, pitch_max, yaw_min, yaw_max;
  if (
    !getInput("pitch_min", pitch_min) || !getInput("pitch_max", pitch_max) ||
    !getInput("yaw_min", yaw_min) || !getInput("yaw_max", yaw_max)) {
    return false;
  }

  msg.velocity.pitch = vel_pitch;
  msg.velocity.yaw = vel_yaw;
  msg.velocity.pitch_min_range = pitch_min;
  msg.velocity.pitch_max_range = pitch_max;
  msg.velocity.yaw_min_range = yaw_min;
  msg.velocity.yaw_max_range = yaw_max;

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
CreateRosNodePlugin(pb2025_sentry_behavior::PublishGimbalVelocity, "PublishGimbalVelocity");