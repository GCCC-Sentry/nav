// Copyright 2025 Lihan Chen
// Licensed under the Apache License, Version 2.0

#include "pb2025_sentry_behavior/plugins/action/pub_stance.hpp"

namespace pb2025_sentry_behavior
{

PublishStanceAction::PublishStanceAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosTopicPubNode<example_interfaces::msg::Float32>(name, config, params)
{
}

BT::PortsList PublishStanceAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("stance_id", 0.0,
      "Stance ID: 0=movement, 1=attack, 2=defense"),
  });
}

bool PublishStanceAction::setMessage(example_interfaces::msg::Float32 & msg)
{
  double stance_id;
  if (!getInput("stance_id", stance_id)) {
    return false;
  }
  msg.data = static_cast<float>(stance_id);
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishStanceAction, "PublishStance");
