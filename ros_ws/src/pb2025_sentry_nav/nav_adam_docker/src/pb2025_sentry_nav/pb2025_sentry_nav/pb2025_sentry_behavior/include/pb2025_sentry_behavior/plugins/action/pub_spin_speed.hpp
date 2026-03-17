// include/pb2025_sentry_behavior/plugins/action/pub_spin_speed.hpp

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SPIN_SPEED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SPIN_SPEED_HPP_

#include <string>

// 确保引用的是正确的头文件 (之前您可能已经改过了)
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace pb2025_sentry_behavior
{

// 1. 基类改为 BT::RosTopicPubNode
class PublishSpinSpeedAction
: public BT::RosTopicPubNode<example_interfaces::msg::Float32>
{
public:
  PublishSpinSpeedAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(example_interfaces::msg::Float32 & msg) override;

  // 2. 删除 setHaltMessage 声明
  // bool setHaltMessage(example_interfaces::msg::Float32 & msg) override; 
};

}  // namespace pb2025_sentry_behavior

#endif