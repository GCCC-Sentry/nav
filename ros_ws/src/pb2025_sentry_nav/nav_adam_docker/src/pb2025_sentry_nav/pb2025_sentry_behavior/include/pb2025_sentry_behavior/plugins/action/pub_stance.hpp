// include/pb2025_sentry_behavior/plugins/action/pub_stance.hpp

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_STANCE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_STANCE_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "example_interfaces/msg/float32.hpp"

namespace pb2025_sentry_behavior
{

// Publishes stance command to cmd_stance topic
// stance_id: 0 = movement, 1 = attack, 2 = defense
class PublishStanceAction
: public BT::RosTopicPubNode<example_interfaces::msg::Float32>
{
public:
  PublishStanceAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(example_interfaces::msg::Float32 & msg) override;
};

}  // namespace pb2025_sentry_behavior

#endif
