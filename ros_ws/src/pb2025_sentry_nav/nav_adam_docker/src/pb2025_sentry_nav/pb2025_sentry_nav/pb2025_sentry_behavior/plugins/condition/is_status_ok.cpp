// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb2025_sentry_behavior/plugins/condition/is_status_ok.hpp"
#include <chrono>
#include <sstream>

namespace pb2025_sentry_behavior
{

IsStatusOKCondition::IsStatusOKCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsStatusOKCondition::checkRobotStatus, this), config)
{
  RCLCPP_INFO(logger_, "【条件节点】IsStatusOK 初始化完成");
}

BT::NodeStatus IsStatusOKCondition::checkRobotStatus()
{
  int hp_min, heat_max, ammo_min;
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");

  if (!msg) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 3000,
      "⚠ IsStatusOK: 未收到机器人状态数据，返回 FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  getInput("hp_min", hp_min);
  getInput("heat_max", heat_max);
  getInput("ammo_min", ammo_min);

  const bool is_alive = (msg->current_hp > 0);
  const bool is_hp_ok = (msg->current_hp >= hp_min);
  const bool is_heat_ok = (msg->shooter_17mm_1_barrel_heat <= heat_max);
  const bool is_ammo_ok = (msg->projectile_allowance_17mm >= ammo_min);
  const bool all_ok = (is_alive && is_hp_ok && is_heat_ok && is_ammo_ok);

  // 每个实例使用自己的计时器 (成员变量), 每 3 秒打印一次
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time_).count();
  if (elapsed >= 3000) {
    last_print_time_ = now;

    // 简洁一行日志: [检查名] HP=130/400(>=200) 热量=0(<=999999) 弹药=750(>=0) → SUCCESS/FAILURE
    std::stringstream ss;
    ss << "IsStatusOK[hp>=" << hp_min << "]: "
       << "HP=" << static_cast<int>(msg->current_hp) << "/" << static_cast<int>(msg->maximum_hp)
       << (is_hp_ok ? " ✓" : " ✗")
       << " | 热量=" << msg->shooter_17mm_1_barrel_heat << "<=" << heat_max
       << (is_heat_ok ? " ✓" : " ✗")
       << " | 弹药=" << msg->projectile_allowance_17mm << ">=" << ammo_min
       << (is_ammo_ok ? " ✓" : " ✗")
       << " → " << (all_ok ? "SUCCESS(继续)" : "FAILURE(需回家)");

    if (all_ok) {
      RCLCPP_INFO(logger_, "%s", ss.str().c_str());
    } else {
      RCLCPP_WARN(logger_, "%s", ss.str().c_str());
    }
  }

  return all_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsStatusOKCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::InputPort<int>("hp_min", 300, "Minimum HP. NOTE: Sentry init/max HP is 400"),
    BT::InputPort<int>("heat_max", 350, "Maximum heat. NOTE: Sentry heat limit is 400"),
    BT::InputPort<int>("ammo_min", 0, "Lower then minimum ammo will return FAILURE")};
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsStatusOKCondition>("IsStatusOK");
}
