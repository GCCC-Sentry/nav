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

#include "pb2025_sentry_behavior/plugins/condition/is_game_status.hpp"
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace pb2025_sentry_behavior
{

// 全局日志文件
static std::ofstream game_log_file;
static std::string game_log_path = "/root/ros_ws/src/game_status_log.txt";
static auto last_game_print_time = std::chrono::steady_clock::now();

// 获取当前时间字符串
std::string getCurrentTimeString() {
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
  ss << "." << std::setfill('0') << std::setw(3) << ms.count();
  return ss.str();
}

// 写比赛状态日志
void logGameStatus(const std::string& message) {
  if (!game_log_file.is_open()) {
    game_log_file.open(game_log_path, std::ios::app);
    if (game_log_file.is_open()) {
      game_log_file << "\n\n========================================\n";
      game_log_file << "比赛状态日志启动时间: " << getCurrentTimeString() << "\n";
      game_log_file << "========================================\n\n";
    }
  }
  
  if (game_log_file.is_open()) {
    game_log_file << "[" << getCurrentTimeString() << "] " << message << std::endl;
    game_log_file.flush();
  }
}

// 比赛阶段名称
const char* getGameProgressName(int progress) {
  switch(progress) {
    case 0: return "未开始";
    case 1: return "准备阶段";
    case 2: return "自检阶段";
    case 3: return "5秒倒计时";
    case 4: return "比赛中";
    case 5: return "比赛结算";
    default: return "未知阶段";
  }
}

IsGameStatusCondition::IsGameStatusCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsGameStatusCondition::checkGameStart, this), config)
{
  RCLCPP_INFO(logger_, "【条件节点】IsGameStatus 初始化完成");
  logGameStatus("【条件节点】IsGameStatus 初始化完成");
}

BT::NodeStatus IsGameStatusCondition::checkGameStart()
{
  // 每次检查都打印（配合行为树 2s Sleep 间隔，不会刷屏）
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_game_print_time).count();
  bool should_print = (elapsed >= 1500);
  
  int expected_game_progress, min_remain_time, max_remain_time;
  auto msg = getInput<pb_rm_interfaces::msg::GameStatus>("key_port");
  
  if (!msg) {
    std::string error_msg = "❌ 错误：无法获取比赛状态数据（GameStatus消息不可用）";
    RCLCPP_ERROR(logger_, "%s", error_msg.c_str());
    logGameStatus(error_msg);
    return BT::NodeStatus::FAILURE;
  }

  getInput("expected_game_progress", expected_game_progress);
  getInput("min_remain_time", min_remain_time);
  getInput("max_remain_time", max_remain_time);

  const bool is_progress_match = (msg->game_progress == expected_game_progress);
  const bool is_time_in_range =
    (msg->stage_remain_time >= min_remain_time) && (msg->stage_remain_time <= max_remain_time);
  const bool game_ready = (is_progress_match && is_time_in_range);

  if (should_print) {
    std::stringstream ss;
    ss << "\n╔════════════════════════════════════════════════════════╗";
    ss << "\n║  【比赛状态】比赛进度检查                            ║";
    ss << "\n╠════════════════════════════════════════════════════════╣";
    ss << "\n║  当前阶段: " << static_cast<int>(msg->game_progress) 
       << " (" << getGameProgressName(msg->game_progress) << ")";
    while (ss.str().substr(ss.str().rfind('\n') + 1).length() < 56) ss << " ";
    ss << "║";
    
    ss << "\n║  期望阶段: " << expected_game_progress 
       << " (" << getGameProgressName(expected_game_progress) << ")";
    while (ss.str().substr(ss.str().rfind('\n') + 1).length() < 56) ss << " ";
    ss << "║";
    
    ss << "\n║    匹配状态: " << (is_progress_match ? "✓ 匹配" : "❌ 不匹配");
    while (ss.str().substr(ss.str().rfind('\n') + 1).length() < 56) ss << " ";
    ss << "║";
    
    ss << "\n║  剩余时间: " << msg->stage_remain_time << "秒";
    while (ss.str().substr(ss.str().rfind('\n') + 1).length() < 56) ss << " ";
    ss << "║";
    
    ss << "\n║  时间范围: [" << min_remain_time << "s - " << max_remain_time << "s]";
    while (ss.str().substr(ss.str().rfind('\n') + 1).length() < 56) ss << " ";
    ss << "║";
    
    ss << "\n║    时间状态: " << (is_time_in_range ? "✓ 在范围内" : "❌ 超出范围");
    while (ss.str().substr(ss.str().rfind('\n') + 1).length() < 56) ss << " ";
    ss << "║";
    
    ss << "\n╠════════════════════════════════════════════════════════╣";
    ss << "\n║  综合判断: " << (game_ready ? "✓✓✓ 比赛已开始，可以执行任务" : "❌ 比赛未开始或已结束");
    while (ss.str().substr(ss.str().rfind('\n') + 1).length() < 56) ss << " ";
    ss << "║";
    ss << "\n╚════════════════════════════════════════════════════════╝";
    
    RCLCPP_INFO(logger_, "%s", ss.str().c_str());
    logGameStatus(ss.str());
    
    last_game_print_time = now;
  }

  return game_ready ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsGameStatusCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::GameStatus>(
      "key_port", "{@referee_gameStatus}", "GameStatus port on blackboard"),
    BT::InputPort<int>("expected_game_progress", 4, "Expected game progress stage"),
    BT::InputPort<int>("min_remain_time", 0, "Minimum remaining time (s)"),
    BT::InputPort<int>("max_remain_time", 420, "Maximum remaining time (s)"),
  };
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsGameStatusCondition>("IsGameStatus");
}
