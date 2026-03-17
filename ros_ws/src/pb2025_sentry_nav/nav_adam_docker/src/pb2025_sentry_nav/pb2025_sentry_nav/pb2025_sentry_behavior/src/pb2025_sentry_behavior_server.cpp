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

#include "pb2025_sentry_behavior/pb2025_sentry_behavior_server.hpp"

#include <filesystem>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "pb2025_sentry_behavior/custom_types.hpp"

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "behaviortree_cpp/xml_parsing.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"

namespace pb2025_sentry_behavior
{

// 全局日志文件
static std::ofstream status_log_file;
static std::string status_log_path = "/root/ros_ws/src/robot_status_log.txt";

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

// 写状态日志
void logStatus(const std::string& message) {
  if (!status_log_file.is_open()) {
    status_log_file.open(status_log_path, std::ios::app);
    if (status_log_file.is_open()) {
      status_log_file << "\n\n========================================\n";
      status_log_file << "机器人状态日志启动时间: " << getCurrentTimeString() << "\n";
      status_log_file << "========================================\n\n";
    }
  }
  
  if (status_log_file.is_open()) {
    status_log_file << "[" << getCurrentTimeString() << "] " << message << std::endl;
    status_log_file.flush();
  }
}

template <typename T>
void SentryBehaviorServer::subscribe(
  const std::string & topic, const std::string & bb_key, const rclcpp::QoS & qos)
{
  auto sub = node()->create_subscription<T>(
    topic, qos,
    [this, bb_key, topic](const typename T::SharedPtr msg) { 
      globalBlackboard()->set(bb_key, *msg);
      
      // 简单记录接收到的数据
      std::stringstream ss;
      ss << "【数据接收】话题: " << topic << ", 黑板键: " << bb_key;
      logStatus(ss.str());
    });
  subscriptions_.push_back(sub);
}

SentryBehaviorServer::SentryBehaviorServer(const rclcpp::NodeOptions & options)
: TreeExecutionServer(options)
{
  RCLCPP_INFO(node()->get_logger(), "========================================");
  RCLCPP_INFO(node()->get_logger(), "哨兵行为树服务器启动中...");
  RCLCPP_INFO(node()->get_logger(), "========================================");
  
  logStatus("========================================");
  logStatus("哨兵行为树服务器启动");
  logStatus("========================================");
  
  node()->declare_parameter("use_cout_logger", false);
  node()->get_parameter("use_cout_logger", use_cout_logger_);

  subscribe<pb_rm_interfaces::msg::EventData>("referee/event_data", "referee_eventData");
  subscribe<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", "referee_allRobotHP");
  subscribe<pb_rm_interfaces::msg::GameStatus>("referee/game_status", "referee_gameStatus");
  subscribe<pb_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", "referee_groundRobotPosition");
  subscribe<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", "referee_rfidStatus");
  subscribe<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", "referee_robotStatus");
  subscribe<pb_rm_interfaces::msg::Buff>("referee/buff", "referee_buff");

  auto detector_qos = rclcpp::SensorDataQoS();
  subscribe<auto_aim_interfaces::msg::Armors>("detector/armors", "detector_armors", detector_qos);
  auto tracker_qos = rclcpp::SensorDataQoS();
  subscribe<auto_aim_interfaces::msg::Target>("tracker/target", "tracker_target", tracker_qos);

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  subscribe<nav_msgs::msg::OccupancyGrid>(
    "global_costmap/costmap", "nav_globalCostmap", costmap_qos);
  
  RCLCPP_INFO(node()->get_logger(), "✓ 所有话题订阅完成");
  logStatus("✓ 所有话题订阅完成");
}

bool SentryBehaviorServer::onGoalReceived(
  const std::string & tree_name, const std::string & payload)
{
  std::stringstream ss;
  ss << "\n╔════════════════════════════════════════════════════════╗";
  ss << "\n║  【决策】收到执行行为树的目标请求                    ║";
  ss << "\n║  行为树名称: " << tree_name;
  while (ss.str().substr(ss.str().rfind('\n') + 1).length() < 56) ss << " ";
  ss << "║";
  ss << "\n╚════════════════════════════════════════════════════════╝";
  
  RCLCPP_INFO(node()->get_logger(), "%s", ss.str().c_str());
  logStatus(ss.str());
  
  return true;
}

void SentryBehaviorServer::onTreeCreated(BT::Tree & tree)
{
  std::string msg = "【决策】行为树创建成功，准备开始执行";
  RCLCPP_INFO(node()->get_logger(), "%s", msg.c_str());
  logStatus(msg);
  
  if (use_cout_logger_) {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }
  tick_count_ = 0;
}

std::optional<BT::NodeStatus> SentryBehaviorServer::onLoopAfterTick(BT::NodeStatus /*status*/)
{
  ++tick_count_;
  
  // 每50次tick记录一次
  if (tick_count_ % 50 == 0) {
    std::stringstream ss;
    ss << "【决策】行为树执行中... (已执行 " << tick_count_ << " 次tick)";
    logStatus(ss.str());
  }
  
  return std::nullopt;
}

std::optional<std::string> SentryBehaviorServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled)
{
  std::stringstream ss;
  ss << "\n╔════════════════════════════════════════════════════════╗";
  ss << "\n║  【决策】行为树执行完成                              ║";
  ss << "\n║  最终状态: " << (status == BT::NodeStatus::SUCCESS ? "成功" : 
                              status == BT::NodeStatus::FAILURE ? "失败" : "运行中");
  while (ss.str().substr(ss.str().rfind('\n') + 1).length() < 56) ss << " ";
  ss << "║";
  ss << "\n║  总执行次数: " << tick_count_ << " 次tick";
  while (ss.str().substr(ss.str().rfind('\n') + 1).length() < 56) ss << " ";
  ss << "║";
  ss << "\n╚════════════════════════════════════════════════════════╝";
  
  RCLCPP_INFO(node()->get_logger(), "%s", ss.str().c_str());
  logStatus(ss.str());
  
  logger_cout_.reset();
  
  std::string result = treeName() +
                       " 行为树完成，状态=" + std::to_string(static_cast<int>(status)) +
                       "，执行了 " + std::to_string(tick_count_) + " 次tick";
  return result;
}

}  // namespace pb2025_sentry_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<pb2025_sentry_behavior::SentryBehaviorServer>(options);

  RCLCPP_INFO(action_server->node()->get_logger(), "启动哨兵行为树服务器");
  
  // Force instantiation of PoseStamped type conversion
  try {
    auto test_pose = BT::convertFromString<geometry_msgs::msg::PoseStamped>("0;0;0");
    (void)test_pose;
  } catch (...) {}


  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  // Groot2 editor requires a model of your registered Nodes.
  // You don't need to write that by hand, it can be automatically
  // generated using the following command.
  std::string xml_models = BT::writeTreeNodesModelXML(action_server->factory());

  // Save the XML models to a file
  std::ofstream file(std::filesystem::path(ROOT_DIR) / "behavior_trees" / "models.xml");
  file << xml_models;

  rclcpp::shutdown();
}
