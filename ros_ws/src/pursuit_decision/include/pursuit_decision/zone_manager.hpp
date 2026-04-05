#ifndef PURSUIT_DECISION__ZONE_MANAGER_HPP
#define PURSUIT_DECISION__ZONE_MANAGER_HPP

#include <string>
#include <vector>

namespace pursuit_decision
{

/// 地形图层ID
enum class LayerId : int {
  OWN_HALF = 0,       // 己方半场
  ENEMY_HALF = 1,     // 敌方半场
  CENTRAL_HIGHLAND = 2 // 中央高地
};

/// 多边形区域定义
struct Zone {
  std::string name;
  std::vector<std::pair<double, double>> polygon;  // (x,y) 顶点
  double z_min;         // 最小高度(map frame)
  double z_max;         // 最大高度(map frame)
  bool is_forbidden;    // 是否为禁区
  LayerId layer_id;     // 所属图层
};

class ZoneManager
{
public:
  ZoneManager() = default;

  /// 从参数初始化区域定义
  void configure(
    const std::vector<Zone> & zones,
    double z_layer_tolerance);

  /// 判断点所在的图层 (-1 = 未知)
  int get_layer(double x, double y, double z) const;

  /// 判断点是否在禁区内
  bool is_in_forbidden_zone(double x, double y) const;

  /// 获取点所在区域名称
  std::string get_zone_name(double x, double y) const;

  /// 判断两个位置是否在同一图层
  bool is_same_layer(
    double x1, double y1, double z1,
    double x2, double y2, double z2) const;

private:
  std::vector<Zone> zones_;
  double z_layer_tolerance_{0.3};

  /// 射线法判断点是否在多边形内
  bool point_in_polygon(
    double x, double y,
    const std::vector<std::pair<double, double>> & polygon) const;
};

}  // namespace pursuit_decision

#endif  // PURSUIT_DECISION__ZONE_MANAGER_HPP
