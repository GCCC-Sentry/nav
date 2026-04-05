#include "pursuit_decision/zone_manager.hpp"

#include <cmath>

namespace pursuit_decision
{

void ZoneManager::configure(
  const std::vector<Zone> & zones,
  double z_layer_tolerance)
{
  zones_ = zones;
  z_layer_tolerance_ = z_layer_tolerance;
}

int ZoneManager::get_layer(double x, double y, double z) const
{
  for (const auto & zone : zones_) {
    if (point_in_polygon(x, y, zone.polygon)) {
      // 检查高度是否匹配该区域的高度范围
      if (z >= (zone.z_min - z_layer_tolerance_) &&
          z <= (zone.z_max + z_layer_tolerance_))
      {
        return static_cast<int>(zone.layer_id);
      }
    }
  }
  return -1;  // 未知区域
}

bool ZoneManager::is_in_forbidden_zone(double x, double y) const
{
  for (const auto & zone : zones_) {
    if (zone.is_forbidden && point_in_polygon(x, y, zone.polygon)) {
      return true;
    }
  }
  return false;
}

std::string ZoneManager::get_zone_name(double x, double y) const
{
  for (const auto & zone : zones_) {
    if (point_in_polygon(x, y, zone.polygon)) {
      return zone.name;
    }
  }
  return "unknown";
}

bool ZoneManager::is_same_layer(
  double x1, double y1, double z1,
  double x2, double y2, double z2) const
{
  int layer1 = get_layer(x1, y1, z1);
  int layer2 = get_layer(x2, y2, z2);

  // 如果任一位置在未知区域，保守地认为不在同一图层
  if (layer1 < 0 || layer2 < 0) return false;

  return layer1 == layer2;
}

bool ZoneManager::point_in_polygon(
  double x, double y,
  const std::vector<std::pair<double, double>> & polygon) const
{
  // 射线法 (Ray Casting Algorithm)
  int n = static_cast<int>(polygon.size());
  if (n < 3) return false;

  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    double xi = polygon[i].first, yi = polygon[i].second;
    double xj = polygon[j].first, yj = polygon[j].second;

    if (((yi > y) != (yj > y)) &&
        (x < (xj - xi) * (y - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
  }
  return inside;
}

}  // namespace pursuit_decision
