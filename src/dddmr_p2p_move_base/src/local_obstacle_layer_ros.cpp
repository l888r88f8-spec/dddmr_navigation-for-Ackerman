#include "p2p_move_base/local_obstacle_layer_ros.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace p2p_move_base
{
namespace
{
const sensor_msgs::msg::PointField * findField(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name)
{
  const auto iterator = std::find_if(
    cloud.fields.begin(),
    cloud.fields.end(),
    [&name](const auto & field) { return field.name == name; });
  return iterator == cloud.fields.end() ? nullptr : &(*iterator);
}

std::size_t fieldSize(const sensor_msgs::msg::PointField & field)
{
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32:
      return sizeof(float);
    case sensor_msgs::msg::PointField::FLOAT64:
      return sizeof(double);
    default:
      return 0;
  }
}

bool readFieldAsDouble(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const sensor_msgs::msg::PointField & field,
  const std::size_t point_offset,
  double * value)
{
  const std::size_t size = fieldSize(field);
  if (size == 0 || field.count == 0 || point_offset + field.offset + size > cloud.data.size()) {
    return false;
  }

  const auto * field_data = cloud.data.data() + point_offset + field.offset;
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    float field_value = 0.0F;
    std::memcpy(&field_value, field_data, sizeof(field_value));
    *value = static_cast<double>(field_value);
    return true;
  }

  double field_value = 0.0;
  std::memcpy(&field_value, field_data, sizeof(field_value));
  *value = field_value;
  return true;
}
}  // namespace

std::vector<LocalPoint> pointCloud2ToLocalPoints(const sensor_msgs::msg::PointCloud2 & cloud)
{
  const auto * x_field = findField(cloud, "x");
  const auto * y_field = findField(cloud, "y");
  const auto * z_field = findField(cloud, "z");
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr || cloud.point_step == 0) {
    return {};
  }

  const std::size_t point_count = static_cast<std::size_t>(cloud.width) *
    static_cast<std::size_t>(cloud.height);
  std::vector<LocalPoint> points;
  points.reserve(point_count);

  for (std::size_t index = 0; index < point_count; ++index) {
    const std::size_t point_offset = index * static_cast<std::size_t>(cloud.point_step);
    if (point_offset >= cloud.data.size()) {
      break;
    }

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!readFieldAsDouble(cloud, *x_field, point_offset, &x) ||
      !readFieldAsDouble(cloud, *y_field, point_offset, &y) ||
      !readFieldAsDouble(cloud, *z_field, point_offset, &z))
    {
      continue;
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    points.push_back({x, y, z});
  }

  return points;
}

std_msgs::msg::String snapshotToStatusMessage(const LocalObstacleSnapshot & snapshot)
{
  std_msgs::msg::String status;
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "{\"frame_id\":\"" << snapshot.frame_id << "\""
         << ",\"raw_points\":" << snapshot.raw_points
         << ",\"filtered_points\":" << snapshot.filtered_points
         << ",\"obstacle_points\":" << snapshot.obstacle_points.size()
         << ",\"perception_stale\":" << (snapshot.perception_stale ? "true" : "false")
         << ",\"over_budget\":" << (snapshot.over_budget ? "true" : "false")
         << ",\"update_time_ms\":" << snapshot.update_time_ms
         << "}";
  status.data = stream.str();
  return status;
}

sensor_msgs::msg::PointCloud2 snapshotToPointCloud2(
  const LocalObstacleSnapshot & snapshot,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(snapshot.obstacle_points.size());
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(snapshot.obstacle_points.size());

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (const auto & point : snapshot.obstacle_points) {
    *x = static_cast<float>(point.x);
    *y = static_cast<float>(point.y);
    *z = static_cast<float>(point.z);
    ++x;
    ++y;
    ++z;
  }
  return cloud;
}

}  // namespace p2p_move_base
