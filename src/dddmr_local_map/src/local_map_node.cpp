#include <Eigen/Core>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "dddmr_local_map/ros_adapter.hpp"

#ifndef DDDMR_LOCAL_MAP_ENTRY_NAME
#define DDDMR_LOCAL_MAP_ENTRY_NAME "local_map_node"
#endif

namespace {

using dddmr_local_map::Occupancy;
using dddmr_local_map::ros::AdapterConfig;
using dddmr_local_map::ros::AdapterHandle;
using dddmr_local_map::ros::LocalMapRosAdapter;

sensor_msgs::msg::PointCloud2 makePointCloud(
    const std::string& frame, const rclcpp::Time& stamp,
    const std::vector<Eigen::Vector3f>& points) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame;
  cloud.header.stamp = stamp;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  sensor_msgs::msg::PointField field;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  field.name = "x";
  field.offset = 0;
  cloud.fields.push_back(field);
  field.name = "y";
  field.offset = 4;
  cloud.fields.push_back(field);
  field.name = "z";
  field.offset = 8;
  cloud.fields.push_back(field);
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
  for (std::size_t index = 0; index < points.size(); ++index) {
    const float x = points[index].x();
    const float y = points[index].y();
    const float z = points[index].z();
    unsigned char* base = cloud.data.data() + index * cloud.point_step;
    std::memcpy(base, &x, sizeof(float));
    std::memcpy(base + 4, &y, sizeof(float));
    std::memcpy(base + 8, &z, sizeof(float));
  }
  return cloud;
}

void visualize(const std::shared_ptr<LocalMapRosAdapter>& adapter,
               const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr&
                   occupancy_publisher,
               const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr&
                   surface_publisher) {
  const bool want_occupancy =
      occupancy_publisher->get_subscription_count() > 0U;
  const bool want_surface = surface_publisher->get_subscription_count() > 0U;
  if (!want_occupancy && !want_surface) {
    return;
  }
  const std::shared_ptr<const AdapterHandle> handle = adapter->handle();
  if (handle == nullptr || !handle->mapHealthy() || handle->snapshot == nullptr) {
    // Publish an empty transient-local sample so a previously latched
    // diagnostic cannot look current after the map becomes unhealthy/empty.
    const std::string frame = handle != nullptr && handle->snapshot != nullptr
                                  ? handle->snapshot->frame()
                                  : adapter->config().map_frame;
    const rclcpp::Time stamp = handle != nullptr
                                   ? handle->measurement_stamp
                                   : rclcpp::Time(0LL, RCL_ROS_TIME);
    const std::vector<Eigen::Vector3f> empty;
    if (want_occupancy) {
      occupancy_publisher->publish(makePointCloud(frame, stamp, empty));
    }
    if (want_surface) {
      surface_publisher->publish(makePointCloud(frame, stamp, empty));
    }
    return;
  }
  const auto snapshot = handle->snapshot;
  const auto dims = snapshot->windowDims();
  const auto origin = snapshot->originVoxel();
  const double voxel = snapshot->config().voxel_size;
  std::vector<Eigen::Vector3f> occupancy_points;
  std::vector<Eigen::Vector3f> surface_candidates;
  occupancy_points.reserve(static_cast<std::size_t>(
      std::min<std::size_t>(snapshot->occupiedCount(), 200000U)));
  surface_candidates.reserve(static_cast<std::size_t>(dims.x()) *
                             static_cast<std::size_t>(dims.y()));

  const auto append = [&origin, voxel](std::vector<Eigen::Vector3f>* output,
                                       int ix, int iy, int iz) {
    output->emplace_back(
        static_cast<float>((static_cast<double>(origin[0]) + ix + 0.5) * voxel),
        static_cast<float>((static_cast<double>(origin[1]) + iy + 0.5) * voxel),
        static_cast<float>((static_cast<double>(origin[2]) + iz + 0.5) *
                           voxel));
  };

  for (int ix = 0; ix < dims.x(); ++ix) {
    for (int iy = 0; iy < dims.y(); ++iy) {
      bool in_occupied_run = false;
      int run_top = -1;
      for (int iz = 0; iz < dims.z(); ++iz) {
        const auto lookup = snapshot->lookupVoxel(
            origin[0] + ix, origin[1] + iy, origin[2] + iz, false);
        const bool occupied = lookup.occupancy == Occupancy::kOccupied;
        if (occupied) {
          in_occupied_run = true;
          run_top = iz;
          if (want_occupancy) {
            append(&occupancy_points, ix, iy, iz);
          }
        } else if (in_occupied_run) {
          // A raw surface candidate is the top of an occupied run, not the
          // global lowest occupied cell in the column.
          if (want_surface) {
            append(&surface_candidates, ix, iy, run_top);
          }
          in_occupied_run = false;
        }
      }
      if (want_surface && in_occupied_run) {
        append(&surface_candidates, ix, iy, run_top);
      }
    }
  }

  if (want_occupancy) {
    occupancy_publisher->publish(
        makePointCloud(snapshot->frame(), handle->measurement_stamp,
                       occupancy_points));
  }
  if (want_surface) {
    surface_publisher->publish(makePointCloud(
        snapshot->frame(), handle->measurement_stamp, surface_candidates));
  }
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(DDDMR_LOCAL_MAP_ENTRY_NAME);

  AdapterConfig config;
  const auto config_status =
      dddmr_local_map::ros::loadAdapterConfig(*node, "local_map.", &config);
  if (!config_status.ok) {
    RCLCPP_FATAL(node->get_logger(), "invalid local_map parameters: %s",
                 config_status.error.c_str());
    rclcpp::shutdown();
    return 1;
  }

  // The shadow process owns its TF listener and feeds this buffer. The adapter
  // looks up map<-sensor and map<-base at every cloud acquisition stamp.
  tf2_ros::Buffer tf_buffer(node->get_clock());
  // Timed acquisition lookups need a thread that can continue receiving TF
  // while the cloud callback waits. The default listener owns that thread.
  tf2_ros::TransformListener tf_listener(tf_buffer);

  std::shared_ptr<LocalMapRosAdapter> adapter;
  try {
    adapter = std::make_shared<LocalMapRosAdapter>(*node, tf_buffer, config);
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(node->get_logger(), "failed to start adapter: %s",
                 exception.what());
    rclcpp::shutdown();
    return 1;
  }

  // Diagnostics only. There is deliberately no command publisher here.
  auto occupancy_publisher =
      node->create_publisher<sensor_msgs::msg::PointCloud2>(
          "local_map/occupancy", rclcpp::QoS(1).transient_local());
  auto surface_publisher =
      node->create_publisher<sensor_msgs::msg::PointCloud2>(
          "local_map/surface_candidates", rclcpp::QoS(1).transient_local());

  auto timer = node->create_wall_timer(std::chrono::milliseconds(200),
                                       [adapter, occupancy_publisher,
                                        surface_publisher]() {
                                         visualize(adapter, occupancy_publisher,
                                                   surface_publisher);
                                       });

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
