#include <gtest/gtest.h>

#include <perception_3d/ground_surface_filter.h>

#include <cmath>
#include <map>
#include <utility>

namespace {

pcl::PointXYZI Point(float x, float y, float z)
{
  pcl::PointXYZI point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = 1.0f;
  return point;
}

std::map<std::pair<int, int>, int> CountCells(
  const pcl::PointCloud<pcl::PointXYZI>& cloud,
  double resolution)
{
  std::map<std::pair<int, int>, int> counts;
  for(const auto& point : cloud.points){
    const int cell_x = static_cast<int>(std::floor(point.x / resolution));
    const int cell_y = static_cast<int>(std::floor(point.y / resolution));
    counts[{cell_x, cell_y}]++;
  }
  return counts;
}

}  // namespace

TEST(GroundSurfaceFilter, KeepsASlopedSurfaceAsOneLayerPerCell)
{
  auto input = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  input->push_back(Point(0.01f, 0.01f, 0.00f));
  input->push_back(Point(0.11f, 0.01f, 0.04f));
  input->push_back(Point(0.21f, 0.01f, 0.08f));
  input->push_back(Point(0.31f, 0.01f, 0.12f));

  input->push_back(Point(0.01f, 0.02f, 1.00f));
  input->push_back(Point(0.11f, 0.02f, 1.04f));
  input->push_back(Point(0.21f, 0.02f, 1.08f));

  perception_3d::GroundSurfaceFilterConfig config;
  config.merge_ground_layers = true;
  config.merge_ground_xy_resolution = 0.1;
  config.merge_ground_z_tolerance = 0.02;
  config.force_single_ground_surface = true;
  config.single_surface_max_slope_deg = 30.0;
  config.single_surface_z_margin = 0.02;
  config.single_surface_select_policy = "largest_connected";

  const auto output = perception_3d::FilterGroundSurface(input, config);

  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->points.size(), 4u);
  for(const auto& cell_count : CountCells(*output, config.merge_ground_xy_resolution)){
    EXPECT_EQ(cell_count.second, 1);
  }
  for(const auto& point : output->points){
    EXPECT_LT(point.z, 0.5f);
  }
}

TEST(GroundSurfaceFilter, PreservesLayerMergingWhenSingleSurfaceIsDisabled)
{
  auto input = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  input->push_back(Point(0.01f, 0.01f, 0.00f));
  input->push_back(Point(0.02f, 0.02f, 0.01f));
  input->push_back(Point(0.03f, 0.03f, 1.00f));

  perception_3d::GroundSurfaceFilterConfig config;
  config.merge_ground_layers = true;
  config.merge_ground_xy_resolution = 0.1;
  config.merge_ground_z_tolerance = 0.03;
  config.force_single_ground_surface = false;

  const auto output = perception_3d::FilterGroundSurface(input, config);

  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->points.size(), 2u);
}

TEST(GroundSurfaceFilter, KeepsDisconnectedFlatLandingCellsWhenUsingLargestConnected)
{
  auto input = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  input->push_back(Point(0.01f, 0.01f, 0.00f));
  input->push_back(Point(0.11f, 0.01f, 0.03f));
  input->push_back(Point(0.21f, 0.01f, 0.06f));
  input->push_back(Point(0.31f, 0.01f, 0.09f));

  input->push_back(Point(1.01f, 0.01f, 0.50f));
  input->push_back(Point(1.11f, 0.01f, 0.50f));
  input->push_back(Point(1.21f, 0.01f, 0.50f));

  perception_3d::GroundSurfaceFilterConfig config;
  config.merge_ground_layers = true;
  config.merge_ground_xy_resolution = 0.1;
  config.merge_ground_z_tolerance = 0.02;
  config.force_single_ground_surface = true;
  config.single_surface_max_slope_deg = 20.0;
  config.single_surface_z_margin = 0.02;
  config.single_surface_select_policy = "largest_connected";

  const auto output = perception_3d::FilterGroundSurface(input, config);

  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->points.size(), 7u);
  for(const auto& cell_count : CountCells(*output, config.merge_ground_xy_resolution)){
    EXPECT_EQ(cell_count.second, 1);
  }
}
