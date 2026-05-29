#include <gtest/gtest.h>

#include <perception_3d/ground_surface_filter.h>

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

}  // namespace

TEST(GroundSurfaceFilter, MergesNearbyGroundPointsWithinEachCell)
{
  auto input = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  input->push_back(Point(0.01f, 0.01f, 0.00f));
  input->push_back(Point(0.02f, 0.02f, 0.01f));
  input->push_back(Point(0.03f, 0.03f, 1.00f));

  perception_3d::GroundSurfaceFilterConfig config;
  config.merge_ground_layers = true;
  config.merge_ground_xy_resolution = 0.1;
  config.merge_ground_z_tolerance = 0.03;

  const auto output = perception_3d::FilterGroundSurface(input, config);

  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->points.size(), 2u);
}
