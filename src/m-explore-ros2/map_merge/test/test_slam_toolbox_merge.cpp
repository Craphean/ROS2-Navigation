#include <combine_grids/merging_pipeline.h>
#include <gtest/gtest.h>
#include <cmath>
#include <vector>

namespace
{
nav_msgs::msg::OccupancyGrid::SharedPtr makeGrid(
    uint32_t width, uint32_t height, float resolution,
    double origin_x, double origin_y)
{
  auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  grid->info.width = width;
  grid->info.height = height;
  grid->info.resolution = resolution;
  grid->info.origin.position.x = origin_x;
  grid->info.origin.position.y = origin_y;
  grid->info.origin.position.z = 0.0;
  grid->info.origin.orientation.w = 1.0;
  grid->data.assign(static_cast<size_t>(width) * height, -1);
  return grid;
}

geometry_msgs::msg::Transform makePose2D(double x, double y, double yaw)
{
  geometry_msgs::msg::Transform t;
  t.translation.x = x;
  t.translation.y = y;
  t.translation.z = 0.0;
  t.rotation.x = 0.0;
  t.rotation.y = 0.0;
  t.rotation.z = std::sin(yaw * 0.5);
  t.rotation.w = std::cos(yaw * 0.5);
  return t;
}
}  // namespace

TEST(SlamToolboxMerge, AlignsDifferentSizedGridsWithDynamicOrigins)
{
  constexpr float res = 0.05f;
  // Robot 1 spawned at (-2.0, -0.5, 0.0), grid 120x100 with origin (-1.0, -1.0)
  auto grid1 = makeGrid(120, 100, res, -1.0, -1.0);
  // Robot 2 spawned at (0.0, 0.5, 0.0), grid 90x130 with origin (-2.5, -2.0)
  auto grid2 = makeGrid(90, 130, res, -2.5, -2.0);

  // Place a shared world obstacle at world (1.0, 1.0):
  // In robot1 map frame: (1.0 - (-2.0), 1.0 - (-0.5)) = (3.0, 1.5)
  // Cell in grid1: ((3.0 - (-1.0))/0.05, (1.5 - (-1.0))/0.05) = (80, 50)
  grid1->data[50 * grid1->info.width + 80] = 100;

  // In robot2 map frame: (1.0 - 0.0, 1.0 - 0.5) = (1.0, 0.5)
  // Cell in grid2: ((1.0 - (-2.5))/0.05, (0.5 - (-2.0))/0.05) = (70, 50)
  grid2->data[50 * grid2->info.width + 70] = 100;

  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> grids{grid1, grid2};
  std::vector<geometry_msgs::msg::Transform> poses{
      makePose2D(-2.0, -0.5, 0.0),
      makePose2D(0.0, 0.5, 0.0)};

  combine_grids::MergingPipeline merger;
  merger.feed(grids.begin(), grids.end());
  ASSERT_TRUE(merger.setTransforms(poses.begin(), poses.end(), true));
  auto merged = merger.composeGrids();
  ASSERT_NE(merged, nullptr);

  // Count how many occupied cells exist in the merged map: should be exactly 1
  // because both robots' obstacle cells represent the same world coordinate (1.0, 1.0)!
  int occupied_count = 0;
  size_t occupied_idx = 0;
  for (size_t i = 0; i < merged->data.size(); ++i) {
    if (merged->data[i] == 100) {
      ++occupied_count;
      occupied_idx = i;
    }
  }
  EXPECT_EQ(occupied_count, 1);

  size_t mx = occupied_idx % merged->info.width;
  size_t my = occupied_idx / merged->info.width;
  double world_x = merged->info.origin.position.x + mx * merged->info.resolution;
  double world_y = merged->info.origin.position.y + my * merged->info.resolution;
  EXPECT_NEAR(world_x, 1.0, 1e-4);
  EXPECT_NEAR(world_y, 1.0, 1e-4);

  // Simulate slam_toolbox dynamically resizing grid1 in -x/-y direction:
  // new origin (-3.0, -2.5), size 180x150.
  // Same obstacle at robot1 map (3.0, 1.5) is now at cell ((3.0 - (-3.0))/0.05, (1.5 - (-2.5))/0.05) = (120, 80)
  auto grid1_expanded = makeGrid(180, 150, res, -3.0, -2.5);
  grid1_expanded->data[80 * grid1_expanded->info.width + 120] = 100;

  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> grids2{grid1_expanded, grid2};
  merger.feed(grids2.begin(), grids2.end());
  ASSERT_TRUE(merger.setTransforms(poses.begin(), poses.end(), true));
  auto merged2 = merger.composeGrids();
  ASSERT_NE(merged2, nullptr);

  occupied_count = 0;
  for (size_t i = 0; i < merged2->data.size(); ++i) {
    if (merged2->data[i] == 100) {
      ++occupied_count;
      occupied_idx = i;
    }
  }
  EXPECT_EQ(occupied_count, 1);
  mx = occupied_idx % merged2->info.width;
  my = occupied_idx / merged2->info.width;
  world_x = merged2->info.origin.position.x + mx * merged2->info.resolution;
  world_y = merged2->info.origin.position.y + my * merged2->info.resolution;
  EXPECT_NEAR(world_x, 1.0, 1e-4);
  EXPECT_NEAR(world_y, 1.0, 1e-4);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

