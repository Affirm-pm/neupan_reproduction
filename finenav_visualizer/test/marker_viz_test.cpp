#include <gtest/gtest.h>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#define private public
#include "finenav_visualizer/marker_viz.hpp"
#undef private

namespace
{

std_msgs::msg::ColorRGBA color()
{
  std_msgs::msg::ColorRGBA value;
  value.r = 1.0F;
  value.a = 1.0F;
  return value;
}

std::vector<Eigen::Vector3d> points(double offset)
{
  return {
    Eigen::Vector3d{offset, 0.0, 0.0},
    Eigen::Vector3d{offset + 1.0, 0.0, 0.0},
    Eigen::Vector3d{offset + 2.0, 1.0, 0.0},
  };
}

TEST(MarkerVizTest, AddsLineStripsWithUniqueIds)
{
  finenav::viz::MarkerViz markers;

  markers.addLineStrip(points(0.0), color(), 0.02, "trajectories");
  markers.addLineStrip(points(10.0), color(), 0.04, "trajectories");

  ASSERT_EQ(markers.markers_.markers.size(), 2U);
  EXPECT_EQ(markers.markers_.markers[0].type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(markers.markers_.markers[1].type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(markers.markers_.markers[0].ns, "trajectories");
  EXPECT_EQ(markers.markers_.markers[1].ns, "trajectories");
  EXPECT_EQ(markers.markers_.markers[0].id, 0);
  EXPECT_EQ(markers.markers_.markers[1].id, 1);
  EXPECT_EQ(markers.markers_.markers[0].points.size(), 3U);
  EXPECT_EQ(markers.markers_.markers[1].points.size(), 3U);
  EXPECT_FLOAT_EQ(markers.markers_.markers[0].scale.x, 0.02F);
  EXPECT_FLOAT_EQ(markers.markers_.markers[1].scale.x, 0.04F);
}

TEST(MarkerVizTest, ClearResetsMarkerIds)
{
  finenav::viz::MarkerViz markers;

  markers.addLineStrip(points(0.0), color());
  markers.clear();
  markers.addLineStrip(points(1.0), color());

  ASSERT_EQ(markers.markers_.markers.size(), 1U);
  EXPECT_EQ(markers.markers_.markers.front().id, 0);
}

TEST(MarkerVizTest, SharesIdSequenceAcrossShapeApis)
{
  finenav::viz::MarkerViz markers;

  markers.addLineStrip(points(0.0), color());
  markers.addSphere(Eigen::Vector3d{0.0, 0.0, 0.0}, 0.01, color());
  markers.addSpheres(
    {Eigen::Vector3d{1.0, 0.0, 0.0}, Eigen::Vector3d{2.0, 0.0, 0.0}},
    0.01,
    {color(), color()});

  ASSERT_EQ(markers.markers_.markers.size(), 4U);
  EXPECT_EQ(markers.markers_.markers[0].id, 0);
  EXPECT_EQ(markers.markers_.markers[1].id, 1);
  EXPECT_EQ(markers.markers_.markers[2].id, 2);
  EXPECT_EQ(markers.markers_.markers[3].id, 3);
}

}  // namespace
