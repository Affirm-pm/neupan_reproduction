#include <gtest/gtest.h>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

// Pulled in by the finenav headers below, so it has to be parsed BEFORE the
// access hack: `#define private public` is only meant to open finenav's own
// declarations, not a third-party library's internals.
#include <fmt/format.h>

#define private public
#include "finenav_visualizer/marker_viz.hpp"
#undef private

namespace
{

finenav::viz::Color color()
{
  return finenav::viz::Color{1.0F, 0.0F, 0.0F, 1.0F};
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
  finenav::viz::MarkerViz markers("test_markers");

  markers.addLineStrip(points(0.0), 0.02, color());
  markers.addLineStrip(points(10.0), 0.04, color());

  ASSERT_EQ(markers.markers_.markers.size(), 2U);
  EXPECT_EQ(markers.markers_.markers[0].type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(markers.markers_.markers[1].type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(markers.markers_.markers[0].ns, "LineStrip");
  EXPECT_EQ(markers.markers_.markers[1].ns, "LineStrip");
  EXPECT_EQ(markers.markers_.markers[0].id, 0);
  EXPECT_EQ(markers.markers_.markers[1].id, 1);
  EXPECT_EQ(markers.markers_.markers[0].points.size(), 3U);
  EXPECT_EQ(markers.markers_.markers[1].points.size(), 3U);
  EXPECT_FLOAT_EQ(markers.markers_.markers[0].scale.x, 0.02F);
  EXPECT_FLOAT_EQ(markers.markers_.markers[1].scale.x, 0.04F);
}

TEST(MarkerVizTest, ClearResetsMarkerIds)
{
  finenav::viz::MarkerViz markers("test_markers");

  markers.addLineStrip(points(0.0), 0.02, color());
  markers.clear();
  markers.addLineStrip(points(1.0), 0.02, color());

  ASSERT_EQ(markers.markers_.markers.size(), 1U);
  EXPECT_EQ(markers.markers_.markers.front().id, 0);
}

// The point of Color is that a colour is written the same way wherever it goes:
// a palette name converts implicitly, and a computed colour is a braced list.
// Both are pinned here because neither is enforced by the type system — they
// are conveniences, and a convenience that silently stops working is worse than
// one that was never offered.
TEST(MarkerVizTest, AcceptsNamedAndComputedColors)
{
  finenav::viz::MarkerViz markers("test_markers");

  markers.addSphere(Eigen::Vector3d{0.0, 0.0, 0.0}, 0.5,
                    finenav::viz::ColorName::RED);
  markers.addSphere(Eigen::Vector3d{1.0, 0.0, 0.0}, 0.5, {0.1F, 0.4F, 0.9F, 0.35F});

  ASSERT_EQ(markers.markers_.markers.size(), 2U);
  EXPECT_FLOAT_EQ(markers.markers_.markers[0].color.r, 1.0F);
  EXPECT_FLOAT_EQ(markers.markers_.markers[0].color.g, 0.1F);
  EXPECT_FLOAT_EQ(markers.markers_.markers[0].color.a, 1.0F);
  EXPECT_FLOAT_EQ(markers.markers_.markers[1].color.r, 0.1F);
  EXPECT_FLOAT_EQ(markers.markers_.markers[1].color.b, 0.9F);
  EXPECT_FLOAT_EQ(markers.markers_.markers[1].color.a, 0.35F);
}

// Alpha lives in the colour, so a shape must not scale it on the way through.
// addAxis was the case that got this wrong: it baked alpha into the arrow's
// own colour and addArrow multiplied by it again, halving it twice over.
TEST(MarkerVizTest, AxisKeepsTheAlphaItWasGiven)
{
  finenav::viz::MarkerViz markers("test_markers");

  markers.addAxis(Eigen::Isometry3d::Identity());
  markers.addLine(Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{1.0, 0.0, 0.0},
                  0.02, {1.0F, 0.1F, 0.1F, 0.5F});

  ASSERT_EQ(markers.markers_.markers.size(), 4U);
  for (int i = 0; i < 3; ++i) {
    EXPECT_FLOAT_EQ(markers.markers_.markers[i].color.a, 1.0F);
  }
  EXPECT_FLOAT_EQ(markers.markers_.markers[3].color.a, 0.5F);
}

// The defaults are real calls, not documentation: a shape given neither size
// nor color is still drawn, in the palette's grey, under its kind's namespace.
TEST(MarkerVizTest, DefaultsSizeColorAndNamespace)
{
  finenav::viz::MarkerViz markers("test_markers");

  markers.addLineStrip(points(0.0));
  markers.addCuboid(Eigen::Isometry3d::Identity(), 1.0, 1.0, 1.0);

  ASSERT_EQ(markers.markers_.markers.size(), 2U);
  EXPECT_FLOAT_EQ(markers.markers_.markers[0].scale.x, 0.02F);
  EXPECT_FLOAT_EQ(markers.markers_.markers[0].color.r, 0.7F);
  EXPECT_FLOAT_EQ(markers.markers_.markers[0].color.a, 1.0F);
  EXPECT_EQ(markers.markers_.markers[0].ns, "LineStrip");
  EXPECT_FLOAT_EQ(markers.markers_.markers[1].color.r, 0.7F);
  EXPECT_EQ(markers.markers_.markers[1].ns, "Cuboid");
}

TEST(MarkerVizTest, SharesIdSequenceAcrossShapeApis)
{
  finenav::viz::MarkerViz markers("test_markers");

  markers.addLineStrip(points(0.0), 0.02, color());
  markers.addSphere(Eigen::Vector3d{0.0, 0.0, 0.0}, 0.01, color());
  markers.addCuboid(Eigen::Isometry3d::Identity(), 1.0, 1.0, 1.0, color());

  ASSERT_EQ(markers.markers_.markers.size(), 3U);
  EXPECT_EQ(markers.markers_.markers[0].id, 0);
  EXPECT_EQ(markers.markers_.markers[1].id, 1);
  EXPECT_EQ(markers.markers_.markers[2].id, 2);
}

// A message is the whole picture: whatever the algorithm stopped staging this
// cycle has to disappear from the viewer.  That rests on one DELETEALL leading
// the array, and on it carrying an EMPTY namespace — a namespaced DELETEALL
// clears that namespace alone, and every shape here is stamped with one, so it
// would clear nothing.  The staging buffer itself stays clean: the reset is
// added on the way out, once per message, not once per shape.
TEST(MarkerVizTest, LeadsEveryMessageWithADisplayWideDelete)
{
  finenav::viz::MarkerViz markers("test_markers");

  markers.addSphere(Eigen::Vector3d{0.0, 0.0, 0.0}, 0.5);
  markers.addLineStrip(points(0.0));

  const auto msg = markers.takeMessage(finenav::common::Frame::World, rclcpp::Time{0, 0, RCL_ROS_TIME});

  ASSERT_EQ(msg.markers.size(), 3U);
  EXPECT_TRUE(markers.markers_.markers.empty());  // handed over, not copied
  EXPECT_EQ(msg.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);
  EXPECT_TRUE(msg.markers[0].ns.empty());
  EXPECT_EQ(msg.markers[1].action, visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(msg.markers[2].action, visualization_msgs::msg::Marker::ADD);
}

// Taking is what makes a publish cheap: the array moves out whole, point
// vectors and all, so the buffer behind it is empty and numbered afresh — the
// next cycle stages the same shapes and gets the same message again.
TEST(MarkerVizTest, TakingHandsTheBufferOverAndRenumbersIds)
{
  finenav::viz::MarkerViz markers("test_markers");

  markers.addLineStrip(points(0.0));
  const auto first = markers.takeMessage(finenav::common::Frame::World, rclcpp::Time{0, 0, RCL_ROS_TIME});
  ASSERT_TRUE(markers.markers_.markers.empty());

  markers.addLineStrip(points(4.0));
  const auto second = markers.takeMessage(finenav::common::Frame::World, rclcpp::Time{0, 0, RCL_ROS_TIME});

  ASSERT_EQ(second.markers.size(), 2U);          // DELETEALL + the strip
  EXPECT_EQ(second.markers[1].id, 0);            // afresh, not 1
  EXPECT_EQ(second.markers[1].points.size(), 3U);  // points travelled with it
  EXPECT_FLOAT_EQ(second.markers[1].points[0].x, 4.0F);
  EXPECT_EQ(first.markers[1].points.size(), 3U);
}

// A frame is a TAG chosen when the data is handed over and resolved on the way
// out, not a name copied down at construction: a system that renamed the frames
// at startup gets its own names stamped in, and no spelling of a frame id exists
// for a caller to get wrong.
TEST(MarkerVizTest, StampsTheNameBehindTheFrameTag)
{
  finenav::viz::MarkerViz markers("test_markers");

  markers.addSphere(Eigen::Vector3d{0.0, 0.0, 0.0}, 0.5);

  const auto msg = markers.takeMessage(finenav::common::Frame::Robot,
                                       rclcpp::Time{0, 0, RCL_ROS_TIME});

  ASSERT_EQ(msg.markers.size(), 2U);
  EXPECT_EQ(msg.markers[1].header.frame_id, "base_link");
}

// Persistence is the default and the only behaviour: a zero lifetime is what
// the viewer reads as "until I am replaced", so nothing stamps one.
TEST(MarkerVizTest, ShapesCarryNoExpiry)
{
  finenav::viz::MarkerViz markers("test_markers");

  markers.addSphere(Eigen::Vector3d{0.0, 0.0, 0.0}, 0.5);

  const auto msg = markers.takeMessage(finenav::common::Frame::World, rclcpp::Time{0, 0, RCL_ROS_TIME});

  ASSERT_EQ(msg.markers.size(), 2U);
  EXPECT_EQ(msg.markers[1].lifetime.sec, 0);
  EXPECT_EQ(msg.markers[1].lifetime.nanosec, 0U);
}

}  // namespace
