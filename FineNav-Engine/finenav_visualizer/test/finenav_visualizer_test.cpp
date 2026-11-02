// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include "finenav_visualizer/finenav_visualizer.hpp"
#include "finenav_visualizer/marker_viz.hpp"
#include "finenav_visualizer/point_cloud_viz.hpp"

namespace {

// A channel is identified by its topic and by nothing else: markers("costmap") is
// the same buffer everywhere it is asked for.  That is the whole contract, and it
// matters because the alternative is silent — two callers naming one topic would
// stage into two buffers, and only one of them would ever reach the viewer, with
// each side blind to the other's shapes.
//
// It is checked here on the PUBLIC surface: no private access, and no node,
// because get-or-create is a property of the maps the handle keeps rather than of
// anything it publishes.  The default-constructed handle is the honest subject —
// the identity an algorithm relies on is the same one it gets.
TEST(FineNavVisualizerTest, SameTopicReturnsTheSameMarkerChannel) {
    finenav::viz::FineNavVisualizer viz;

    finenav::viz::MarkerViz& first = viz.markers("finenav_test_markers_same");
    finenav::viz::MarkerViz& second = viz.markers("finenav_test_markers_same");

    EXPECT_EQ(&first, &second);
}

TEST(FineNavVisualizerTest, DifferentTopicsReturnDifferentMarkerChannels) {
    finenav::viz::FineNavVisualizer viz;

    finenav::viz::MarkerViz& a = viz.markers("finenav_test_markers_a");
    finenav::viz::MarkerViz& b = viz.markers("finenav_test_markers_b");

    EXPECT_NE(&a, &b);
}

TEST(FineNavVisualizerTest, SameTopicReturnsTheSameCloudChannel) {
    finenav::viz::FineNavVisualizer viz;

    finenav::viz::PointCloudViz& first = viz.pointCloud("finenav_test_cloud_same");
    finenav::viz::PointCloudViz& second = viz.pointCloud("finenav_test_cloud_same");

    EXPECT_EQ(&first, &second);
}

TEST(FineNavVisualizerTest, DifferentTopicsReturnDifferentCloudChannels) {
    finenav::viz::FineNavVisualizer viz;

    finenav::viz::PointCloudViz& a = viz.pointCloud("finenav_test_cloud_a");
    finenav::viz::PointCloudViz& b = viz.pointCloud("finenav_test_cloud_b");

    EXPECT_NE(&a, &b);
}

// Markers and clouds are separate namespaces of topics, because they carry
// different message types: one name may stand for both without the two sharing a
// buffer, and creating one must leave the other exactly where it was — in either
// order.
TEST(FineNavVisualizerTest, MarkerAndCloudMapsDoNotCollide) {
    finenav::viz::FineNavVisualizer viz;

    finenav::viz::MarkerViz& markers = viz.markers("finenav_test_shared_topic");
    finenav::viz::PointCloudViz& cloud = viz.pointCloud("finenav_test_shared_topic");

    EXPECT_NE(static_cast<const void*>(&markers), static_cast<const void*>(&cloud));
    EXPECT_EQ(&markers, &viz.markers("finenav_test_shared_topic"));
    EXPECT_EQ(&cloud, &viz.pointCloud("finenav_test_shared_topic"));
}

}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
