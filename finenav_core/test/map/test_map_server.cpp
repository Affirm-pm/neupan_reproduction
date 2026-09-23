// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/map/map_server.hpp"
#include "test_helpers.hpp"

using namespace finenav::core;
using finenav::common::Position3D;

class MapServerTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }
    void TearDown() override {
        rclcpp::shutdown();
    }
};

TEST_F(MapServerTest, ConstructWithValidStateProvider) {
    auto node = rclcpp::Node::make_shared("test_node");
    MockStateProvider provider;

    auto server = std::make_shared<MapServer<SmokeMap>>(
        node, "smoke_map", "test_server", &provider);
    EXPECT_TRUE(server != nullptr);
}

TEST_F(MapServerTest, GetLockedReadView) {
    auto node = rclcpp::Node::make_shared("test_node");
    MockStateProvider provider;
    auto server = std::make_shared<MapServer<SmokeMap>>(
        node, "smoke_map", "test_server", &provider);

    auto view = server->getLockedReadView();
    EXPECT_TRUE(view->isInside(Position3D{}));
}

TEST_F(MapServerTest, GetLockedWriteView) {
    auto node = rclcpp::Node::make_shared("test_node");
    MockStateProvider provider;
    auto server = std::make_shared<MapServer<SmokeMap>>(
        node, "smoke_map", "test_server", &provider);

    auto view = server->getLockedWriteView();
    EXPECT_TRUE(view->isInside(Position3D{}));
    view->shiftWindowTo(Position3D{10, 0, 0});
}

TEST_F(MapServerTest, NonCopyableNonMovable) {
    static_assert(!std::is_copy_constructible_v<MapServer<SmokeMap>>);
    static_assert(!std::is_copy_assignable_v<MapServer<SmokeMap>>);
    static_assert(!std::is_move_constructible_v<MapServer<SmokeMap>>);
    static_assert(!std::is_move_assignable_v<MapServer<SmokeMap>>);
}
