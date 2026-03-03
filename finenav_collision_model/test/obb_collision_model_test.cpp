// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include "finenav_collision_model/obb_collision_model.hpp"
#include <cmath>

using namespace finenav;

// ─── 辅助：始终返回 false 的 rule（无障碍） ───────────────────────────
auto free_rule  = [](const Position3D&) { return false; };
// 始终返回 true 的 rule（全障碍）
auto block_rule = [](const Position3D&) { return true;  };

// ─── 1. 非法参数：step <= 0 抛出异常 ─────────────────────────────────
TEST(OBBCollisionModelTest, InvalidStepThrows) {
    OBBCollisionPolicy bad_policy{0.0};
    EXPECT_THROW(OBBCollisionModel({1.0, 1.0, 1.0}, Pose::Identity(), bad_policy),
                 std::invalid_argument);

    OBBCollisionPolicy neg_policy{-0.1};
    EXPECT_THROW(OBBCollisionModel({1.0, 1.0, 1.0}, Pose::Identity(), neg_policy),
                 std::invalid_argument);
}

// ─── 2. 未调用 precompute 直接 checkCollision 抛出异常 ──────────────
TEST(OBBCollisionModelTest, CheckCollisionBeforePrecomputeThrows) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    EXPECT_THROW(model.checkCollision(free_rule), std::logic_error);
}

// ─── 3. 无碰撞：rule 全返回 false ────────────────────────────────────
TEST(OBBCollisionModelTest, NoCollisionWhenFree) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    EXPECT_FALSE(model.checkCollision(pose, free_rule));
}

// ─── 4. 有碰撞：rule 全返回 true ─────────────────────────────────────
TEST(OBBCollisionModelTest, CollisionWhenBlocked) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    EXPECT_TRUE(model.checkCollision(pose, block_rule));
}

// ─── 5. 高频接口：precompute 后多次 checkCollision ───────────────────
TEST(OBBCollisionModelTest, HighFreqInterfaceReusesCache) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    model.precompute(pose);

    // 多次调用应返回相同结果
    EXPECT_FALSE(model.checkCollision(free_rule));
    EXPECT_FALSE(model.checkCollision(free_rule));
    EXPECT_TRUE(model.checkCollision(block_rule));
}

// ─── 6. 障碍在 OBB 范围内才碰撞（位置相关 rule）────────────────────
TEST(OBBCollisionModelTest, CollisionOnlyWhenObstacleIntersectsOBB) {
    // OBB: 1x1x1 m，中心在原点，半轴长 0.5m，X 面表面点 x = ±0.5
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    // 障碍在 x=2.0，远离 OBB → 无碰撞
    auto far_rule = [](const Position3D& p) { return p.x() > 1.9; };
    EXPECT_FALSE(model.checkCollision(pose, far_rule));

    // 障碍在 x=0.5（OBB 表面），预计有采样点触碰
    auto surface_rule = [](const Position3D& p) { return std::abs(p.x() - 0.5) < 1e-6; };
    EXPECT_TRUE(model.checkCollision(pose, surface_rule));
}

// ─── 7. OBB 带偏移的 local_pose ──────────────────────────────────────
TEST(OBBCollisionModelTest, LocalPoseOffset) {
    // OBB 在机器人本体系中偏移 (5, 0, 0)
    Pose local_pose = Pose::Identity();
    local_pose.translation() = Vector3D(5.0, 0.0, 0.0);

    OBBCollisionModel model({1.0, 1.0, 1.0}, local_pose);
    Pose robot_pose = Pose::Identity(); // 机器人在原点

    // OBB 中心在世界系 (5, 0, 0)，半轴长 0.5m，X 面表面点 x = 4.5 / 5.5
    // 障碍在原点附近 → 无碰撞
    auto near_origin = [](const Position3D& p) { return p.norm() < 0.6; };
    EXPECT_FALSE(model.checkCollision(robot_pose, near_origin));

    // 障碍在 x=5.0 附近 → 有碰撞
    auto near_offset = [](const Position3D& p) { return std::abs(p.x() - 5.0) < 0.6; };
    EXPECT_TRUE(model.checkCollision(robot_pose, near_offset));
}

// ─── 8. OBB 带旋转的 local_pose（绕 Z 轴旋转 90°）────────────────────
TEST(OBBCollisionModelTest, LocalPoseRotation) {
    // OBB 2x1x1（长轴沿 X），绕 Z 旋转 90° 后长轴变为 Y 方向
    Pose local_pose = Pose::Identity();
    local_pose.linear() = Eigen::AngleAxisd(M_PI / 2.0, Vector3D::UnitZ()).toRotationMatrix();

    OBBCollisionModel model({2.0, 1.0, 1.0}, local_pose);
    Pose robot_pose = Pose::Identity();

    // 旋转后 OBB 长轴在 Y，Y 方向表面点应在 ±1.0
    // x 方向表面点应在 ±0.5（原来的 Y 半轴 = 0.5）
    auto y_surface = [](const Position3D& p) { return std::abs(std::abs(p.y()) - 1.0) < 1e-6; };
    EXPECT_TRUE(model.checkCollision(robot_pose, y_surface));

    // X 方向不应有到 ±1.0 的采样点（原长轴已旋转到 Y）
    auto x_far = [](const Position3D& p) { return std::abs(p.x()) > 0.9; };
    EXPECT_FALSE(model.checkCollision(robot_pose, x_far));
}

// ─── 9. 机器人移动后采样点跟随更新 ──────────────────────────────────
TEST(OBBCollisionModelTest, PrecomputeUpdatesWithPose) {
    OBBCollisionModel model({1.0, 1.0, 1.0});

    // 位姿1：OBB 在原点
    Pose pose1 = Pose::Identity();
    model.precompute(pose1);
    auto at_origin = [](const Position3D& p) { return p.norm() < 0.6; };
    EXPECT_TRUE(model.checkCollision(at_origin));

    // 位姿2：机器人平移到 (10, 0, 0)，OBB 中心也移到 (10, 0, 0)
    Pose pose2 = Pose::Identity();
    pose2.translation() = Vector3D(10.0, 0.0, 0.0);
    model.precompute(pose2);

    // 原点附近不再有碰撞
    EXPECT_FALSE(model.checkCollision(at_origin));

    // x=10 附近有碰撞
    auto at_new_pos = [](const Position3D& p) { return (p - Vector3D(10, 0, 0)).norm() < 0.6; };
    EXPECT_TRUE(model.checkCollision(at_new_pos));
}

// ─── 10. 边界补全：角点 (max_u, max_v) 确实在缓存中 ─────────────────
// precompute 中专门补全了三种边点，若漏掉会导致 OBB 棱/角漏检。
// OBB {1,1,1}，某面上角点为 (±0.5, ±0.5, ±0.5)，验证这些点存在。
TEST(OBBCollisionModelTest, BorderCompletionCornerPointsPresent) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    model.precompute(pose);

    // 精确命中 OBB 某个角点 (0.5, 0.5, 0.5)
    auto corner_rule = [](const Position3D& p) {
        return (p - Vector3D(0.5, 0.5, 0.5)).norm() < 1e-9;
    };
    EXPECT_TRUE(model.checkCollision(corner_rule))
        << "Corner point (0.5,0.5,0.5) must be in cached_points_";

    // 精确命中另一角点 (-0.5, 0.5, -0.5)
    auto corner_rule2 = [](const Position3D& p) {
        return (p - Vector3D(-0.5, 0.5, -0.5)).norm() < 1e-9;
    };
    EXPECT_TRUE(model.checkCollision(corner_rule2))
        << "Corner point (-0.5,0.5,-0.5) must be in cached_points_";
}

// ─── 11. 非对称 OBB：三轴完全不同 ───────────────────────────────────
// 使用 {0.5, 2.0, 3.0}，验证各轴表面范围均正确。
TEST(OBBCollisionModelTest, AsymmetricOBBSurfaceBounds) {
    // half: x=0.25, y=1.0, z=1.5
    OBBCollisionModel model({0.5, 2.0, 3.0});
    Pose pose = Pose::Identity();
    model.precompute(pose);

    // X 面：x = ±0.25，OBB 内 y∈[-1,1], z∈[-1.5,1.5]
    auto x_face = [](const Position3D& p) { return std::abs(std::abs(p.x()) - 0.25) < 1e-9; };
    EXPECT_TRUE(model.checkCollision(x_face));

    // X 方向不应有超出 ±0.25 的采样点
    auto x_out = [](const Position3D& p) { return std::abs(p.x()) > 0.26; };
    EXPECT_FALSE(model.checkCollision(x_out));

    // Y 面：y = ±1.0
    auto y_face = [](const Position3D& p) { return std::abs(std::abs(p.y()) - 1.0) < 1e-9; };
    EXPECT_TRUE(model.checkCollision(y_face));

    // Z 面角点：(±0.25, ±1.0, ±1.5) 必须存在
    auto z_corner = [](const Position3D& p) {
        return (p - Vector3D(0.25, 1.0, 1.5)).norm() < 1e-9;
    };
    EXPECT_TRUE(model.checkCollision(z_corner));
}

// ─── 12. 旋转 + 平移组合位姿 ─────────────────────────────────────────
// 机器人位姿：绕 Z 旋转 90° 后向 X 方向平移 5m。
// OBB {2,1,1}，旋转后长轴变 Y，中心移到 (5,0,0)。
TEST(OBBCollisionModelTest, CombinedRotationAndTranslation) {
    OBBCollisionModel model({2.0, 1.0, 1.0});

    Pose robot_pose = Pose::Identity();
    robot_pose.linear() = Eigen::AngleAxisd(M_PI / 2.0, Vector3D::UnitZ()).toRotationMatrix();
    robot_pose.translation() = Vector3D(5.0, 0.0, 0.0);
    model.precompute(robot_pose);

    // OBB 中心在 (5,0,0)，旋转后长轴沿 Y，所以 Y 表面在 y = ±1.0（相对中心）
    // 世界系表面点应在 (5, ±1.0, *) 附近
    auto y_surface_world = [](const Position3D& p) {
        return std::abs(p.x() - 5.0) < 1e-6 && std::abs(std::abs(p.y()) - 1.0) < 1e-6;
    };
    EXPECT_TRUE(model.checkCollision(y_surface_world))
        << "After 90deg rotation + translation, Y-face should appear at world (5, ±1, z)";

    // 原点附近不应有任何采样点
    auto near_origin = [](const Position3D& p) { return p.norm() < 3.5; };
    EXPECT_FALSE(model.checkCollision(near_origin))
        << "OBB translated to x=5 should not have points near origin";
}

// ─── 13. 缓存点数量的确定性：precompute 调用两次结果相同 ─────────────
// 验证 precompute 内部正确调用了 clear()，不会发生累积追加。
// 方法：用计数 lambda 统计遍历点数，两次 precompute 后点数应相同。
TEST(OBBCollisionModelTest, CacheCountIsDeterministic) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    // 第一次 precompute
    model.precompute(pose);
    int count1 = 0;
    model.checkCollision([&count1](const Position3D&) { ++count1; return false; });

    // 第二次 precompute（相同位姿）
    model.precompute(pose);
    int count2 = 0;
    model.checkCollision([&count2](const Position3D&) { ++count2; return false; });

    EXPECT_EQ(count1, count2)
        << "precompute called twice should produce same number of cached points, "
        << "got " << count1 << " vs " << count2;
    EXPECT_GT(count1, 0) << "cached_points_ should not be empty after precompute";
}
