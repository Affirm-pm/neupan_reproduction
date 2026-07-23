// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/navigator.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "finenav_util/common/log_macros.hpp"

namespace finenav::navigator {

BtNavigator::BtNavigator(rclcpp::Node::SharedPtr parent_node,
                     finenav::core::IRobotStateProvider* state_provider)
    : state_provider_(state_provider), master_(parent_node), parent_node_(parent_node)
{
    // ── 创建私有子节点（参数前缀由 yaml 的顶层 key "finenav_navigator" 决定）──
    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        parent_node->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(
        "finenav_navigator",            // 节点名称，与 yaml 顶层 key 一致
        parent_node->get_name(),        // 命名空间继承宿主节点名
        pnode_opts);

    // ── 读取参数 ──────────────────────────────────────────────────────────────
    param_listener_ = std::make_unique<finenav_navigator::ParamListener>(pnode_);
    params_ = param_listener_->get_params();

    // ── 创建 Action Server ────────────────────────────────────────────────────
    action_server_ = std::make_unique<ActionServer>(
        pnode_,
        params_.action_name,
        std::bind(&BtNavigator::execute, this));

    // ── 初始化行为树引擎 ──────────────────────────────────────────────────────
    bt_engine_ = std::make_unique<BehaviorTreeEngine>();
    if (!params_.plugins.empty())       { bt_engine_->registerPlugins(params_.plugins); }
    if (!params_.behavior_trees.empty()){ bt_engine_->registerBehaviorTrees(params_.behavior_trees); }

    // ── 启动 spin 线程（最后执行，确保所有订阅/服务已注册）────────────────────
    pnode_thread_ = std::make_unique<finenav::util::NodeThread>(pnode_);

    FINENAV_LOG_INFO(pnode_->get_logger(), "[Navigator] Initialized and spinning.");
}

void BtNavigator::execute()
{
    FINENAV_LOG_INFO(pnode_->get_logger(), "[Navigator] Executing action...");

    auto goal = action_server_->get_current_goal();

    try {
        tree_ = bt_engine_->factory().createTree(goal->behavior_tree);
    } catch (std::exception& ex) {
        FINENAV_LOG_ERROR(pnode_->get_logger(),
            "[Navigator] Failed to create tree [{}]: {}",
            goal->behavior_tree, ex.what());
        action_server_->terminate_all();
        return;
    }

    auto blackboard = tree_.rootBlackboard();
    blackboard->set("goal", goal->pose.pose);
    blackboard->set("_node", parent_node_);
    blackboard->set("_nav_task_master", &master_);

    // ── _root ref_traj publisher (first layer subscribes via connectEdge) ──
    using FeedbackMsg = finenav_interfaces::msg::TaskFeedback;
    auto root_pub = pnode_->create_publisher<FeedbackMsg>(
        finenav::util::TaskChannel::feedbackTopic("_root"), 10);

    rclcpp::WallRate loop_rate(params_.tick_frequency);
    FINENAV_LOG_INFO(pnode_->get_logger(),
        "[Navigator] Starting navigation to ({:.2f}, {:.2f})",
        goal->pose.pose.position.x, goal->pose.pose.position.y);

    while (rclcpp::ok()) {
        if (action_server_->is_cancel_requested()) {
            FINENAV_LOG_INFO(pnode_->get_logger(), "[Navigator] Cancellation requested.");
            master_.stopAll();
            tree_.haltTree();
            action_server_->terminate_all();
            return;
        }

        if (action_server_->is_preempt_requested()) {
            FINENAV_LOG_INFO(pnode_->get_logger(), "[Navigator] Preemption requested.");
            master_.stopAll();
            tree_.haltTree();
            return;
        }

        // 每 tick 前将最新机器人位姿和速度注入 Blackboard，供条件节点读取
        if (state_provider_) {
            if (auto rs = state_provider_->getRobotState()) {
                geometry_msgs::msg::Pose p;
                p.position.x    = rs->position.x();
                p.position.y    = rs->position.y();
                p.position.z    = rs->position.z();
                p.orientation.w = rs->orientation.w();
                p.orientation.x = rs->orientation.x();
                p.orientation.y = rs->orientation.y();
                p.orientation.z = rs->orientation.z();
                blackboard->set<geometry_msgs::msg::Pose>("robot_pose", p);

                geometry_msgs::msg::Twist t;
                t.linear.x  = rs->linear_velocity.x();
                t.linear.y  = rs->linear_velocity.y();
                t.linear.z  = rs->linear_velocity.z();
                t.angular.x = rs->angular_velocity.x();
                t.angular.y = rs->angular_velocity.y();
                t.angular.z = rs->angular_velocity.z();
                blackboard->set<geometry_msgs::msg::Twist>("robot_twist", t);
            }
        }

        auto status = tree_.tickExactlyOnce();

        // Forward fresh leaf trajectory to chassis command
        if (chassis_cmd_) {
            for (auto& leaf : pipe_.leaves()) {
                auto fb = master_.feedback(leaf);
                if (fb && fb != last_consumed_fb_) {
                    if (chassis_cmd_->validate(fb->trajectory)) {
                        chassis_cmd_->publish(fb->trajectory);
                    } else {
                        FINENAV_LOG_WARN(pnode_->get_logger(),
                            "ChassisCommand rejected trajectory from '{}' (valid_fields=0x{:x}, twists={}).",
                            leaf, fb->trajectory.valid_fields, fb->trajectory.twists.size());
                    }
                    last_consumed_fb_ = fb;
                    break;
                }
            }
        }

        if (status == BT::NodeStatus::SUCCESS) {
            FINENAV_LOG_INFO(pnode_->get_logger(), "[Navigator] Navigation succeeded.");
            action_server_->succeeded_current();
            return;
        } else if (status == BT::NodeStatus::FAILURE) {
            FINENAV_LOG_ERROR(pnode_->get_logger(), "[Navigator] Navigation failed.");
            action_server_->terminate_all();
            return;
        }

        // TODO: 发布 Feedback

        loop_rate.sleep();
    }
}

} // namespace finenav::navigator
