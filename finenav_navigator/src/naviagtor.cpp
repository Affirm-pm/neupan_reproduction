// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/navigator.hpp"
#include "finenav_navigator/chassis_command/chassis_cmd_vel.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "finenav_util/common/log_macros.hpp"

namespace finenav::navigator {

BtNavigator::BtNavigator(rclcpp::Node::SharedPtr parent_node,
                     finenav::core::RobotStateProvider* state_provider)
    : state_provider_(state_provider), parent_node_(parent_node)
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

    // TaskMaster on pnode_ so its topics share the /hello_finenav namespace
    // with every ControlLayer's TaskWorker (also under /hello_finenav).
    master_ = std::make_unique<finenav::util::TaskMaster>(pnode_);

    // PipelineController: BT nodes drive the control_pipeline through this.
    pipeline_ = std::make_unique<PipelineController>(pnode_, master_.get(), &pipe_);

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

    // Always load framework built-in plugins first.
    bt_engine_->registerPlugins({"finenav_navigator/lib/finenav_navigator"});

    // Then load any additional paths from user parameters.
    if (!params_.plugins.empty())       { bt_engine_->registerPlugins(params_.plugins); }
    if (!params_.behavior_trees.empty()){ bt_engine_->registerBehaviorTrees(params_.behavior_trees); }

    // ── 启动 spin 线程（最后执行，确保所有订阅/服务已注册）────────────────────
    pnode_thread_ = std::make_unique<finenav::util::NodeThread>(pnode_);

    // ── Default chassis command ─────────────────────────────────────────────
    chassis_cmd_ = std::make_shared<ChassisCmdVel>(parent_node_);

    FINENAV_LOG_INFO(pnode_->get_logger(), "Initialized and spinning.");
}

void BtNavigator::execute()
{
    FINENAV_LOG_INFO(pnode_->get_logger(), "Executing action...");

    auto goal = action_server_->get_current_goal();
    if (!goal || goal->poses.empty()) {
        FINENAV_LOG_ERROR(pnode_->get_logger(),
            "NavigateToPose goal carries no poses — nothing to navigate to.");
        if (goal) action_server_->terminate_all();
        return;
    }

    try {
        tree_ = bt_engine_->factory().createTree(goal->behavior_tree);
    } catch (std::exception& ex) {
        FINENAV_LOG_ERROR(pnode_->get_logger(),
            "Failed to create tree [{}]: {}",
            goal->behavior_tree, ex.what());
        action_server_->terminate_all();
        return;
    }

    auto blackboard = tree_.rootBlackboard();
    blackboard->set("goal_poses", goal->poses);
    blackboard->set("_pipeline", pipeline_.get());
    blackboard->set("_node", parent_node_);
    blackboard->set("_chassis_cmd", chassis_cmd_.get());

    auto episode_start = pnode_->now();
    rclcpp::WallRate loop_rate(params_.tick_frequency);
    FINENAV_LOG_INFO(pnode_->get_logger(),
        "Starting navigation to ({:.2f}, {:.2f})",
        goal->poses.back().position.x, goal->poses.back().position.y);

    while (rclcpp::ok()) {
        if (action_server_->is_cancel_requested()) {
            FINENAV_LOG_INFO(pnode_->get_logger(), "Cancellation requested.");
            master_->stopAll();
            tree_.haltTree();
            action_server_->terminate_all();
            return;
        }

        if (action_server_->is_preempt_requested()) {
            FINENAV_LOG_INFO(pnode_->get_logger(), "Preemption requested.");
            master_->stopAll();
            tree_.haltTree();
            return;
        }

        // 每 tick 前将最新机器人位姿和速度注入 Blackboard，供条件节点读取
        if (state_provider_) {
            if (auto rs = state_provider_->getRobotState()) {
                geometry_msgs::msg::Pose p;
                p.position.x    = rs->pose.position.x;
                p.position.y    = rs->pose.position.y;
                p.position.z    = rs->pose.position.z;
                p.orientation.w = rs->pose.orientation.w;
                p.orientation.x = rs->pose.orientation.x;
                p.orientation.y = rs->pose.orientation.y;
                p.orientation.z = rs->pose.orientation.z;
                blackboard->set<geometry_msgs::msg::Pose>("robot_pose", p);

                geometry_msgs::msg::Twist t;
                t.linear.x  = rs->twist.linear.x;
                t.linear.y  = rs->twist.linear.y;
                t.linear.z  = rs->twist.linear.z;
                t.angular.x = rs->twist.angular.x;
                t.angular.y = rs->twist.angular.y;
                t.angular.z = rs->twist.angular.z;
                blackboard->set<geometry_msgs::msg::Twist>("robot_twist", t);
            }
        }

        auto status = tree_.tickExactlyOnce();

        // Forward fresh leaf trajectory to chassis command
        if (chassis_cmd_) {
            std::vector<std::pair<std::string, std::shared_ptr<const FeedbackMsg>>> fresh;
            for (auto& leaf : pipe_.leaves()) {
                auto fb = master_->feedback(leaf);
                if (fb && fb != last_consumed_fb_[leaf]) {
                    fresh.emplace_back(leaf, fb);
                }
            }
            if (fresh.size() == 1) {
                auto& [leaf, fb] = fresh[0];
                if (chassis_cmd_->validate(fb->trajectory)) {
                    chassis_cmd_->publish(fb->trajectory);
                } else {
                    FINENAV_LOG_WARN(pnode_->get_logger(),
                        "ChassisCommand rejected trajectory from '{}' (valid_fields=0x{:x}, twists={}).",
                        leaf, fb->trajectory.valid_fields, fb->trajectory.twists.size());
                }
                last_consumed_fb_[leaf] = fb;
            } else if (fresh.size() > 1) {
                std::string names;
                for (auto& [n, _] : fresh) names += n + " ";
                FINENAV_LOG_WARN(pnode_->get_logger(),
                    "Multiple leaves have fresh trajectories ({}). "
                    "Check BT. Not publishing.", names);
            }
        }

        if (status == BT::NodeStatus::SUCCESS) {
            FINENAV_LOG_INFO(pnode_->get_logger(), "Navigation succeeded.");
            auto result = std::make_shared<typename ActionT::Result>();
            if (state_provider_) {
                if (auto rs = state_provider_->getRobotState()) {
                    auto& g = goal->poses.back();
                    result->position_error.x = rs->pose.position.x - g.position.x;
                    result->position_error.y = rs->pose.position.y - g.position.y;
                    result->position_error.z = rs->pose.position.z - g.position.z;

                    Eigen::Quaterniond q_cur(rs->pose.orientation.w, rs->pose.orientation.x,
                                             rs->pose.orientation.y, rs->pose.orientation.z);
                    Eigen::Quaterniond q_goal(g.orientation.w, g.orientation.x,
                                              g.orientation.y, g.orientation.z);
                    auto euler = (q_goal.inverse() * q_cur).toRotationMatrix()
                                     .eulerAngles(2, 1, 0);  // ZYX order
                    result->angle_error.x = static_cast<float>(euler[2]);  // roll
                    result->angle_error.y = static_cast<float>(euler[1]);  // pitch
                    result->angle_error.z = static_cast<float>(euler[0]);  // yaw
                }
            }
            result->total_time = rclcpp::Duration(pnode_->now() - episode_start);
            master_->stopAll();
            action_server_->succeeded_current(result);
            return;
        } else if (status == BT::NodeStatus::FAILURE) {
            FINENAV_LOG_ERROR(pnode_->get_logger(), "Navigation failed.");
            master_->stopAll();
            action_server_->terminate_all();
            return;
        }

        // Publish feedback (throttled to ~10 Hz)
        if (state_provider_ && (pnode_->now() - last_feedback_time_).seconds() >= 0.1) {
            if (auto rs = state_provider_->getRobotState()) {
                auto fb = std::make_shared<typename ActionT::Feedback>();
                fb->current_pose.header.stamp = pnode_->now();
                fb->current_pose.header.frame_id = "map"; // TODO： 弄个宏定义define
                fb->current_pose.pose.position.x    = rs->pose.position.x;
                fb->current_pose.pose.position.y    = rs->pose.position.y;
                fb->current_pose.pose.position.z    = rs->pose.position.z;
                fb->current_pose.pose.orientation.w = rs->pose.orientation.w;
                fb->current_pose.pose.orientation.x = rs->pose.orientation.x;
                fb->current_pose.pose.orientation.y = rs->pose.orientation.y;
                fb->current_pose.pose.orientation.z = rs->pose.orientation.z;
                fb->navigation_time = rclcpp::Duration(pnode_->now() - episode_start);
                fb->current_speed.linear.x  = rs->twist.linear.x;
                fb->current_speed.linear.y  = rs->twist.linear.y;
                fb->current_speed.linear.z  = rs->twist.linear.z;
                fb->current_speed.angular.x = rs->twist.angular.x;
                fb->current_speed.angular.y = rs->twist.angular.y;
                fb->current_speed.angular.z = rs->twist.angular.z;
                action_server_->publish_feedback(fb);
            }
            last_feedback_time_ = pnode_->now();
        }

        loop_rate.sleep();
    }
}

} // namespace finenav::navigator
