# Step 4 — BT 接口重设计规划

> 状态：**规划草稿，待评审**。本文件只做设计讨论，不包含实现；评审通过后再派子代理执行。
> 依赖：Step 3（`a34a503`）已完成 —— `_root` 已实体化为 `"_root"`，goal 以 `Pose[]` 走 `NavigateToPose.action` → `_root/__feedback` → `ref_traj`；`TaskCmd` 已合并为 `{START, STOP}` 枚举。
> 依赖：**Step 3.5（ReactivePolicy 受 TaskCmd 控制）** —— 见 §9，已在 §4.3 的节点语义中体现（Reactive 层不再「不受控」）。

---

## 1. 结构前提：ControlPipelineTree 是「有根树」而非一般 DAG

`finenav_core/include/finenav_core/plan/control_pipeline_tree.hpp`：

```cpp
std::unordered_map<std::string, std::string> parent_;                  // 节点 → 至多一个父节点
std::unordered_map<std::string, std::vector<std::string>> children_;  // 节点 → 多个子节点

bool addEdge(from, to) {
    if (parent_.count(to) && parent_[to] != from) return false;  // 单父约束
    if (wouldCreateCycle(from, to)) return false;                 // 无环
    ...
}
```

结论：
- **有根树**：每节点至多一个父节点（`_root` 无父），无环。
- **从 `_root` 到任意节点/leaf 的路径唯一**。
- 「多条路径」的直觉来自 **fan-out**（一个节点多个子节点 → 多个 leaf），不是同一个 leaf 有多条到达路径。
- 支持**分叉（fan-out）**，不支持**汇聚（fan-in/merge）**——无法让两条支路汇入同一下游节点（那需要多父）。

对 Step 4 的意义：`StartBranch(to)` 的路径计算**唯一无歧义**（沿 `parent()` 回溯到 `_root` 再反转）。

### 1.1 更精炼的模型：用户输入 DAG，我们按「单活跃路径」内部拆树

上一节说「当前实现强制单父（树）」，但用户的**目标模型**其实是：让用户输入一个 **DAG**，而不是树。二者的边界在「汇聚（fan-in）」：

```
A ──► B1 ──► C        （C 有两个父：B1、B2）
 └──► B2 ──┘
```

- **当前 `addEdge` 拒绝这种图**（`parent_[C]` 已被 B1 占用，再写 B2 返回 false）。用户希望**允许**它——输入 DAG 时 `C` 只存在**一个实例**，我们内部**不做约化**（约化 = 复制出两个 C，会破坏 warmstart、浪费资源，不可取）。
- **为什么它不是一般的 DAG**：正常工况**不存在两个节点同时向下游发东西**，任何时候只走一条路径。即「fan-in」语义是 **MUX（选一条父边）**，不是 **MERGE（合并两条父边）**。
- **约定（告诉用户）**：只有 **out-degree == 0 的 leaf** 才能输出 command；内部节点（如上面的 C，若它还往下游连）不输出 command。
- **拆树方式：按「控制层」拆，不按「实例」拆**。同一时刻只有一条活跃链 `_root → … → 某 leaf`，这条链天然是树（每节点单父）。切支路 = 停旧链、起新链（沿拓扑序），`C` 实例始终只有一个，warmstart 得以保留。

于是：

> **输入是 DAG，运行是「每次只点亮一条从 `_root` 到某个 leaf 的链」。**

这带来一个**不变量**（Step 4 的 `StartBranch`/`ComputePlan` 必须维护）：

- **Mux 不变量**：任意时刻，对任意节点，至多一条「入边」是活跃的（`_root` 除外）。等价于：command-driven 层中，最多只有一条 `_root → leaf` 链处于 START 状态；其它层都 STOP。
- fan-out（一节点多个子节点）仍是「启动时选一个子」，不是「同时喂多个子」。

对 `StartBranch(to)` 的实现含义：沿 DAG 反查 `to` 的所有父（可能是多个），但**只启动当前选中的那条链**；off-path 的兄弟支路统一 STOP。路径计算的「唯一性」由「单活跃链」保证，而非「图里只有一条路径」。

---

## 2. 现状（Step 3 之后）的三个问题

1. **`_root` 操作是硬编码的**：`BtNavigator::execute()` 直接往 `_root/__feedback` 发 goal，BT 树无法控制「何时开始导航」。
2. **`ComputePlan` 语义混乱**：`goal`/`planner_id` 端口已死（goal 走 `_root`、`planner_id` worker 不用），`onHalted()` 空实现（切支路不停旧层）。
3. **没有「支路」抽象**：选支路 = 手动对路径上每个层逐个 `ComputePlan` start，没有「一键启动某条支路」。

---

## 3. 设计目标

1. 把 `_root` 操作变成 BT 节点 = 「开始导航 / 发布 goal_poses[]」。
2. 保留 per-layer 手动控制（灵活配合 BT 流控），新增「一键启动哪条支路」。

---

## 4. 核心设计

### 4.1 黑盒契约（navigator 设置，替代硬编码）

| key | 值 | 用途 |
|---|---|---|
| `{goal}` | `poses.back()`（单 Pose） | IsGoalReached / IsNearPoint / DynamicGoalFollower（不变） |
| `{goal_poses}` | `poses`（`Pose[]`） | 供 `PublishGoal` 发布到 `_root` |
| `_pipeline` | `PipelineController*`（新） | 替代裸的 `_nav_task_master`，封装所有控制/数据操作 |
| `_node` / `_chassis_cmd` | 不变 | — |

navigator 不再直接发 `_root`，改为把 `{goal_poses}` 放黑盒，由 BT 决定何时发布。

### 4.2 `PipelineController`（新增，`finenav_navigator`）

封装 `TaskMaster` + `ControlPipelineTree` + `_root` publisher，暴露：

```cpp
class PipelineController {
public:
    void publishGoal(const std::vector<geometry_msgs::msg::Pose>& poses);  // 发到 _root/__feedback
    void startBranch(const std::string& to);   // 停 off-path + 起 on-path（拓扑序）
    void stopBranch(const std::string& to);    // 停该路径上的 command-driven 层
    void startLayer(const std::string& name);
    void stopLayer(const std::string& name);
    void stopAll();
    std::shared_ptr<const FeedbackMsg> feedback(const std::string& name);
    std::vector<std::string> pathTo(const std::string& to);  // _root → ... → to（唯一）
    std::vector<std::string> leaves();
};
```

### 4.3 BT 节点

| 节点 | 类型 | 端口 | 语义 |
|---|---|---|---|
| **`PublishGoal`** | SyncAction | `goal_poses` (Pose[]) | 发布到 `_root/__feedback` →「开始导航」 |
| **`StartBranch`** | StatefulAction | `to`（目标节点/leaf） | 一键启动支路：停 off-path + 起 on-path（拓扑序），monitor 目标 leaf 的 feedback |
| **`ComputePlan`**（重设计） | StatefulAction | `layer`（去 goal/planner_id） | per-layer 手动 start + monitor + `onHalted()` 补 `stop` |
| `StopPlan` | SyncAction | `layer` | 保留（`stopLayer`） |
| `StopChassisMotion` | SyncAction | — | 保留 |

`StartBranch` = 高层支路开关；`ComputePlan` = 低层逐层控制；底层都走 `master->start/stop`，`StartBranch` 额外用 `pipe_` 算路径。

**注意**：Step 3.5 之后，Reactive 层**也受 START/STOP 控制**（§9），因此 `StartBranch` 对**三种 policy 一视同仁**：START = 点亮该层（Reactive 开始监听、Single/Continuous 开始规划），STOP = 熄灭该层。Reactive 层没有「完成」语义（它只做数据变换），所以 `StartBranch`/`ComputePlan` 监控 leaf 时仍应监控 **command-driven 的 leaf**；Reactive 中间层只做启停、不参与「完成判定」。

### 4.4 示例 XML

`NavigateToPose`（单支路 MPPI，一键式）：

```xml
<BehaviorTree ID="NavigateToPose">
  <Sequence>
    <PublishGoal goal_poses="{goal_poses}"/>        <!-- 开始导航：注入 goal_poses[] -->
    <ReactiveFallback>
      <IsGoalReached .../>                          <!-- 到达 → 成功 -->
      <StartBranch to="mppi_layer"/>                <!-- 一键启动整条支路并持续跟踪 -->
    </ReactiveFallback>
    <StopChassisMotion/>
  </Sequence>
</BehaviorTree>
```

**多支路切换**（一键选支路）：

```xml
<ReactiveFallback>
  <StartBranch to="mppi_layer"/>    <!-- 支路 A；A 失败自动停 A、起 B -->
  <StartBranch to="rot_layer"/>     <!-- 支路 B -->
</ReactiveFallback>
```

---

## 5. 关键决策点（待确认，附推荐默认）

### D1. `StartBranch` 是否自动停「off-path」的其它层？

- **[推荐] 自动停（一键切换）**：`StartBranch(to)` 自动 STOP 所有 off-path 的 command-driven 层，保证任意时刻只有一条路径有效；共享前缀层保持激活。切换支路只需一个节点。
- 只启动、手动停：`StartBranch` 只 START on-path（幂等），切支路要显式 `StopBranch`/`StopPlan`。更显式但样板代码多。

### D2. `StartBranch` 节点形态

- **[推荐] Stateful（启动+等待）**：`onStart` 停旧起新，`onRunning` 轮询目标 leaf feedback（COMPLETED→SUCCESS / FAILED→FAILURE），`onHalted` 停支路。一个节点 =「切到该支路并运行到结束」。
- Sync（只启动）：另设 `WaitLayer`/`WaitBranch` 轮询完成。更模块化但节点更多。

### D3. `ComputePlan` 重设计深度

- **[推荐] 完整重设计**：去掉 `goal`/`planner_id`（只剩 `layer`），补 `onHalted()→stop`，改用 `_pipeline`。XML 同步改。
- 最小改动：保留死端口（继续忽略），只补 `onHalted`。XML 基本不动但语义不干净。

---

## 6. 涉及文件清单

**新增**
- `finenav_navigator/include/finenav_navigator/pipeline_controller.hpp`
- `finenav_navigator/src/pipeline_controller.cpp`
- `finenav_navigator/include/finenav_navigator/plugins/action/publish_goal.hpp`
- `finenav_navigator/src/plugins/action/publish_goal.cpp`
- `finenav_navigator/include/finenav_navigator/plugins/action/start_branch.hpp`
- `finenav_navigator/src/plugins/action/start_branch.cpp`
- （可选）`stop_branch.hpp/.cpp`

**修改**
- `finenav_navigator/include/finenav_navigator/navigator.hpp` / `src/naviagtor.cpp`
  - 建 `PipelineController`；黑盒设 `{goal_poses}` + `_pipeline`；删硬编码 `_root` 发布；`root_feedback_pub_` 移入 controller；评估移除 `master()` 访问器（带 TODO）。
- `finenav_navigator/include/finenav_navigator/plugins/action/compute_plan.hpp` / `src/plugins/action/compute_plan.cpp`
  - 去 `goal`/`planner_id`，补 `onHalted`，改 `_pipeline`。
- `finenav_navigator/src/plugins/action/stop_plan.cpp` — 改 `_pipeline`。
- `finenav_navigator/behavior_trees/navigate_to_pose.xml` / `test_tree.xml` — 用新接口。
- `finenav_navigator/CMakeLists.txt` — 注册新源文件（若显式列举）。

**关联（本次不处理，遗留）**
- `finenav_map/occupancy_grid_2d`、`temporal_voxel_map`、`finenav_planner/*` 仍在用旧 `FINENAV_REGISTER_ALGO_CONFIG` + Eigen 风格 `robot_state`，需后续迁移（与 Step 4 无关）。

---

## 7. 实施与验证步骤

1. 实现 `PipelineController`。
2. 实现 `PublishGoal` / `StartBranch`，重设计 `ComputePlan`，改 `StopPlan`。
3. 改 `navigator`（黑盒契约 + 删硬编码发布）。
4. 更新两个 BT XML。
5. 编译 `finenav_navigator finenav_engine finenav_example finenav_core finenav_util finenav_interfaces`（注意：不涉及坏掉的 map/planner 包）。
6. 冒烟验证：`NavigateToPose` 单支路 + 多支路切换行为正确。

---

## 8. 待办 / 开放项

- [ ] D1 / D2 / D3 三个决策点确认。
- [ ] 确认 BehaviorTree.CPP v4 对 `std::vector<geometry_msgs::msg::Pose>` 作为端口/黑盒类型的支持（若不行，`PublishGoal` 直接 `blackboard->get<std::vector<Pose>>("goal_poses")` 绕过端口注册）。
- [ ] 确认 `_pipeline` 用裸指针还是 `shared_ptr` 放黑盒（建议裸指针，与现有 `_nav_task_master` 一致，生命周期由 navigator 保证）。
- [ ] `master()` 访问器（`navigator.hpp` 里的 TODO）去留。

---

## 9. Step 3.5 — 让 ReactivePolicy 也受 TaskCmd 控制（先于 Step 4 落地）

### 9.1 目标

当前 ReactivePolicy 是「始终在线」的数据变换：构造即建 `feedback_pub_`，`onPlanRef` 里无条件 `react()`，**没有 TaskWorker、不订阅 `__cmd`、不受 START/STOP 控制**（`createLayerByName` 里还专门 `if (!reactive) manageLayer(...)` 把它排除在 TaskMaster 之外）。

Step 3.5 之后，**三种 policy 一视同仁**：

| policy | START 语义 | STOP 语义 |
|---|---|---|
| SingleShot | 执行一次 `planOnce()` → COMPLETED/FAILED | 中断 → FAILED |
| Continuous | 以 N Hz 循环 `planOnce()` 直到 STOP | 中断 → FAILED |
| **Reactive** | **开始监听上游 ref；每个 ref 到达立即 `react()`（planOnce + 发 RUNNING）** | **停止监听（不再 react）；不发终止状态** |

「Reactive 持续监听输入，一旦输入来了立刻执行；TaskCmd 仅控制启停」——STOP 后到 START 之间到达的 ref 只被暂存（`storePlanRef` 照常），下次 START 后的第一个 ref 再触发。

### 9.2 实现要点（最小改动，4 个文件）

1. **`control_layer.hpp`**
   - 删 `void startReactive();`，改为 `void runReactive();`（episode body）。
   - 删 `feedback_pub_` 成员（统一走 `worker_->publishRunning(...)`）。
   - 增 `std::atomic<bool> started_{false};`（Reactive 监听门控）。
   - 更新类注释：ReactivePolicy 不再「always-on」，而是 command-driven 的「监听型」。

2. **`control_layer_impl.hpp`**
   - 构造函数 `std::visit` 的 Reactive 分支：`startWorker([this] { runReactive(); });`（不再 `startReactive()`）。
   - `onPlanRef`：`storePlanRef` 后，Reactive 分支改为 `if (started_.load()) react();`。
   - `react()`：`feedback_pub_->publish(...)` 改为 `worker_->publishRunning(std::move(*out));`。
   - 新增：
     ```cpp
     void runReactive() {
         started_.store(true);
         while (rclcpp::ok() && !worker_->shouldStop())
             std::this_thread::sleep_for(std::chrono::milliseconds(1));
         started_.store(false);
     }
     ```
     （`startWorker` 已包好 `hasStart → ackStart → waitForState → resetPlanner → episode_body`，所以 START 时自动等状态 + reset 规划器；`runReactive` 只是把「监听开关」打开、阻塞到 STOP、再关上。）

3. **`execution_policy.hpp`**：更新 `ReactivePolicy` 注释 —— 从「data-driven, no TaskWorker」改为「START/STOP 控制的监听型」。

4. **`finenav_engine.cpp`**：`createLayerByName` 删掉 `reactive` 特判，**无条件** `navigator_->master().manageLayer(layer_name);`（模板版 `createLayer` 本来就无条件 manage，对齐）。

### 9.3 语义/线程注意事项

- `react()` 在订阅回调线程（pnode executor）里调 `worker_->publishRunning`，与 worker 线程**不共享可变状态**（TaskWorker 只发布消息，ROS pub 线程安全），安全。
- 路由改走 `worker_->publishRunning` 后，Reactive 不再自己建 `feedback_pub_`，避免同一个 `layer/__feedback` 出现两个 publisher。
- 生命周期：析构里 `stop_=true` + join worker 线程；`runReactive` 只认 `shouldStop()/rclcpp::ok()`，与 `runContinuous` 同款（即：跟现有 Continuous 一样，依赖 rclcpp 关闭或显式 STOP 退出，非本步新增风险）。
- `started_` 在 worker 线程写、回调线程读，`atomic` 即可；`runReactive` 退出时置 false，天然关掉后续反应。

### 9.4 验证

- 编译 `finenav_core finenav_engine finenav_util finenav_interfaces`（不碰坏掉的 map/planner 包）。
- 冒烟：一条 `_root >> reactive_mid >> mppi` 链，BT 只 START mppi 时 reactive_mid 不 react；START reactive_mid + mppi 时，注入 ref 后 reactive_mid 立刻转发、mppi 收到。
- `createLayerByName` 对 ReactivePolicy 也注册了 `__cmd`（master 能 start/stop 它）。
