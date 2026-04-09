# Plan: FineNav Localizer Subsystem (ESKF-based Smooth Localization)

实现一个位于 `finenav_core/localization/` 的定位子库，核心由一个通用 ESKF 滤波器驱动，状态向量为 `[p, R, v, ω]`（位置、旋转、线速度、角速度），通过定时器驱动的 predict-then-update 架构和 `smoothing_steps` 参数实现平滑输出。整个子库为纯 C++ API 层（非 ROS2 Node），但内部使用 rclcpp 定时器和日志。

## 文件结构

```
finenav_core/
  include/finenav_core/localization/
    eskf.hpp                  ✅ 已完成（通用 ESKF 类模板）
    filter_model.hpp          ✅ 已完成（状态向量 / 噪声 / 观测模型定义）
    observation_queue.hpp     🔲 待实现（泛型观测缓存队列）
    finenav_localizer.hpp     🔲 待实现（定位滤波器主逻辑声明）
  src/localization/
    finenav_localizer.cpp     🔲 待实现（定位滤波器主逻辑实现）
```

---

## Steps

### Step 1 — 创建泛型观测队列 `observation_queue.hpp`

新建 `finenav_core/include/finenav_core/localization/observation_queue.hpp`，实现 `ObservationQueue<MsgT>` 类模板：

- 内部用 `std::deque<Entry>` 存储 `{rclcpp::Time stamp, MsgT msg}`，`std::mutex` 保护。
- 提供接口：
  - `push(const MsgT&)` — 从消息 header 自动提取时间戳
  - `drainAll() → std::vector<Entry>` — 原子清空并返回，按时间戳升序排列
  - `size()`, `clear()`, `empty()`
- 构造时指定 `max_queue_size`（默认 32）和 `smoothing_steps`（默认 1）；超出容量时弹出最旧元素并输出 `RCLCPP_WARN`。
- `smoothing_steps` 不在队列内部消费，而是由 `FineNavLocalizer` 在应用观测时读取，用于将 R 放大 N 倍（`R_eff = R * smoothing_steps`）。

---

### Step 2 — 声明 `FineNavLocalizer` 类（`finenav_localizer.hpp`）

新建 `finenav_core/include/finenav_core/localization/finenav_localizer.hpp`：

#### 构造参数（`LocalizerOptions` 结构体）

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `update_rate_hz` | `double` | `100.0` | 定时器频率 |
| `max_delay_sec` | `double` | `0.5` | 最大容忍延迟，超出丢弃 |
| `pose_smoothing_steps` | `int` | `1` | Pose 观测协方差放大倍数 |
| `twist_smoothing_steps` | `int` | `1` | Twist 观测协方差放大倍数 |
| `pose_gate_dist` | `double` | `12.59` | Pose 马氏门控阈值（χ²(6), 95%） |
| `twist_gate_dist` | `double` | `12.59` | Twist 马氏门控阈值（χ²(6), 95%） |
| `max_dt_sec` | `double` | `0.5` | dt 钳制上限，防止时间跳变 |
| `model_opts` | `NavigationModelD::Options` | `{}` | ESKF 过程噪声参数 |

#### 构造逻辑

仿照 `MapServer` 构造函数：
- 创建子节点 `pnode_`（名称 `"finenav_localizer"`）、`NodeThread pnode_thread_`
- 创建 `ESKFd eskf_`（初始状态为零）
- 创建 `tf2_ros::TransformBroadcaster tf_br_`（使用 `pnode_`）
- 创建 `ObservationQueue<PoseWithCovarianceStamped> pose_queue_`
- 创建 `ObservationQueue<TwistWithCovarianceStamped> twist_queue_`
- 创建 `update_timer_`（周期 = `1.0 / update_rate_hz`，绑定 `timerCallback()`）

#### 内部类型：`StateSnapshot`

```cpp
struct StateSnapshot {
    rclcpp::Time   stamp;
    NavStateD      state;
    PMat           covariance;
};
```

用 `std::deque<StateSnapshot> state_history_` 维护滚动历史（按 `max_delay_sec` 裁剪旧条目）。

#### 公开 C++ API

```cpp
// 重置滤波器（清空历史、重置协方差）
void setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);

// 推入观测队列（线程安全）
void feedPose (const geometry_msgs::msg::PoseWithCovarianceStamped&  msg);
void feedTwist(const geometry_msgs::msg::TwistWithCovarianceStamped& msg);

// 读取最新估计（shared_mutex 保护）
NavStateD      getState()           const;
PMat           getCovariance()      const;
rclcpp::Time   getLastUpdateTime()  const;
```

---

### Step 3 — 实现 `FineNavLocalizer::timerCallback()`（`finenav_localizer.cpp`）

核心更新流程：

1. **计算 dt**：与上次 predict 的时间差；首次调用用 `1.0 / update_rate_hz`。若 `dt > max_dt_sec`，钳制并输出 `RCLCPP_WARN`；若 `dt < 0`，跳过并警告（时间跳变）。

2. **Predict**：调用 `eskf_.predict(dt)`。

3. **保存快照**：将 `{now, eskf_.getState(), eskf_.getCovariance()}` 推入 `state_history_`；裁剪超出 `max_delay_sec` 的旧条目。

4. **排空 Pose 观测队列**（`pose_queue_.drainAll()`），对每条观测：
   - 计算 `delay = now - obs.stamp`；若 `delay > max_delay_sec`，丢弃并 `RCLCPP_WARN`。
   - 在 `state_history_` 中二分查找最接近 `obs.stamp` 的快照，用该历史状态的 `NavStateD` 和协方差计算（调用 `NavigationModelD::observePose`）。
   - **马氏门控**：`d² = innovᵀ · S⁻¹ · innov`，其中 `S = H · P_current · Hᵀ + R_eff`；若 `d² > pose_gate_dist`，跳过并 `RCLCPP_WARN`。
   - 调用 `eskf_.update<6>(innovation, H, R_eff)`，`R_eff = R_obs * pose_smoothing_steps`。

5. **排空 Twist 观测队列**，逻辑同上，使用 `NavigationModelD::observeVelocity` 和 `twist_gate_dist` / `twist_smoothing_steps`。

6. **用写锁更新对外暴露的状态快照与 `last_update_time_`**。

7. **广播 TF**：从 `eskf_.getState()` 的 `p` 和 `R` 构造 `geometry_msgs::msg::TransformStamped`（`frame_id="map"`, `child_frame_id="base_link"`）并发布。

#### 私有辅助函数

```cpp
// PoseWithCovarianceStamped → (pos, rot, R_6x6)
static std::tuple<Vec3d, SO3d, Eigen::Matrix<double,6,6>>
    extractPoseObservation(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);

// TwistWithCovarianceStamped → (lin_vel_body, ang_vel_body, R_6x6)
static std::tuple<Vec3d, Vec3d, Eigen::Matrix<double,6,6>>
    extractTwistObservation(const geometry_msgs::msg::TwistWithCovarianceStamped& msg);
```

---

### Step 4 — 构建配置

#### `finenav_core/CMakeLists.txt`

- 新增编译目标（保留原有 `INTERFACE` 库不变）：
  ```cmake
  ament_auto_add_library(finenav_localizer SHARED
      src/localization/finenav_localizer.cpp
  )
  target_link_libraries(finenav_localizer
      Sophus::Sophus
  )
  ```
- 创建 `src/localization/` 目录。

#### `finenav_core/package.xml`

添加依赖：
```xml
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>tf2_ros</depend>
<depend>tf2_eigen</depend>
```

（`tf2_ros` / `tf2_eigen` 已在 package.xml 中存在，确认 `geometry_msgs` 和 `nav_msgs` 有无缺失。）

---

## Further Considerations

1. **`finenav_core` 库类型**：目前 `finenav_core` 是 `INTERFACE`（header-only）。新增 `.cpp` 需要独立编译目标 `finenav_localizer`，保持原 `INTERFACE` 不变，避免影响已有依赖。

2. **延迟观测回放精度 vs 复杂度（Approach A vs B）**：最终决定采用方案B
   - **Approach A（当前方案）**：对延迟观测用历史状态计算 innovation，但在 *当前* 协方差上做 Kalman 更新。工程上最常见，适合 delay < 200ms 的场景。
   - **Approach B（完整重播）**：找到历史快照、在历史时刻做更新、然后重新 predict 到当前时刻。精度更高，代价是存储与计算开销线性增长。当 delay 超过数个滤波周期时可升级。

3. **马氏门控阈值**：观测维度为 6（pose / twist 各 6-DOF），χ²(6) 的 95% 分位点为 `12.59`，99% 为 `16.81`。建议提供 per-observation-type 的独立阈值，通过 `LocalizerOptions` 可配置。

4. **`smoothing_steps` 语义**：等效于将观测协方差 R 乘以 N，使滤波器在观测变化剧烈时缓慢跟随，实现输出平滑。N=1 时行为退化为标准 ESKF，N 越大平滑效果越强但跟踪延迟也越大。

5. **线程安全边界**：
   - `feedPose()` / `feedTwist()` 在任意外部线程调用，通过 `ObservationQueue` 内部 mutex 保护。
   - `timerCallback()` 运行在 `pnode_thread_` 的 executor 线程，是唯一修改 `eskf_` 和 `state_history_` 的线程，无需额外锁。
   - 对外 getter（`getState()` 等）通过 `shared_mutex cached_state_mutex_` 保护输出快照（缓存在 `timerCallback` 末尾写入）。

6. **初始化保护**：在 `setInitialPose` 被调用之前，`timerCallback` 应跳过 predict/update，仅输出 `RCLCPP_WARN_THROTTLE`，避免用全零状态广播无效 TF（与 `EKFLocalizer` 的 `is_set_initialpose_` 机制对应）。

