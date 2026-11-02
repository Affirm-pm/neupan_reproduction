# 4. 组装控制 Pipeline

选定 Map、Planner 并完成 WorldView 后，应用代码只需加载插件包、根据 WorldView 来创建控制层，再确定各个控制层之间的连接顺序。

## 示例应用

以下为 `finenav_example/getting_started/hello_finenav.cpp` 中的 Pipeline 组装片段：

```cpp
using namespace finenav;

auto node = rclcpp::Node::make_shared("hello_finenav");
auto engine = std::make_shared<finenav::FineNavEngine>(node);

engine->loadAlgoPlugins({"finenav_example"});

engine->createControlLayer(
    "dummy_line_world_view",
    "line_layer",
    core::EventDrivenPolicy{});

engine->createControlLayer(
    "dummy_follow_world_view",
    "follow_layer",
    core::PeriodicPolicy{10.0});

engine->registerControlBranch(
    "navigate",
    {"line_layer", "follow_layer"});
```

## 数据流

```mermaid
flowchart LR
    G[导航目标] --> L["line_layer · 事件驱动"]
    L -->|位姿路径| F["follow_layer · 10 Hz"]
    F --> C[/cmd_vel/]
```

| 调用 | 作用 |
| --- | --- |
| `loadAlgoPlugins({"finenav_example"})` | 发现包内已安装的 Map、Planner、WorldView 插件库。 |
| `createControlLayer(..., "line_layer", EventDrivenPolicy{})` | 有新上游参考输入时生成路径。 |
| `createControlLayer(..., "follow_layer", PeriodicPolicy{10.0})` | 按目标 10 Hz，根据最新路径和机器人状态输出跟踪指令。 |
| `registerControlBranch("navigate", {"line_layer", "follow_layer"})` | 按顺序连接两层，注册名为 `navigate` 的分支。 |

第一层接收导航目标，第二层接收第一层生成的位姿路径。

这些组装接口返回 `bool`。最小示例省略了错误处理；编写自己的应用时，可以通过检查返回值，在失败时停止初始化。

## 参数配置

`hello_finenav.launch.py` 加载 `hello_finenav.yaml`，以下为与 Pipeline 相关的参数摘录：

```yaml
/hello_finenav:
  ros__parameters:
    use_sim_time: true  # 使用 /clock 提供的仿真时间。

/hello_finenav/finenav_localizer:
  ros__parameters:
    pose_topic: ""  # 绝对位姿输入话题；空字符串表示不启用。
    pose_smoothing_time: 0.0  # 位姿修正的平滑时间，单位秒；0 表示立即应用修正。
    initial_pose_topic: ""  # 一次性初始位姿输入话题；空字符串表示不使用。
    twist_topic: ""  # 机器人坐标系下的速度输入话题；空字符串表示不启用。

    odom_topic: "/odom"  # 里程计输入话题，用于更新机器人位姿。
    odom_use_twist: true  # 同时使用输入里程计中的 twist 更新机器人速度。

    state_history_duration: 5.0  # 机器人状态历史的保留时长，单位秒，用于按时间戳查询。

    force_2d_mode: true  # 是否使用2D模式，将位姿的 z、横滚角和俯仰角约束为 0。
    publish_tf: false  # 是否发布世界坐标系到机器人坐标系的 TF。
    publish_odom: true  # 将估计状态以 Odometry 消息发布到 ~/nav_state。

/hello_finenav/dummy_map:
  ros__parameters:
    dummy_map:
      window_size: 50.0  # 地图窗口半边长参数，单位米；本例的地图边界固定。

/hello_finenav/line_layer:
  ros__parameters:
    dummy_line_planner:
      path_resolution: 0.2  # 生成直线路径时的采样点间距，单位米。

/hello_finenav/follow_layer:
  ros__parameters:
    dummy_follow_planner:
      linear_speed: 0.5  # 基准前进速度，单位米/秒；航向误差增大时会降低速度。
      angular_speed: 1.0  # 将航向误差转换为角速度的比例增益。
      lookahead_distance: 1.0  # 沿路径选择跟踪目标点的前瞻距离，单位米。
```

`/hello_finenav/line_layer` 和 `/hello_finenav/follow_layer` 来自 `hello_finenav.cpp` 中注册的控制层名字；
`dummy_line_planner`、`dummy_follow_planner` 是各算法的参数命名空间。
修改控制层名字后，需要同步修改 YAML 中对应的节点路径。
定位器读取 `/odom`，默认底盘接口向 `/cmd_vel` 发布 `geometry_msgs/msg/Twist`。

运行时可以动态调整相关参数。以 `DummyFollowPlanner` 执行路径跟踪时的基准速度为例，
可通过以下命令将 `linear_speed` 设置为 `0.3`，单位为米/秒：

```bash
ros2 param set /hello_finenav/follow_layer dummy_follow_planner.linear_speed 0.3
```

Pipeline 负责按照控制层的连接顺序完成路径规划和跟踪，生成底盘运动指令；行为树负责选择和启动控制分支、判断任务是否完成，以及处理执行过程中的故障。
关于行为树的配置与使用，见下一节 [5. 按需求配置行为树](behavior-tree.md)。

完整示例与接口定义可参考：

- [应用入口（hello_finenav.cpp）](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/getting_started/hello_finenav.cpp)：查看 Engine 初始化、插件加载、控制层创建及分支注册的完整代码。
- [运行配置（hello_finenav.yaml）](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/config/hello_finenav.yaml)：查看示例启动时加载的完整参数，包括定位器、Map 和 Planner 的配置。
- [公开 API（finenav_engine.hpp）](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_engine_cpp/include/finenav_engine_cpp/finenav_engine.hpp)：查询 `loadAlgoPlugins()`、`createControlLayer()` 和 `registerControlBranch()` 等接口的参数与返回值。
