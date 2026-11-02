# 自定义 Planner

已有算法不满足机器人或任务需求时，可以实现自己的 Planner。
从 [Planner 示例](https://github.com/FineNav/FineNav-Engine/tree/main/finenav_example/create_custom_planner)开始。

## 定义接口约定

在每个 Planner 内定义自己的 `IWorldView`，仅包含该算法需要的环境查询，
使算法不依赖具体 Map。即使两个算法当前使用相同的查询，也分别定义接口，
之后通过各自的 WorldView 适配所选地图。

分别声明每一层的输入和输出。直线规划层使用位姿输入、位姿输出：

```cpp
static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
static constexpr finenav::core::IOProfile OutputProfile{true, false, false, false};
```

跟踪层消费位姿路径，输出速度：

```cpp
static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
static constexpr finenav::core::IOProfile OutputProfile{false, true, false, false};
```

## 实现算法

在 `DummyLinePlanner` 类内，接口约定如下：

```cpp
class IWorldView {
  public:
    virtual ~IWorldView() = default;
    virtual bool isWalkable(const finenav::common::Position3D& position) const = 0;
};

using ConfigType = dummy_line_planner::Params;
using Context = finenav::core::PlanningContext<DummyLinePlanner>;
using Result = std::optional<finenav::core::OutputTrajectoryT<DummyLinePlanner>>;

void configure(const ConfigType& config);
void reset();
Result plan(const Context& context, const IWorldView& world_view);
```

在 `configure()` 中应用参数，在 `reset()` 中清理本次执行状态，
在 `plan()` 中根据 `robot_state`、`ref_traj` 和 WorldView 计算输出。
有效输入无法产生结果时返回 `std::nullopt`，这表示失败。

通过 `add_library(... SHARED ...)` 构建共享库并安装到 `lib`；
父级软件包查找 `finenav_cmake`，并在包声明中加入构建依赖。完整配置见示例 CMake 文件。

## 注册并发布插件

定义参数 YAML，生成参数库。YAML 命名空间、生成的参数命名空间与注册名应保持一致：

```cpp
FINENAV_REGISTER_PLANNER(
    finenav::example::DummyLinePlanner,
    dummy_line_planner)
```

通过 ament 资源发布共享库：

```cmake
finenav_register_algo_plugin(dummy_line_planner)
```

然后为所选 Map [编写 WorldView](worldview.md)，在 [C++ 应用](cpp-application.md)中组装使用。
