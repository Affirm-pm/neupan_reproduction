# 3. 编写 WorldView

WorldView 是连接 Map 与 Planner 的桥梁，负责将 Map 提供的地图信息转换为 Planner 可直接使用的信息，例如代价值。
用户需要编写 WorldView，将选定的 Map 与 Planner 插件组装在一起。

## Planner 接口

Planner 通过定义 `IWorldView` 抽象类，规定用户编写的 WorldView 需要提供哪些接口。例如，
`DummyLinePlanner` 在类内声明接口：

```cpp
class DummyLinePlanner {
  public:
    class IWorldView {
      public:
        virtual ~IWorldView() = default;
        virtual bool isWalkable(const finenav::common::Position3D& position) const = 0;
    };
    // ...
};
```

`DummyFollowPlanner` 也独立声明 `IWorldView`，目前包含相同的查询。
不同 Planner 定义的 `IWorldView` 是不同的接口类型，需要分别实现。
用户需要编写继承对应 `IWorldView` 的具体类，实现所需的 WorldView 逻辑，将地图信息适配为该 Planner 所需的查询结果。

## 实现 WorldView 继承类

下面的 `DummyLineWorldView` 继承 `DummyLinePlanner::IWorldView`，保存所选 Map 的引用，并实现 Planner 要求的可通行查询。
该类位于 `finenav::example` 命名空间中：

```cpp
class DummyLineWorldView : public DummyLinePlanner::IWorldView {
  public:
    // 继承类使用的参数类型，由 YAML 参数定义生成。
    using ConfigType = dummy_line_world_view::Params;

    // 保存 Map 的只读引用，供后续查询使用。
    DummyLineWorldView(rclcpp::Node::SharedPtr, const DummyMap& map) : map_(map) {}

    // 接收适配参数。本例没有需要应用的参数，因此保留空实现。
    void configure(const ConfigType&) {}

    // 在每轮规划前准备所需数据。本例直接查询地图，无需额外预处理。
    void prepare(const finenav::core::PlanningContext<DummyLinePlanner>&) {}

    // 实现 Planner 声明的接口，将占据状态转换为可通行判断。
    bool isWalkable(const finenav::common::Position3D& position) const override {
        const bool occupied = map_.isOccupied(position);
        return !occupied;
    }

  private:
    const DummyMap& map_;
};
```

这里的转换规则是：Map 返回“已占据”时，WorldView 返回“不可通行”；Map 返回“未占据”时，WorldView 返回“可通行”。
编写自己的 WorldView 时，需要按照对应 `IWorldView` 中每个查询的含义，使用所选 Map 提供的数据实现它。
`configure()` 和 `prepare()` 也需要提供；是否在其中添加处理逻辑，取决于实际的参数和数据准备需求。

## 类型绑定与插件注册

注册文件将每个适配类与一个 Planner、所选 Map 绑定：

```cpp
FINENAV_REGISTER_WORLDVIEW(finenav::example::DummyLineWorldView, dummy_line_world_view,
                           finenav::example::DummyLinePlanner, finenav::example::DummyMap)

FINENAV_REGISTER_WORLDVIEW(finenav::example::DummyFollowWorldView, dummy_follow_world_view,
                           finenav::example::DummyFollowPlanner, finenav::example::DummyMap)
```

这里的注册名就是应用调用 `createControlLayer()` 时传入的名称。
参数定义为 CMake 文件旁边的 `dummy_line_world_view.yaml` 和 `dummy_follow_world_view.yaml`。
构建时需要生成参数代码、编译并安装共享库，再登记插件供 Engine 加载。关键语句如下：

```cmake
# 生成两个 WorldView 的参数代码。
generate_parameter_library(dummy_line_world_view_params dummy_line_world_view.yaml)
generate_parameter_library(dummy_follow_world_view_params dummy_follow_world_view.yaml)

# 将 WorldView 实现编译为共享库。
add_library(dummy_world_view SHARED src/dummy_world_view.cpp)

# 安装共享库，并登记为 Engine 可发现的算法插件。
install(TARGETS dummy_world_view LIBRARY DESTINATION lib)
finenav_register_algo_plugin(dummy_world_view)
```

完整配置（含头文件路径、依赖链接和安装规则）见
[CMakeLists.txt](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_world_view/CMakeLists.txt)。

相关源代码位于[适配类](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_world_view/include/dummy_world_view.hpp)、
[类型注册](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_world_view/src/dummy_world_view.cpp)、
[CMake 配置](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_world_view/CMakeLists.txt)。

下一步：[4. 组装控制 Pipeline](cpp-application.md)。
