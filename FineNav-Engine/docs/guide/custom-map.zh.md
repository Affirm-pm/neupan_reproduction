# 自定义 Map

需要不同的环境表示或观测数据源时，可以实现自己的 Map。
从 [Map 示例](https://github.com/FineNav/FineNav-Engine/tree/main/finenav_example/create_custom_map)开始。

## 定义数据与查询

定义地图数据及 WorldView 所需查询，例如占据、距离或地形高度。
示例的基本结构为：

```cpp
using ConfigType = dummy_map::Params;
using DataType = int;

void configure(const ConfigType& cfg);
bool isOccupied(const finenav::common::Position3D& p) const;
void selfManage(finenav::core::MapRuntime& runtime);
```

将 `DataType` 换成实际数据表示，在 `configure()` 应用参数，实现对存储数据的查询。

## 对接观测数据

在 `selfManage()` 中通过 `MapRuntime` 注册观测源。
`addObservationSource<MsgT>()` 接收源名称、话题、QoS、更新策略和回调；
回调提供消息、时间戳与机器人状态，供 Map 更新数据。
不需要观测源的地图可以保留空的 `selfManage()`。

## 打包与使用

定义参数 YAML，生成参数库，将 Map 构建为共享库，并在 `.cpp` 中注册：

```cpp
FINENAV_REGISTER_MAP(finenav::example::DummyMap, dummy_map)
```

替换为自己的类型和名称，并保持注册命名空间与 YAML 参数定义一致。
示例目录包含 CMake 配置。父级包查找 `finenav_cmake`，安装共享库并发布发现信息：

```cmake
finenav_register_algo_plugin(dummy_map)
```

最小 Map 约定包含配置和 `selfManage()`；窗口跟踪是额外能力，按实际数据表示需要实现。

最后[编写 WorldView](worldview.md)，将 Map 查询转换为所选 Planner 的接口，
并在 [C++ 应用](cpp-application.md)中加载插件、选择 WorldView。
