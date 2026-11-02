# finenav_map

地图组件维护在 [FineNav/finenav_map](https://github.com/FineNav/finenav_map)。

## 下载

```bash
mkdir -p ~/finenav_ws/src
cd ~/finenav_ws
git clone https://github.com/FineNav/finenav_map.git src/finenav_map
```

## 组件目录

| ROS 包 | Map 注册名 | 用途 |
| --- | --- | --- |
| `Temperal_voxel_map` | `Temperal_voxel_map` | 基于点云更新的滚动时空体素地图，支持过期占据体素清除。 |
| `finenav_grid_map` | — | 可复用的三维滚动栅格容器，为自定义 Map 提供底层数据结构与存储操作。 |

`finenav_grid_map` 提供栅格索引、坐标转换、存储和窗口移动等基础能力。
它本身不是实现框架 Map 接口的地图插件；开发自定义 Map 时可按需复用，
框架不限定 Map 的底层数据结构，也不要求依赖该包。

具体接口和参数见对应目录的 README：
[时空体素地图](https://github.com/FineNav/finenav_map/tree/main/Temperal_voxel_map)、
[栅格容器](https://github.com/FineNav/finenav_map/tree/main/finenav_grid_map)。

## 示例 Map

Getting Started 使用 ROS 包 `finenav_example` 中的 `dummy_map`。
它没有观测数据源，`isOccupied()` 始终返回 false。
示例中的两个 WorldView 共享同一个 Map 服务，适合练习组件接入，不提供障碍物感知导航。

参数定义中有只读的 `window_size`，但当前 Dummy 实现的查询边界仍为固定值，
不宜用它演示地图窗口尺寸调整。

需要实现自己的环境表示和数据接入时，继续阅读[自定义 Map](../custom-map.md)。
