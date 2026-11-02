# finenav_map

Map components are maintained in
[FineNav/finenav_map](https://github.com/FineNav/finenav_map).

## Download

```bash
mkdir -p ~/finenav_ws/src
cd ~/finenav_ws
git clone https://github.com/FineNav/finenav_map.git src/finenav_map
```

## Component catalogue

| ROS package | Registered Map | Purpose |
| --- | --- | --- |
| `Temperal_voxel_map` | `Temperal_voxel_map` | Rolling spatio-temporal voxel Map updated from point clouds, with stale occupancy clearing. |
| `finenav_grid_map` | — | Reusable rolling 3D grid container providing data structures and storage operations for custom Maps. |

`finenav_grid_map` provides grid indexing, coordinate conversion, storage, and
window shifting. It does not itself implement the framework's Map interface.
Custom Maps can reuse it as needed; the framework does not prescribe an
underlying data structure or require a dependency on this package.

Package READMEs describe their interfaces and parameters:
[spatio-temporal voxel Map](https://github.com/FineNav/finenav_map/tree/main/Temperal_voxel_map),
[grid container](https://github.com/FineNav/finenav_map/tree/main/finenav_grid_map).

## Example Map

Getting Started uses `dummy_map` from ROS package `finenav_example`.
It has no observation sources, and `isOccupied()` always returns false.
Both example WorldViews share its one Map server. It is useful for integration
exercises, not obstacle-aware navigation.

The schema exposes `window_size` as read-only, but the current dummy implementation
keeps fixed query bounds; do not use that setting as a demonstration of a resized map.

For your own representation and sensor inputs, continue with
[Custom Map](../custom-map.md).
