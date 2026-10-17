# Custom Map

Write a custom Map when you need another environment representation or data source.
Start from the
[Map example](https://github.com/FineNav/FineNav-Engine/tree/81c8f277518fb9229eb5e78be43375227490ce39/finenav_example/create_custom_map).

## Define the data and queries

Choose the data your Map stores and the queries it exposes: occupancy, distance,
terrain height, or other information needed by a WorldView.
The example's basic shape is:

```cpp
using ConfigType = dummy_map::Params;
using DataType = int;

void configure(const ConfigType& cfg);
bool isOccupied(const finenav::common::Position3D& p) const;
void selfManage(finenav::core::MapRuntime& runtime);
```

Replace the example `DataType` with your representation. Apply parameters in
`configure()` and implement useful queries over your stored data.

## Connect observations

Use `selfManage()` to register observation sources through `MapRuntime`.
Its `addObservationSource<MsgT>()` API takes a source name, topic, QoS,
update policy, and callback. The callback receives the message, timestamp,
and robot state so you can update your Map from sensor data.
Maps that need no observation sources can leave `selfManage()` empty.

## Package and use it

Define the parameter YAML, generate its parameter library, and build the Map
as a shared library. Register it in a `.cpp` file:

```cpp
FINENAV_REGISTER_MAP(finenav::example::DummyMap, dummy_map)
```

For your plugin, replace the example class and namespace with your own and keep
the registration namespace consistent with the YAML schema.
The example directory includes the CMake configuration.

Finally, [write a WorldView](worldview.md) that translates your Map's
queries into the selected Planner's interface. Load the plugins and select that
WorldView in your [C++ application](cpp-application.md).