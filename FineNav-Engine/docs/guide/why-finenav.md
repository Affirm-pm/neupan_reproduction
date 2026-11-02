# Why FineNav-Engine?

FineNav-Engine lets you assemble a layered control architecture around your
requirements. Choose existing or custom Maps and Planners that conform to the
framework interfaces, connect them through
a WorldView, and use the framework APIs to build your navigation system.
The framework also provides a runtime branch-switching mechanism, allowing a
Behavior Tree to select pipelines assembled and registered ahead of time.

## 1. Layered control composition

Each control layer encapsulates its own algorithm and execution policy,
connecting to other layers through defined input and output interfaces.
Algorithm implementations are decoupled across layers. With compatible inputs
and outputs between connected layers, you can choose the
number of layers, their composition, and execution rates to build a control
pipeline suited to your robot.

For example, the tutorial connects a straight-path planning layer to a tracking
layer: the first generates a pose path, and the second continuously produces
velocity commands from it. To change the planning algorithm, keep the tracking
layer and connect a new planning layer with compatible inputs and outputs,
reusing the rest of the workflow.

[Assemble a control pipeline](cpp-application.md).

## 2. Component adaptation and integration

Use Maps and Planners already adapted to the framework, or implement and register
your own map and planning plugins against the framework interfaces.
A WorldView adapts a Map's data queries to the interface a Planner requires,
allowing map representations and planning algorithms to be selected and reused
independently.

Once the components conform to the interfaces and are registered as plugins,
application integration primarily consists of writing
the required WorldView adapter, calling framework APIs to load plugins, create
control layers, and connect branches, then configuring runtime parameters and
task behavior. The framework handles component assembly and execution so you
can focus on algorithm choices and application requirements.

For example, the tutorial Map provides an occupancy query, while the Planner
needs a walkability check. Once the WorldView translates that query, the
application selects the combination by its registered name without handling
map-query details in the main program.

[Write a WorldView](worldview.md).

## 3. Runtime branch switching

A navigation system can assemble and register multiple control branches ahead
of time, then use a Behavior Tree to select and switch the active pipeline at
runtime. Branches can use different layer combinations, with the application's
Behavior Tree logic deciding when to select each branch.

This provides a foundation for organizing multiple navigation strategies within
one application. Validate branch combinations and switching behavior in your
application, particularly data handoff and layer start/stop behavior when
branches share control layers.

[Configure a Behavior Tree](behavior-tree.md).

[Get started with FineNav-Engine](installation.md).
