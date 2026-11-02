# 5. Configure a Behavior Tree

The pipeline defines how to compute commands. The Behavior Tree defines when
to execute it, what counts as arrival, and how to respond to a failure.
This section uses `finenav_example/getting_started/behavior_trees/navigate_to_pose.xml` as an example to explain how to configure a Behavior Tree.

## Example behavior tree

```xml
<root BTCPP_format="4">

    <BehaviorTree ID="GettingStartedNavigate">
        <Sequence>
            <PublishNavigationGoal/>

            <RecoveryNode number_of_retries="3">
                <SubTree ID="DriveToGoal"/>

                <!-- Control branch FAULTED: recovery -->
                <Sequence name="Recover">
                    <GetFaultedControlLayer control_layer="{faulted_layer}"/>
                    <StopChassisMotion/>
                    <RestartFromControlLayer control_layer="{faulted_layer}"/>
                </Sequence>
            </RecoveryNode>
        </Sequence>
    </BehaviorTree>


    <BehaviorTree ID="DriveToGoal">
        <Sequence>
            <ReactiveFallback name="NavigateOrSucceed">
                <IsGoalReached  xy_tolerance="0.25"
                                yaw_tolerance="0.1"
                                speed_tolerance="0.1"/>
                <ExecuteControlBranch branch="navigate"/>
            </ReactiveFallback>
            <!-- Goal reached: publish stop chassis. -->
            <StopChassisMotion/>
        </Sequence>
    </BehaviorTree>

</root>
```

## Loading and execution

The application entry point, `hello_finenav.cpp`, sets `GettingStartedNavigate` as the default tree:

```cpp
engine->setDefaultBehaviorTree("GettingStartedNavigate");
```

`GettingStartedNavigate` must match the main tree's `ID` in the loaded XML.
The navigator uses this default tree when a navigation request does not specify a tree ID.

The navigator loads the XML from the directory configured in YAML:

```yaml
/hello_finenav/finenav_navigator:
  ros__parameters:
    behavior_trees:
      - "finenav_example/getting_started/behavior_trees"
```

[4. Assemble a control pipeline](cpp-application.md) registers a control branch with `registerControlBranch("navigate", ...)`.
Registering a branch defines the connections between control layers; the Behavior Tree activates it with `ExecuteControlBranch`.
The XML attribute `branch="navigate"` must match the branch name registered in `hello_finenav.cpp`.

## Node reference

| Node | Purpose |
| --- | --- |
| `PublishNavigationGoal` | Supplies the requested goal to the first layer. |
| `ReactiveFallback` | Rechecks arrival while the branch is running. |
| `IsGoalReached` | Checks the configured position, orientation, and speed tolerances. |
| `ExecuteControlBranch branch="navigate"` | Activates the C++-registered branch; stays RUNNING until halted or a layer faults. |
| `StopChassisMotion` | Sends a stop command when reached, or before recovery. |
| `RecoveryNode` | Retries the navigation child after the recovery child succeeds, up to the configured retry count. |
| `GetFaultedControlLayer` + `RestartFromControlLayer` | Selects the faulted layer and requests restarting that layer and its downstream layers. |

`ExecuteControlBranch` does not decide that a goal is reached. Keep a completion
condition such as `IsGoalReached` so the task can finish.

Recovery behaviors are configured in the Behavior Tree XML file. The recovery sequence shown here is an example; configure the actual recovery behavior according to your operating scenario and task requirements.

## Task configuration

| Requirement | Change |
| --- | --- |
| Accept a larger or smaller arrival region | Adjust `xy_tolerance` in metres. |
| Require a particular terminal heading | Use a positive `yaw_tolerance` in radians and a controller that can align at the goal. |
| Require a stopped robot | Use positive `speed_tolerance` in m/s and a controller that brakes before success. |
| Retry a different number of times | Change `number_of_retries`. |
| Execute another pipeline | Register it in C++, then use its exact name in `branch`. |

!!! note "Additional information"

    The dummy follower is a simplified implementation for this example. It does not implement final-heading alignment or distance-based braking near the goal. The example Behavior Tree sets both `yaw_tolerance="0.1"` and `speed_tolerance="0.1"`, so the task may remain active near the goal if the heading or speed condition is not met. For a **position-only simulation exercise**, replace the existing `IsGoalReached` element inside `DriveToGoal` with:

    ```xml
    <IsGoalReached xy_tolerance="0.25"
                   yaw_tolerance="0.0"
                   speed_tolerance="0.0"/>
    ```

For this node, zero disables the corresponding check. Once position is reached,
the tree halts the branch and calls `StopChassisMotion`.

The relevant source code is available in the [example tree](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/getting_started/behavior_trees/navigate_to_pose.xml) and
[completion checks](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_navigator/src/plugins/condition/is_goal_reached.cpp).

Next, explore [finenav_planner](plugins/planners.md) and [finenav_map](plugins/maps.md).
