## The Mythical HFSMBTH

Inspired by the 2019 Game Developers Conference [talk](https://www.youtube.com/watch?v=Qq_xX1JCreI&t=1159s) by Bobby Anguelov, we present the "Mythical HFSMBT Hybrid".

Please use the following publication for reference when using the HFSMBTH:

  * Joshua M. Zutell, David C. Conner, and Philipp Schillinger, "Flexible Behavior Trees: In search of the mythical    HFSMBTH for Collaborative Autonomy in Robotics", arXiv 2022, https://doi.org/10.48550/arXiv.2203.05389.


#### BehaviorTree.cpp Hybrid Demo

  * `clear; ros2 run delib_ws_worlds run --ros-args -p problem_number:=2`
  * `clear; ros2 launch flexbe_onboard behavior_onboard.launch.py`
  * `clear; ros2 launch flexbe_webui flexbe_ocs.launch.py headless:=true`
  * `clear; ros2 run flexbe_webui webui_client`

Additionally we need to launch the custom `flexbe_btcpp_executor` node which executes the
BT.cpp v4-based behavior trees using an `ExecuteTree` action server interface.
  * `clear; ros2 launch pyrobosim_flexbe_btcpp pyrobosim_flexbe_btcpp.launch.xml`

This custom executor node loads the node definitions used by this workshop demonstration.

Via the FlexBE webui load the `BtCppHFSMBTH` behavior and execute.
    This uses a `RunBtCppTreeState` to send `ExecuteTree` action goals to the custom `flexbe_btcpp_executor` node.

We include a subset of the demonstration trees from this workshop in the `pyrobosim_flexbe_btcpp` package; these define unique behavior IDs; e.g.

```xml
  <BehaviorTree ID="Problem2Tree">
```
which is required if they are launched by the `ExecuteTree.action`.

Additional behavior tree packages may be listed in the `pyrobosim_flexbe_btcpp/config/pyrobosim_flexbe_btcpp.yaml` configuration file, but the required node definitions must be
registered with the `flexbe_btcpp_executor` code.

The FlexBE operator may preempt the `RunBtCppTreeState` which sends a message to cancel the `ExecuteTree` action to the BT action server, which halts the behavior tree.

You may optionally start `groot2` to visualize the behavior tree as it executes.
