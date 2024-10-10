# FlexBE States and Behaviors for ROSCon 2024 Deliberation Technologies Workshop

## Installation and Setup

For the ROSCon '24 Deliberation Technologies workshop all required software is included in the Docker container.
To build locally at home see the [installation instructions](docs/installation.md).

## Usage

We provide a few behaviors as described below.

This quick start uses the `delib_ws_p2_sm` behavior.
This includes a statemachine with on nested sub-statemachine to allow selection of a door to open, and a
nested behavior called `DetectSelect` that plans path to location, retrieves a user selected object, and places at target location.

To run start the following in separate terminals

* `pyrobosim` with whatever world you choose
  * `clear; ros2 run delib_ws_worlds run --ros-args -p problem_number:=2`

* `clear; ros2 launch flexbe_onboard behavior_onboard.launch.py`
  * This is what would be run "onboard" the robot

* `clear; ros2 launch flexbe_webui flexbe_ocs.launch.py headless:=true`
  * This launches the remote "Operator Control Station" (OCS) minus the UI

* `clear; ros2 run flexbe_webui webui_client`
  * This launches UI
  * Note: It is best to give the OCS software several seconds to start and load behaviors before launching the UI
    Wait on the `Begin behavior mirror processing ...` message in terminal

> Note: On some systems, the docker and GPU interactions interfere with UI rendering.
> `export QT_QUICK_BACKEND=software` in terminal or use the aliased helper function
> `qt_soft_render` before launching the `webui_client`

* `clear; ros2 run flexbe_input input_action_server`
  * This launches an action server that will pop up a simple UI that will allow the operator input data on request
  * In this demonstration, it is used to select from among the detected objects
  * This must be started to avoid a `'flexbe/behavior_input' is not available` warning in the demonstration behavior.

  From the FlexBE UI "Behavior Dashboard", select "Load Behavior" from the main toolbar, and then select `delib_ws_p2_sm` from the list of available behaviors.

  Click on the "StateMachine Editor" tab to see the state machine.
  For now, this demonstration behavior is configured to require the operator to confirm most transitions unless you place in "Full" autonomy on the "Runtime Control".

  Click on the "Runtime Control" tab.
  You may specify the parameters for the initial "Move to Location" where you will look for objects, and the "Place Location".
  The default values show are specified as "Behavior Parameters" on the dashboard.

  Select "Start Behavior" and follow along as the UI "mirrors" the onboard behavior.
  For `p2`, you should first invoke the `open` outcome to activate the "OpenDoorSM" sub-state machine
  to open the required doors and dumpster (you may invoke this repeatedly), then
  activate the `go` outcome to activate the "DetectSelect" sub-behavior to go to the target
  location and detect available objects.

  > Note: In `Low` autonomy, you will need to click on the outcome label when requested (that is, when the state has finished) to confirm the transition when prompted at the UI.
  >
  > WARNING: An operator can preempt a state, so wait for the state to finish and request the outcome by highlighting the transition.

  Alternatively you can try :
  * `clear; ros2 run delib_ws_worlds run --ros-args -p problem_number:=4`

  And load `PatrolCharge` to move about all rooms and recharge the battery when level drops below 30% charge.
  This uses a `ConcurrencyContainer` and a `PriorityContainer` to preempt the `Patrol` behavior and recharge the battery.

There are a number of demonstration behaviors created to demonstrate various state implementations

### Provided State Implementations

The `flexbe_states` package in the `dependencies/FlexBE/flexbe_behavior_engine` submodule provides a number of generic states.

Specifically we demonstrate:

* `LogState` - print message to terminal (both console and UI)
* `LogKeyState` - print message containing user data value at specified key
* `OperatorDecisionState` - allow operator to select among specified outcomes; in full autonomy the "suggested" outcome is selected.
* `SelectionState` - uses `input_action_server` to allow user to select among given data
* `InputState` - uses `input_action_server` to allow user to input simple data such as string, number, or list of numbers

The `pyrobosim_flexbe_states` package under `technologies/FlexBE` includes:

* `check_door_state.py` - Uses pyrobosim `RequestWorldState` service to check door status using non-blocking call
* `detect_local_objects_state.py` - Invokes a pyrobosim `ExecuteTaskAction` to `detect` objects, then uses `RequestWorldState`
* `detect_objects_state.py` - Invokes the pyrobosim `DetectObjects` action to detect objects, then adds to user data
* `door_action_state.py` - Invoke `ExecuteTaskAction` to `open` or `close` an openable (including doors, dumpster, pantry, fridge, ...)
* `follow_path_state.py` - Follow current path passed as user data into state
* `monitor_battery_state.py` - Monitor battery state and return outcome if either low or high level detected (blocks, use in concurrent state)
* `navigate_action_state.py` - Navigation action that combines planning and following
* `next_room_state.py` - Does simple planning to determine adjacent room based on connected rooms
* `pick_action_state.py` - Does pick action at current location
* `place_action_state.py` - Does place action at current location
* `plan_path_state.py` - Request plan from current location to target location
* `run_btcpp_tree_state.py` - Run a behavior tree using custom BehaviorTree.CPP executor

The above states have extra logging information that is shown in the onboard behavior terminal.  Each state transition `on_start`, `on_enter`, `on_exit`, `on_pause`, `on_resume`, and `on_stop` are logged.  This is not recommended in regular states, but is done here for educational purposes.

### Basic Demonstration Behaviors

In order of increasing complexity

Use `clear; ros2 run delib_ws_worlds run --ros-args -p problem_number:=1`

* `test navigate` - Test navigation state
* `Test Plan Path` - Simple plan then follow behavior with userdata
* `test pick place` - Navigation with pick and place using behavior parameters
* `DetectSelect` - Move, detect objects, select using `input_action_server`, move and place
* `Go Beh` - Go to target location

Use `clear; ros2 run delib_ws_worlds run --ros-args -p problem_number:=1`

* `delib_ws_p2` - use behavior parameters plan to door, open door, then use `DetectSelect` embedded behavior
* `delib_ws_p2_sm` - use hierarchical state machine to select door, open, then use `DetectSelect` embedded behavior
* `Through Door` - Travel through doorway opening if necessary
* `Traverse` - Go to specified target location, opening intermediate doors if necessary using `Go Beh` and `Through Door`
  * This incorporates a simple planning `NextRoomState` to determine the next adjacent room based on world structure

* `Patrol` - traverses each room in particular order using `Traverse` behavior
  * You may want to use low autonomy and confirm reasonable paths and guide to open doors initially before switching to full autonomy

 Use `clear; ros2 run delib_ws_worlds run --ros-args -p problem_number:=4`

* `PatrolCharge` - This uses a `Concurrency` container that includes the `Patrol` behavior and a state machine that monitors battery level.
  * On low battery level (set to 30%), a `PriorityContainer` invokes the `Traverse` behavior to go to charging dock.
  * This pauses the `Patrol` behavior until finished charging.  On resuming, any active planning or follow states will return `failed` and restart the `Patrol` behavior.

You can build on these behavior demonstrations to solve the workshop tasks.

See ["Mythical HFSMBT Hybrid"](docs/hfsmbth.md) for an example demonstration of a hybrid HFSM/BT.

## FlexBE Publications

Please use the following publications for reference when using FlexBE and the FlexBE WebUI:

* Philipp Schillinger, Stefan Kohlbrecher, and Oskar von Stryk, ["Human-Robot Collaborative High-Level Control with Application to Rescue Robotics"](http://dx.doi.org/10.1109/ICRA.2016.7487442), IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

* Joshua Zutell, David C. Conner, and Philipp Schillinger, ["ROS 2-Based Flexible Behavior Engine for Flexible Navigation"](http://dx.doi.org/10.1109/SoutheastCon48659.2022.9764047), IEEE SouthEastCon, April 2022.

* Samuel Raymond, Grace Walters, Joshua Luzier, and David C. Conner, "Design and Development of the FlexBE WebUI with Introductory Tutorials", J. Comput. Sci. Coll vol.40, no.3, CCSC Eastern, to appear October 2024.
