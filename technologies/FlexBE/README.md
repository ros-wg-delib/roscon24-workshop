# FlexBE States and Behaviors for ROSCon 2024 Deliberation Technologies Workshop

> NOTE: This is a very preliminary version


# Installation and Setup

For the ROSCon '24 Deliberation Technologies workshop all required software is included in the Docker container.
To build locally at home see the [installation instructions](docs/installation.md).


## Usage

We provide a few behaviors, for now use the `delib_ws_p2_sm` behavior.
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

  > WARNING: An operator can preempt a state, so wait for the state to finish and request the outcome by highlighting the transition.

  > NOTE: Much more to follow.

## To dos

- [ ] Battery monitor state and concurrency example
- [ ] Define (sub-)behaviors for workshop tasks
- [ ] Write up more detail usage instructions


