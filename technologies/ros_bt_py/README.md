# ros\_bt\_py Setup for ROSCon 2024 Deliberation Technologies Workshop
---

## Installation and Setup

For the ROSCon '24 Deliberation Technologies workshop all required software is included in the Docker container.
To build locally at home see the [installation instructions](https://fzi-forschungszentrum-informatik.github.io/ros2_ros_bt_py/index.html).

## Getting Started

To get started with `ros_bt_py` as part of the workshop no additonal packages would be required.
We provide generic nodes for interacting with ROS Topics, Services and Actions, which only require having the messages available in your environment when starting `ros_bt_py`.
Nevertheless we provided the `ros_bt_py_pyrobosim` package containing some additonal nodes, providing convinience wrappers, making interacting with `pyrobosim` simpler.

First run one of the problem scenarios from the workshop:

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=1
```

To run `ros_bt_py` with these convinience nodes you need to run:
```bash
ros2 launch ros_bt_py_pyrobosim ros_bt_py_pyrobosim.launch.py 
```

Alternativly, to launch the regular `ros_bt_py` without any additional nodes run:
```bash
ros2 launch ros_bt_py ros_bt_py.launch.py
```

After launching you can click one of the shown URLs to open our WebUI in a browser.
Commonly this is just `http://localhost:8085`.

## Usage

The WebUI has four distinct sections:
* On the left you have the list of available nodes.
  These can be moved into the main editing area at the center to add them to the tree.
  Some nodes (e.g. EnumConstant) require you to click on the node in the list and add required details at the bottom of the editing window before they can be added to  the tree via the `AddToTree` button.
* In the center there is the main editing window.
  By moving nodes into this area their position within the tree can be adjusted.
  When connecting nodes from the data terminals at either the left (inputs) or right (outputs) side of a node you can created data connections between nodes.
  The color of the node will indicate its current state:
    * Dark Red - Shutdown
    * Blue - Idle
    * Yellow - Running
    * Red - Failure
    * Green - Succeed
* The top bar provides execution controls and reports the current status of the tree.
  Also the load / save controls for trees is located here.
  Trees can be loaded from ROS packages and from the `.ros` directory.
  Saving is unfortunately only possible in the `.ros` directory.
* The center bottom area is where attributes on a clicked node can be edited.
  ROS message types and builtin python types are automatically completed when a type field is edited.
> 
> BUG: When updating node attributs and clicking apply, a popup will say that changes are discarded. This a bug and the changes will apply fully as intended.
> This will be fixed with the next GUI update.
>

This explains the basics of the `ros_bt_py` UI. 
In the following some more details on specific features are given:

### Data Flow

`ros_bt_py` uses a typed datagraph to share data between nodes.
Each node can expose inputs and outputs which can be connected to other nodes.

All inputs are mandatory, meaning that if an input is not connected or at runtime no value is present, a runtime error will be raised.
Outputs are populated on the tick the node succeeds.

Data connections are strongly typed for ROS messages and primitive python types.
While container types (e.g arrays, lists, tuple) are checked, their content types are not.
Connecting an `list[int]` output to an `list[string]` input will result in a runtime error. 

### ROS Actions & Servuces

`ros_bt_py` has generic nodes for interacting with ROS services, topics and actions.
The `Serivce` and `Action` nodes allow to call an arbitrary ROS service/action.
Within both nodes options, the endpoint and type must be specified.
The `FieldsToMessage` and `MessageToFields` nodes can be used to construct and destruct the `Request/Goal` and `Result` messages from these nodes.
`Constant` nodes can be used to create primitve python datatype constants.
>
> BUG: It should also be possible to generate full ROS message types in constants, but unfortunately due to changes to ROS/Python type introspection
> this is not possible. ROS messages need to be constructed using the `FieldsToMessage` node.
> 

## ToDo's

- [ ] Provide example solutiont trees for all problems.
- [ ] Add convinience nodes for pyrobosim interaction.

