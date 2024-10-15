# ros\_bt\_py Setup for ROSCon 2024 Deliberation Technologies Workshop

---

## Installation and Setup

For the ROSCon '24 Deliberation Technologies workshop all required software is included in the Docker container.
To build locally at home see the [installation instructions](https://fzi-forschungszentrum-informatik.github.io/ros2_ros_bt_py/index.html).

## Getting Started

To get started with `ros_bt_py` as part of the workshop no additional packages would be required.
We provide generic nodes for interacting with ROS Topics, Services and Actions, which only require having the messages available in your environment when starting `ros_bt_py`.
Nevertheless we provided the `ros_bt_py_pyrobosim` package containing some additional nodes, providing convenience wrappers, making interacting with `pyrobosim` simpler.

First run one of the problem scenarios from the workshop:

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=1
```

To run `ros_bt_py` with these convenience nodes you need to run:

```bash
ros2 launch ros_bt_py_pyrobosim ros_bt_py_pyrobosim.launch.py
```

Alternatively, to launch the regular `ros_bt_py` without any additional nodes run:

```bash
ros2 launch ros_bt_py ros_bt_py.launch.py
```

After launching you can click one of the shown URLs to open our WebUI in a browser.
Commonly this is just `http://localhost:8085`.

## Usage

The WebUI has four distinct sections:

* On the left you have the list of available nodes.
  These can be moved into the main editing area at the center to add them to the tree.
  Some nodes (e.g. EnumFields) require you to click on the node in the list and add required details at the bottom of the editing window before they can be added to  the tree via the `AddToTree` button.
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
  * **Saving a Tree** In the top right corner there is the save button.
    It allows you to save the current tree in a yaml format to a specified list of storage folders on the system running the `ros_bt_py` ROS node.
    As this might be a remote system we only index folders specified in the launch file for saving an loading. (By default this is the `/home/$USERNAME/.ros` folder).
  * **Loading a Tree** Trees can be loaded from two different sources, installed ROS packages and the storage folders as specified in the saving trees section.
    To load one of the example solution trees you need to click the `Load` button, followed by the `Package` button.
    In the opened loading dialogue you need to enter the name of the ROS package you want to load a tree from.
    In this case `ros_bt_py_pyrobosim` and then select the trees folder.
    Afterwards you can load a tree by clicking on one of the yaml files and pressing the load button.
* The center bottom area is where attributes on a clicked node can be edited.
  ROS message types and builtin python types are automatically completed when a type field is edited.
>
> **BUG:** When updating node attributes and clicking apply, a popup will say that changes are discarded. This a bug and the changes will apply fully as intended.
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

### ROS Actions & Services

`ros_bt_py` has generic nodes for interacting with ROS services, topics and actions.
The `Service` and `Action` nodes allow to call an arbitrary ROS service/action.
Within both nodes options, the endpoint and type must be specified.
The `FieldsToMessage` and `MessageToFields` nodes can be used to construct and destruct the `Request/Goal` and `Result` messages from these nodes.
`Constant` nodes can be used to create primitive python datatype constants.
>
> **BUG:** It should also be possible to generate full ROS message types in constants, but unfortunately due to changes to ROS/Python type introspection
> this is not possible. ROS messages need to be constructed using the `FieldsToMessage` or the `MessageFromConstDict` node.
>

## Subtrees

The `subtree` node allows to load a saved tree yaml as a subtree.
Within the `subtree` node the location of the tree to load must be specified via a URI.
Similarly to loading a tree in the main editor there are two optioon where a subtree can be loaded from, ROS Packages or storage folder.

* **ROS Package** For a ROS package the URI needs to have the following format: `package://<ROS package name>/...` (where `...` is the filepath of the tree yaml in the packages share folder).
  E.g. `package://ros_bt_py_pyrobosim/trees/problem_1.yaml`
* **Storage Folder** For a storage folder the URI has the following format: `file://<path_to_storage_folder>/...`
  E.g. `file:///home/david/.ros/problem_1.yaml`

When using a subtree node, the `Publish Subtrees` option in the top left can be ticked to get visualizations of the subtrees.
After ticking the box, the subtree to view can be selected in the `Tree:` dropdown above the editing overview.
