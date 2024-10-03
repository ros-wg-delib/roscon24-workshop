# BehaviorTree.CPP Nodes for ROSCon 2024 Deliberation Technologies Workshop

This package provides some ready to use Nodes (i.e. "actions") to be used with **pyrobosim**.


We suggest creating your XML trees in the folder `technologies/BehaviorTree.CPP/pyrobosim_btcpp/trees`

## How to compile

```
colcon build --symlink-install --packages-select pyrobosim_btcpp
```

## How to run

A BTCPP executor is provided to run Behavior Trees stored as XML files

For instance, consider this sample XML in [trees/navigation_demo.xml](pyrobosim_btcpp/trees/navigation_demo.xml)

```xml
<root BTCPP_format="4">

  <BehaviorTree ID="MainTree">
    <Sequence>
      <Navigate name="ToOffice" target="office"/>
      <Navigate name="ToFridge" target="fridge"/>
      <Navigate name="ToTable" target="table"/>
    </Sequence>
  </BehaviorTree>

</root>
```

NOTE: remember that the `name` attribute in the XML is optional and used for debugging only.

You can run the BeahviorTree with the command:

```
ros2 run pyrobosim_btcpp btcpp_executor --ros-args -p tree:=trees/navigation_demo.xml
```

The argument `tree` above is the path to the XML file. The path can be either:

 - absolute,
 - relative to the folder where the command is executed
 - relative to the package folder, i.e. `technologies/BehaviorTree.CPP/pyrobosim_btcpp`.

## Implemented Action Nodes

| Action Name  | Description                                 | Input Port                                                                     |
|--------------|---------------------------------------------|--------------------------------------------------------------------------------|
| Close        | Close a door or a container                 | - location: name of the location                                               |
| DetectObject | Return SUCCESS if the object is detected    | - object: name of the object                                                   |
| Navigate     | Move to a specific location                 | - target: name of the location                                                 |
| Open         | Open a door or a container                  | - object: name of the object                                                   |
| PickObject   | Pick an object (you must be in front of it) | - object: name of the object <br>- location: (optional) where the object is        |
| PlaceObject  | Place an object (you must hold it)          | - object: name of the object <br>- location: (optional) where the object should go |
