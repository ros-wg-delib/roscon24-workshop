# BehaviorTree.CPP Nodes for ROSCon 2024 Deliberation Technologies Workshop

This package provides some ready to use Nodes (i.e. "actions") to be used with **pyrobosim**.

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
