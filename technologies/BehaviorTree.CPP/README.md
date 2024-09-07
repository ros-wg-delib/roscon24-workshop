# BehaviorTree.CPP Nodes for ROSCon 2024 Deliberation Technologies Workshop

This package provides some ready to use Nodes (i.e. "actions") to be used with **pyrobosim**.

## How to run

A BTCPP executor is provided to run Behavior Trees stored as XML files

For instance, consider this sample XML in [trees/navigate_demo.xml](pyrobosim_btcpp/trees/navigate_demo.xml)

```xml
<root BTCPP_format="4">

  <BehaviorTree ID="MainTree">
    <Sequence>
      <Navigate target="office"/>
      <Navigate target="dining"/>
      <Navigate target="kitchen"/>
    </Sequence>
  </BehaviorTree>

</root>
```

```
ros2 run pyrobosim_btcpp btcpp_executor --tree=trees/navigation_demo.xml
```
