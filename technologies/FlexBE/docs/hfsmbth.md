## The Mythical HFSMBTH

Inspired by the 2019 Game Developers Conference [talk](https://www.youtube.com/watch?v=Qq_xX1JCreI&t=1159s) by Bobby Anguelov, we present the "Mythical HFSMBT Hybrid".

Please use the following publication for reference when using the HFSMBTH:

  * Joshua M. Zutell, David C. Conner, and Philipp Schillinger, "Flexible Behavior Trees: In search of the mythical    HFSMBTH for Collaborative Autonomy in Robotics", arXiv 2022, https://doi.org/10.48550/arXiv.2203.05389.


#### BehaviorTree.cpp Hybrid Demo

  * `clear; ros2 run delib_ws_worlds run --ros-args -p problem_number:=2`
  * `clear; ros2 launch flexbe_onboard behavior_onboard.launch.py`
  * `clear; ros2 launch flexbe_webui flexbe_ocs.launch.py headless:=true`
  * `clear; ros2 run flexbe_webui webui_client`
  * `clear; ros2 launch pyrobosim_flexbe_btcpp pyrobosim_flexbe_btcpp.launch.xml`

    And load `BtCppHFSMBTH` behavior.
    This uses a `RunBtCppTreeState` and a custom `flexbe_btcpp_executor` node.
