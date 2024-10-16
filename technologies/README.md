# Technologies

This folder contains all the technology-specific implementations of ROS 2 Deliberation technologies.

For a more complete collection of tools, refer to the [Awesome Robotic Deliberation](https://github.com/ros-wg-delib/awesome-ros-deliberation) repository.

---

## Skill Abstractions

While is it possible to directly program our robots to perform complex, long-horizon tasks with plain code, it can be useful to think at higher levels of abstraction.
There are several formalisms that can help you assemble robust, reactive behaviors for your robots.
Arguably, the two most popular ones are **finite-state machines (FSMs)** and **behavior trees (BTs)**.

In this workshop, you will be using the following software tools:

* [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) - A C++ based library for behavior trees.
* [ros_bt_py](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py) - A Python based library for behavior trees.
* [FlexBE](https://github.com/FlexBE) - A Python based library for hierarchical finite-state machines.

Importantly, all these tools (and many others in the ROS ecosystem) provide a programmatic interface for you to write basic skills in code that leverages ROS, and a graphical interface to assemble the skills together into full behaviors and monitor their execution.

---

## Task Planning

While you can get far by programming your robots manually using finite-state machines and behavior trees, you can also automate the sequencing of these skills to perform complex tasks.

If you have a *world model* and a set of *skills* that the robot is allowed to take, there are various algorithms to automatically search for plans that can achieve arbitrary goals possible within that world model.
This is known as **task planning**, and you can learn more in [this blog post](https://roboticseabass.com/2022/07/19/task-planning-in-robotics/).

In this workshop, you will explore task planning in the ROS 2 ecosystem using [SkiROS2](https://github.com/RobotLabLTH/skiros2).

[Start here to solve the workshop problems with SkiROS2 and to additionally explore task planning.](https://github.com/matthias-mayr/skiros2_pyrobosim_lib).

---

## Formal Verification and Model Checking

In addition to making it easier to program our robots, another benefit of using abstractions such as finite-state machines and behavior trees is the ability to analyze their robustiness via *formal verification*.

One such project that aims to solve this problem is the [CONVINCE toolchain](https://convince-project.github.io/overview/).
In this workshop, you will explore [AS2FM](https://github.com/convince-project/AS2FM), which is part of the CONVINCE toolchain, to perform *statistical model checking* on behavior trees.
