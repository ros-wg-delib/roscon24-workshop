# Technologies

## Prerequisites

To work with these tools, you need to have gone through the setup steps in the [root README](../README.md).
Also, we advise you to go through the [problems](../problems/README.md) first, as they will serve as examples on which to apply these technologies.

## Introduction

This folder contains all the technology-specific implementations of ROS 2 Deliberation technologies.
Here you will find the tools and libraries that you can learn about in this repository.

For a more complete collection of tools and resources for deliberation in ROS 2, please refer to the [Awesome Robotic Deliberation](https://github.com/ros-wg-delib/awesome-ros-deliberation) repository.

## Skill Abstractions

While is it possible to directly program our robots to perform complex, long-horizon tasks with plain code, it can be useful to think at higher levels of abstraction.
There are several formalisms that can help you assemble robust, reactive behaviors for your robots.
Arguably, the two most popular ones are **finite-state machines (FSMs)** and **behavior trees (BTs)**.

Importantly, all these tools (and many others in the ROS ecosystem) provide a programmatic interface for you to write basic skills in code that leverages ROS, and a graphical interface to assemble the skills together into full behaviors and monitor their execution.

With this repository, you will be able to use the following technologies:

- [BehaviorTree.CPP](./BehaviorTree.CPP/README.md)
  - A C++ based library for behavior trees.
  - [github repo](https://github.com/BehaviorTree/BehaviorTree.CPP)
  - [documentation](https://www.behaviortree.dev/)
  - [**To start using it to solve the problems, refer to the BehaviorTree.CPP subfolder**](./BehaviorTree.CPP/README.md).

- [ros_bt_py](./ros_bt_py/README.md)
  - A Python based library for behavior trees.
  - [github repo](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py)
  - [**To start using it to solve the problems, refer to the ros_bt_py subfolder**](./ros_bt_py/README.md).

- [FlexBE](./FlexBE/README.md)
  - A Python based library for hierarchical finite-state machines.
  - [github org](https://github.com/FlexBE)
  - [**To start using it to solve the problems, refer to the the FlexBE subfolder**](./FlexBE/README.md).

## Task Planning

While you can get far by programming your robots manually using finite-state machines and behavior trees, you can also automate the sequencing of these skills to perform complex tasks.

If you have a *world model* and a set of *skills* that the robot is allowed to take, there are various algorithms to automatically search for plans that can achieve arbitrary goals possible within that world model.
This is known as **task planning**, and you can learn more in [this blog post](https://roboticseabass.com/2022/07/19/task-planning-in-robotics/).

In this repository, you will explore task planning in the ROS 2 ecosystem using [SkiROS2](https://github.com/RobotLabLTH/skiros2).

[Start here to solve the example problems using SkiROS2](./SkiROS2/skiros2_pyrobosim_lib/README.md)

## Formal Verification and Model Checking

In addition to making it easier to program our robots, another benefit of using abstractions such as finite-state machines and behavior trees is the ability to analyze their robustness via *formal verification*.

One such project that aims to solve this problem is the [CONVINCE toolbox](https://convince-project.github.io/overview/).
In this workshop, you will explore [AS2FM](https://github.com/convince-project/AS2FM), which is part of the CONVINCE toolbox, to perform *statistical model checking* on behavior trees.

[Start here to learn about statistical model checking with AS2FM](./convince/README.md)
