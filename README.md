# Hands-On with ROS 2 Deliberation Technologies

> [!WARNING]
> WORK IN PROGRESS!

This repository contains materials for the [ROSCon 2024](https://roscon.ros.org/2024/) workshop on ROS 2
Deliberation Technologies.

Deliberation in robotics refers to the collection of technologies necessary to create highly capable autonomous robots.
The key idea is the use of *models* of the robot and its environment to enable high-level decision-making for solving complex problems, as well as equipping robots with robust *skill representations* to successfully execute these tasks in the real world.

To learn more about deliberation, refer to [Ingrand and Ghallab (2017)](https://hal.science/hal-01137921).

![image](media/deliberation_ingrand_ghallab_2017.png)
*Overview diagram of robot deliberation, [Ingrand and Ghallab (2017)](https://hal.science/hal-01137921)*

---

## Technology Overview

In this hands-on workshop, you will get the opportunity to use a few ROS 2 enabled tools designed for robot deliberation.

This workshop uses [PyRoboSim](https://github.com/sea-bass/pyrobosim) as a simple 2D simulator with a ROS 2 interface to test our deliberation technologies.
PyRoboSim simulates mobile robots navigating across various locations, with the ability to detect and manipulate objects, and open and close locations.
Additionally, it can simulate failures and battery usage in the above actions.

Using this simulator, we can explore several ROS 2 deliberation technologies.
The [`technologies`](./technologies/README.md) subfolder contains more detailed information.

![image](media/pyrobosim_world.png)

---

## Setup

First, clone this repository and its submodules.

```bash
git clone --recurse-submodules https://github.com/ros-wg-delib/roscon24-workshop.git
```

Then, build the Docker image.

```bash
docker compose build
```

Start a container.

```bash
docker compose run base
```

Once you're in the container, check that you can run a demo.

```bash
ros2 run delib_ws_world run
```

---

## Developing in the Container

By default, the whole ROS 2 workspace should have been built on setup.
However, you will need to rebuild your code as you develop new code or make changes to existing code.

We have created handy aliases that effectively wrap around `colcon build --symlink-install`:

* `delib_build` - Builds the entire workspace
* `delib_build_packages <package1> <package2> ...` - Builds specific packages
* `delib_build_packages_up_to <package>` - Builds all dependencies up to a specific package.
* `delib_clean` - Cleans up the entire workspace.

When you are ready to shut down the container, enter the following command:

```bash
docker compose down --remove-orphans
```

**NOTE:** If you want to clean up any colcon build artifacts mounted to your host system (in the `.colcon` folder of this repo), you can run the following command.
It will ask you for your `sudo` password.

```bash
./clean_build.sh
```

---

## Problem Descriptions

During this workshop, you will work through increasingly difficult problems as you become familiar with deliberation tools.

### Problem 1

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=1
```

**Goal**:
Snacks on the dining room table.

**Initial State**:
Snacks in the kitchen pantry.

**Available Actions**:

* Pick object
* Place object
* Move robot

### Problem 2

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=2
```

**Goal**:
Waste should be in the dumpster.
Dumpster should be closed.

**Initial State**:
Waste is on the office desk and in the office bin.
Hallways into the trash room are closed.

**Available Actions**:

* Pick object
* Place object
* Move robot
    (will fail if door is closed)
* Open door
* Close door

### Problem 3

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=3
```

**Goal**:
Bring bread and butter to the dining table.
Fridge and pantry should be closed at the end.

**Initial State**:
Bread is in the pantry, which is closed.
Butter is in the fridge, which is closed.

**Available Actions**:

* Pick object
* Place object
* Move robot
    (will fail if door is closed)
* Open door
* Close door
* Detect objects (optional)

Actions may fail with some probability.

### Problem 4

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=4
```

**Goal**:
Bring bread and butter to the dining table.
Fridge and pantry should be closed at the end.
Waste should be in the dumpster.
Dumpster should be closed.
Don't run out of battery!

**Initial State**:
Bread is in the pantry, which is closed.
Butter is in the fridge, which is closed.
Waste is on the office desk and in the office bin.
Hallways into the trash room are closed.

**Available Actions**:

* Pick object
* Place object
* Move robot
    (will fail if door is closed)
* Open door
* Close door
* Detect objects (optional)

Actions may fail with some probability.

Actions now use up battery, which can be fixed by navigating to the charger.
