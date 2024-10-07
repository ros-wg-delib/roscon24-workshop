# Hands-On with ROS 2 Deliberation Technologies

> [!WARNING]
> WORK IN PROGRESS!

This repository contains materials for the ROSCon 2024 workshop on ROS 2
Deliberation Technologies.

## Setup

First, clone this repository and its submodules.

```bash
git clone --recurse-submodules https://github.com/ros-wg-delib/roscon24-workshop.git
```

Start the Docker container.
Note that this will pull the Docker image and may take a few minutes.

```bash
docker compose run base
```

Once you're in the container, check that you can run a demo.

```bash
ros2 run delib_ws_p1 run
```

**NOTE:** If you want to cleanup the colcon build artifacts across container usage,
you can run this command (it will ask you for your sudo password):

```bash
./clean_build.sh
```

---

## Problem Descriptions

### Problem 1

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=1
```

__Goal__:
Snacks on the dining room table.

__Initial State__:
Snacks in the kitchen pantry.

__Available Actions__:

- Pick object
- Place object
- Move robot

### Problem 2

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=2
```

__Goal__:
Waste should be in the dumpster.
Dumpster should be closed.

__Initial State__:
Waste is on the office desk and in the office bin.
Hallways into the trash room are closed.

__Available Actions__:

- Pick object
- Place object
- Move robot
    (will fail if door is closed)
- Open door
- Close door

### Problem 3

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=3
```

__Goal__:
Bring bread and butter to the dining table.
Fridge and pantry should be closed at the end.

__Initial State__:
Bread is in the pantry, which is closed.
Butter is in the fridge, which is closed.

__Available Actions__:

- Pick object
- Place object
- Move robot
    (will fail if door is closed)
- Open door
- Close door
- Detect objects (optional)

Actions may fail with some probability.

### Problem 4

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=4
```

__Goal__:
Bring bread and butter to the dining table.
Fridge and pantry should be closed at the end.
Waste should be in the dumpster.
Dumpster should be closed.
Don't run out of battery!

__Initial State__:
Bread is in the pantry, which is closed.
Butter is in the fridge, which is closed.
Waste is on the office desk and in the office bin.
Hallways into the trash room are closed.

__Available Actions__:

- Pick object
- Place object
- Move robot
    (will fail if door is closed)
- Open door
- Close door
- Detect objects (optional)

Actions may fail with some probability.

Actions now use up battery, which can be fixed by navigating to the charger.
