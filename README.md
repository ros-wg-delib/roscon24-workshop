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
ros2 run delib_ws_p1 run
```

**NOTE:** If you want to cleanup the colcon build artifacts across container usage,
you can run this command:

```bash
sudo rm -rf .colcon/build .colcon/install .colcon/log
```

## Problem Descriptions

### Problem 1

[package delib_ws_p1](delib_ws_p1)

__Goal__:
Banana on table 2.

__Initial State__:
Banana on table 1.

__Available Actions__:

- Pick object
- Place object
- Move robot

__Available Conditions__:

- Robot location
    table 1, table 2
- Object location
    table 1, table 2
- Object in hand
    true, false

### Problem 2

[package delib_ws_p2](delib_ws_p2)

__Goal__:
Banana on table 2.

__Initial State__:
Banana on table 1.

__Available Actions__:

- Pick object
- Place object
- Move robot
    (will fail if door is closed)
- Open door
- Close door

__Available Conditions__:

- Robot location
    table 1, table 2
- Object location
    table 1, table 2
- Object in hand
    true, false
- Door state
    open, closed

### Problem 3

[package delib_ws_p3](delib_ws_p3)

__Goal__:
Banana on table 2.

__Initial State__:
Banana on table 1.

__Available Actions__:

- Pick object
    (may fail with probability 0.5)
- Place object
- Move robot
    Locations: table 1, table 2, door
    (will fail if door is closed)
    (can also fail while moving with probability 0.1)
- Open door
    (may fail with probability 0.5)
- Close door

__Available Conditions__:

- Robot location
    table 1, table 2
- Object location
    table 1, table 2
- Object in hand
    true, false
- Door state
    open, closed

### Problem 4

[package delib_ws_p4](delib_ws_p4)

__Goal__:
Banana on table 2.
Never run out of battery.

__Initial State__:
Banana on table 1.

__Available Actions__:

- Pick object
    (may fail with probability 0.5)
- Place object
- Move robot
    Locations: table 1, table 2, door, charging station
    (will fail if door is closed)
    (can also fail while moving with probability 0.1)
- Open door
    (may fail with probability 0.5)
    (returns reason for failure, slipped, locked)
        - locked -> find different way
        - slipped handle -> try again
- Close door
- Charge battery
    (can only be done at charging station)

__Available Conditions__:

- Robot location
    table 1, table 2
- Object location
    table 1, table 2
- Object in hand
    true, false
- Door state
    open, closed
- Battery level
    (low, normal, full)
