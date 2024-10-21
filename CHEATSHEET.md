# Workshop Cheatsheet

## Repository Guide

### Clone this repo

```bash
git clone --recurse-submodules https://github.com/ros-wg-delib/roscon24-workshop.git
```

### Update to the latest version (including submodules)

```bash
git pull
git submodule update --init
```

---

## Docker Guide

### Pull the image

```bash
docker compose pull
```

### Build the image locally

```bash
docker compose build
```

### Start a new container

```bash
docker compose run base
```

### Attach a new shell to an existing container

```bash
docker compose exec base bash
```

### Shutting down cleanly

```bash
docker compose down --remove-orphans
```

NOTE: This is needed because every time you run `docker compose run base`, a new container is created and may become "orphaned".
You can alternatively use `docker compose up base` to avoid this, but it uses up one terminal.

---

## PyRoboSim Guide

### Start a world

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=1
```

Change the `problem_number` argument above to launch different worlds.

To edit existing worlds, refer to the `problems/delib_ws_worlds/worlds` folder in this repository.

### Query an individual robot's state

```bash
ros2 topic echo /robot/robot_state
```

### Query the complete world's state

```bash
ros2 service call /request_world_state pyrobosim_msgs/srv/RequestWorldState {}
```

### Run an action with a specific robot

Navigation example:

```bash
ros2 action send_goal /execute_action pyrobosim_msgs/action/ExecuteTaskAction "{action: {robot: 'robot', type: 'navigate', target_location: 'desk'}}"
```

Pick/place example:

```bash
ros2 action send_goal /execute_action pyrobosim_msgs/action/ExecuteTaskAction "{action: {robot: 'robot', type: 'pick', object: 'waste0'}}"

ros2 action send_goal /execute_action pyrobosim_msgs/action/ExecuteTaskAction "{action: {robot: 'robot', type: 'place', object: 'waste0'}}"
```

### PyRoboSim display is frozen

Sometimes, PyRoboSim's display may not update while a robot is actually moving.
We are actively investigating this issue, but to manually "unstick" the GUI you can toggle the "Show collision polygons" checkbox to force the state to refresh.

---
