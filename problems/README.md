# Problems

This folder contains all the initial files for the workshop.

This includes the world models in [PyRoboSim](https://github.com/sea-bass/pyrobosim), and some simple Python abstractions for commanding simulated robots and retrieving their state.

During this workshop, you will work through increasingly difficult problems as you become familiar with deliberation tools.

---

## Problem 1

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=1
```

**Goal**:
Snacks on the dining room table.

**Initial State**:
Snacks in the kitchen pantry.

**Actions to Use**:

* Pick object
* Place object
* Move robot

---

## Problem 2

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=2
```

**Goal**:
Waste should be in the dumpster.
Dumpster should be closed.

**Initial State**:
Waste is on the office desk and in the office bin.
Hallways into the trash room are closed.

**Actions to Use**:

* Pick object
* Place object
* Move robot
    (will fail if door is closed)
* Open door
* Close door

---

## Problem 3

```bash
ros2 run delib_ws_worlds run --ros-args -p problem_number:=3
```

**Goal**:
Bring bread and butter to the dining table.
Fridge and pantry should be closed at the end.

**Initial State**:
Bread is in the pantry, which is closed.
Butter is in the fridge, which is closed.

**Actions to Use**:

* Pick object
* Place object
* Move robot
    (will fail if door is closed)
* Open door
* Close door
* Detect objects (optional)

Actions may fail with some probability.

---

## Problem 4

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

**Actions to Use**:

* Pick object
* Place object
* Move robot
    (will fail if door is closed)
* Open door
* Close door
* Detect objects (optional)

Actions may fail with some probability.

Actions now use up battery, which can be fixed by navigating to the charger.
