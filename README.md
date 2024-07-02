# Hands-On with ROS 2 Deliberation Technologies

WORK IN PROGRESS!

This repository contains materials for the ROSCon 2024 workshop on ROS 2 Deliberation Technologies.

---

## Setup

First, clone this repository.

```
git clone -b https://github.com/ros-wg-delib/roscon24-workshop.git
```

Then, build the Docker image.

```
docker compose build
```

Start a container.

```
docker compose run base
```

Once you're in the container, check that you can run a demo.

```
ros2 launch pyrobosim_ros demo.launch.py
```

---
