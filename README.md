# Hands-On with ROS 2 Deliberation Technologies

> [!WARNING]  
> WORK IN PROGRESS!

This repository contains materials for the ROSCon 2024 workshop on ROS 2
Deliberation Technologies.

## Setup

First, clone this repository.

```bash
git clone -b https://github.com/ros-wg-delib/roscon24-workshop.git
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
ros2 launch pyrobosim_ros demo.launch.py
```

