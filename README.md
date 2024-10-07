# Hands-On with ROS 2 Deliberation Technologies

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

* The [`problems`](./problems/README.md) folder contains information about the robotics problems you will solve in this workshop.
* The [`technologies`](./technologies/README.md) folder contains more detailed information on the deliberation software tools you will use to solve these problems.

![image](media/pyrobosim_world.png)

---

## Setup

> [!NOTE]
> Please make sure you have completed the setup steps before arriving at the workshop.

### System Requirements

We highly recommend a host PC running Ubuntu 22.04 or 24.04.

If you are using macOS or Windows, you should install virtualization software such as [VirtualBox](https://www.virtualbox.org/) or [VMWare](https://www.vmware.com/products/desktop-hypervisor/workstation-and-fusion).
Then, set up an Ubuntu 22.04 or 24.04 virtual machine.

You also need to have Docker and Docker Compose installed on your system.
If you do not have these tools set up:

* Install Docker Engine using [these instructions](https://docs.docker.com/engine/install/ubuntu/).
  * **IMPORTANT:** Make sure you also go through the [Linux post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/).
* Install Docker Compose using [these instructions](https://docs.docker.com/compose/install/).

Once you are able to run `docker` and `docker compose` commands without `sudo`, you can move on to the installation steps below.

```bash
docker run hello-world
docker compose --help
```

### Installation

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

Once you're in the container, check that you can launch a simulated world and the PyRoboSim UI appears on your screen.

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
