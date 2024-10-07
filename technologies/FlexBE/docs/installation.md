# FlexBE States and Behaviors for ROSCon 2024 Deliberation Technologies Workshop

## Installation and Setup

This documents installation of the required FlexBE Behavior Engine and FlexBE Webui.
This is provided for the workshop in a Docker container, but are available open source

- `git clone -b ros2-devel https://github.com/FlexBE/flexbe_behavior_engine.git`
- `git clone -b main https://github.com/FlexBE/flexbe_webui.git`

This workshop is using the latest `4.0.0+` version of the FlexBE Behavior Engine and WebUI.
> Note: This version does NOT work with the older flexbe_app!

The WebUI requires the following Python libraries specified in the `requires.txt` file:

```text
PySide6>=6.7.1
fastAPI==0.89.1
pydantic>=1.10.13
setuptools
websockets>=10.3
```

On Ubuntu 24.04 (i.e., for ROS 2 Jazzy), the system installs for `sudo apt install python3-websockets python3-pydantic python3-fastapi`
are sufficient and can be installed via `rosdep` from the `package.xml` dependencies.

Unfortunately, `PySide6` is not in the current Ubuntu 24.04 binaries.
In 24.04 you are required to set up a virtual environment and cannot do a local install.

The following has been tested under 24.04.  First go to your `WORKSPACE_ROOT` (e.g. `ros2_ws`) and run the following commands.

```bash
virtualenv -p python3 --system-site-packages ./venv
source ./venv/bin/activate
touch ./venv/COLCON_IGNORE
cd src/
```

This creates a `venv` folder that we will `COLCON_IGNORE` during builds.

We add the following to our `.bashrc` to enable running the `webui_client` from any terminal:

```bash
echo "Add Python virtual environment to current PYTHONPATH"
VENV_PATH="$WORKSPACE_ROOT/venv/lib/python3.12/site-packages"
if [[ ":$PYTHONPATH:" != *":$VENV_PATH:"* ]]; then
    export PYTHONPATH="$PYTHONPATH:$VENV_PATH"
fi
```

The demonstrations make use of pyrobosim

- `git clone -b main https://github.com/sea-bass/pyrobosim.git`

 Follow the pyrobosim startup [directions](https://pyrobosim.readthedocs.io/en/latest/)

> NOTE: Thus far, it has been tested with the `pyrobosim_ros/examples/demo.py` with partial observability enabled:

```python
-- a/pyrobosim_ros/examples/demo.py
+++ b/pyrobosim_ros/examples/demo.py
@@ -98,6 +98,7 @@ def create_world():
         radius=0.1,
         path_executor=ConstantVelocityExecutor(),
         path_planner=path_planner,
+        partial_observability=True
     )

```

This repo provides the following packages:

- `pyrobosim_flexbe_states`
  - generic state implementations that interact with the ROS 2 action interfaces provided by pyrobosim.
- `pyrobosim_flexbe_behaviors`
  - finite state machines that realize specific behaviors.

Build all of these packages using standard ROS 2 `colcon build`
