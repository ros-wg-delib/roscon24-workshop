# Dockerfile for ROSCon 2024 Deliberation Workshop

FROM ros:jazzy-ros-base AS roscon_delib_ws_2024
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install dependencies
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
    apt-utils python3-pip python3-tk python3-wrapt python3-inflection \
    libegl1 libgl1-mesa-dev libglu1-mesa-dev '^libxcb.*-dev' libx11-xcb-dev \
    libxi-dev libxkbcommon-dev libxkbcommon-x11-dev libxrender-dev

# Create a ROS 2 workspace.
RUN mkdir -p /delib_ws/src/

# Install external dependencies.
WORKDIR /delib_ws
COPY dependencies src/dependencies
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src -y --ignore-src
# Aggregated Python dependencies
COPY python-requirements.txt /delib_ws
RUN pip3 install --break-system-packages -r python-requirements.txt
# SkiROS2 dependencies
RUN src/dependencies/SkiROS2/skiros2/skiros2/scripts/install_fd_task_planner.sh

# Remove MatPlotLib display warnings.
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypoint.
CMD /bin/bash
COPY .docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc
