# Dockerfile for ROSCon 2024 Deliberation Workshop

FROM ros:jazzy-ros-base AS roscon_delib_ws_2024
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install dependencies
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    apt-utils python3-pip python3-tk \
    libegl1 libgl1-mesa-dev libglu1-mesa-dev '^libxcb.*-dev' libx11-xcb-dev \
    libxi-dev libxkbcommon-dev libxkbcommon-x11-dev libxrender-dev

# Create a ROS 2 workspace
RUN mkdir -p /delib_ws/src/

# Install pyrobosim
# TODO: Should we clone it this way, or include it as a submodule in the repo?
WORKDIR /delib_ws/src
RUN git clone -b main https://github.com/sea-bass/pyrobosim.git
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN cd pyrobosim && \
    pip3 install -e ./pyrobosim && \
    pip3 install -r test/python_test_requirements.txt

# Include workshop code
COPY . /delib_ws/src/roscon24-workshop

# Build the ROS workspace
WORKDIR /delib_ws
RUN . /opt/ros/jazzy/setup.bash && \
    colcon build

# Remove MatPlotLib display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypoint
CMD /bin/bash
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc
