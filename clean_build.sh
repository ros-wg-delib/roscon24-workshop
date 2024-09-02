#!/bin/sh
# Helper script to clean up the build artifacts generated inside the Docker container.
# It would be nice to not use root user inside the container, but this is not yet in place.

sudo rm -rf .colcon/build .colcon/install .colcon/log
