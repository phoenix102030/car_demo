#!/usr/bin/env bash

# Runs a docker container with the image created by build_demo.bash
# Requires:
#   docker
#   nvidia-container-toolkit
#   an X server
# Recommended:
#   A joystick mounted to /dev/input/js0 or /dev/input/js1

# Wait for Docker daemon to be available
until docker ps > /dev/null 2>&1
do
    echo "Waiting for Docker server"
    sleep 1
done

# Check if rocker is installed
if ! [ -x "$(command -v rocker)" ]; then
    echo "Rocker not found, installing from pip"
    mkdir -p /tmp/car_demo_rocker_venv
    python3 -m venv /tmp/car_demo_rocker_venv
    source /tmp/car_demo_rocker_venv/bin/activate
    pip install -U git+https://github.com/osrf/rocker.git
else
    # If rocker is installed, check if it needs to be updated
    rocker_version=$(rocker --version)
    echo "Found Rocker version: $rocker_version"
fi

# Activate the virtual environment if it exists
if [ -d "/tmp/car_demo_rocker_venv" ]; then
    source /tmp/car_demo_rocker_venv/bin/activate
fi

# Run the rocker command
rocker --nvidia --x11 --devices /dev/input/js0 /dev/input/js1 -- \
    osrf/car_demo:$(git rev-parse --abbrev-ref HEAD)
