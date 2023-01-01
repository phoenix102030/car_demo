#!/bin/bash
USER_ID="$(id -u)"
XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
VOLUMES="--volume=$XSOCK:$XSOCK:rw
         --volume=$XAUTH:$XAUTH:rw"


docker run \
    -it --rm \
    $VOLUMES \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --privileged \
    --net=host \
    --name="car_demo" \
    osrf/car_demo:$(git rev-parse --abbrev-ref HEAD)
