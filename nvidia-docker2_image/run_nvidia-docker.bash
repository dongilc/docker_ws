#!/bin/sh
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

DOCKER_WS=~/docker_ws

docker run --name nvidia_ros1 --runtime=nvidia --privileged --gpus all -it \
           --volume=$XSOCK:$XSOCK:rw \
           --volume=$XAUTH:$XAUTH:rw \
           --volume=$DOCKER_WS:$DOCKER_WS \
           -w=$DOCKER_WS \
           --shm-size=4gb \
           --env="XAUTHORITY=${XAUTH}" \
           --env="DISPLAY=${DISPLAY}" \
           --env=TERM=xterm-256color \
           --env=QT_X11_NO_MITSHM=1 \
           --net=host \
           -e __NV_PRIME_RENDER_OFFLOAD=1 \
           -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
           nvidia_cuda_ros_image_v1 \
           bash
