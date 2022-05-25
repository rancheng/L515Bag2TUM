#!/bin/bash

# get parameter from system
user=`id -un`

# start sharing xhost
xhost +local:root

# run docker
docker run --rm \
  --ipc=host \
  --gpus all \
  --privileged \
  -p 4751:22 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:$docker/.Xauthority \
  -v $HOME/work:$HOME/work \
  -v /mnt/Data/Datasets:/mnt/Data/Datasets \
  -e http_proxy=http://127.0.0.1:2340 \
  -e https_proxy=http://127.0.0.1:2340 \
  -e XAUTHORITY=$home_folder/.Xauthority \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -it --name "ros-melodic" ${user}/ros-bionic-melodic
