#!/bin/bash

dev_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
dev_dir="$( dirname "$dev_dir" )"

set -e
set -o pipefail

docker run "$@" -it --net=host -v  /tmp/.X11-unix:/tmp/.X11-unix \
 --gpus all \
 -e NVIDIA_DRIVER_CAPABILITIES=compute,video,utility,graphics \
 -e NVIDIA_VISIBLE_DEVICES=all \
 -e "TERM=xterm-256color" \
 -v $HOME/24-25WaterCode:/home/ubuntu/.24-25WaterCode.readonly \
 --ipc=host \
 -v /dev:/dev --privileged \
 --shm-size=8G \
 -e DISPLAY=$DISPLAY  --user ubuntu mate2025:latest /bin/bash
