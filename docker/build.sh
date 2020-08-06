# /bin/bash

set -e

REPO=$(dirname $(dirname $(realpath $0)))

docker run --user usr \
    --rm -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/.Xauthority:/home/usr/.Xauthority \
    --device /dev/dri -e DISPLAY=$DISPLAY -e XAUTHORITY=/home/usr/.Xauthority \
    -v $REPO:/home/usr/proj dbatk /home/usr/proj/build.sh