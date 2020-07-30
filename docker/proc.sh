# /bin/bash

set -e

REPO=$(dirname $(dirname $(realpath $0)))
DATA=$(realpath $1)

docker run --user usr \
    --rm -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/.Xauthority:/home/usr/.Xauthority \
    --device /dev/dri -e DISPLAY=$DISPLAY -e XAUTHORITY=/home/usr/.Xauthority \
    -v $REPO:/home/usr/proj -v $DATA:/mnt/data orbslam2 sh -c \
    "~/proj/Examples/Monocular/mono_tum ~/proj/Vocabulary/ORBvoc.txt ~/proj/Examples/Monocular/TUM1.yaml /mnt/data/ && cp /tmp/observations.yaml /mnt/data"
