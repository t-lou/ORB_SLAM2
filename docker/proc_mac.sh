# /bin/bash

set -e

echo "start proc_mac.sh"
REPO=$(dirname $(dirname $(realpath $0)))
echo $REPO
DATA=$(realpath $1)

docker run --user usr \
    --rm -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/.Xauthority:/home/usr/.Xauthority \
    -e DISPLAY=$IP:0 -v $REPO:/home/usr/proj -v $DATA:/mnt/data orb2 bash -c \
    "~/proj/Examples/Monocular/mono_tum ~/proj/Vocabulary/ORBvoc.txt ~/proj/Examples/Monocular/TUM1.yaml /mnt/data/ && eval {mkdir,cd}\ /mnt/data/$(date +%Y-%m-%d-%H:%M:%S)\; && cp /tmp/*.yaml ."
