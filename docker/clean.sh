# /bin/bash

set -e

docker run --user usr \
    --rm -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/.Xauthority:/home/usr/.Xauthority \
    --device /dev/dri -e DISPLAY=$DISPLAY -e XAUTHORITY=/home/usr/.Xauthority \
    -v $(dirname $(dirname $(realpath $0))):/home/usr/proj dbatk \
    sh -c "rm -rf /home/usr/proj/build/ /home/usr/proj/Thirdparty/g2o/build/ /home/usr/proj/Thirdparty/DBoW2/build/"