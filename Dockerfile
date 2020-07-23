FROM ubuntu:20.04

RUN apt update && apt upgrade -y && \
    DEBIAN_FRONTEND="noninteractive" apt -y install tzdata keyboard-configuration && \
    apt install -y git cmake libopencv-dev libgl1-mesa-dev libglew-dev sudo clang vim libeigen3-dev

RUN cd /opt && git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && \
    cmake --build . -j$(nproc)

RUN cd /opt && git clone https://github.com/t-lou/ORB_SLAM2.git && cd ORB_SLAM2 && \
    git checkout ubuntu20 && chmod +x build.sh && ./build.sh

RUN useradd -m usr && echo "usr:usr" | chpasswd && usermod -aG sudo usr
