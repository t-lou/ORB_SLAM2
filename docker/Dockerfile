FROM ubuntu:20.04

RUN apt-get update && apt-get upgrade -y && \
    DEBIAN_FRONTEND="noninteractive" apt-get -y install tzdata keyboard-configuration && \
    apt-get install -y git cmake libopencv-dev libgl1-mesa-dev libglew-dev sudo clang vim libeigen3-dev \
        libyaml-cpp-dev libgoogle-glog-dev libgtest-dev libsuitesparse-dev libzip-dev libboost-all-dev

RUN cd /opt && git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j && make install && cd /opt && rm -rf Pangolin

RUN cd /opt && git clone https://github.com/t-lou/ORB_SLAM2.git && cd ORB_SLAM2 && \
    chmod +x build.sh && ./build.sh

RUN cd /opt && git clone https://github.com/t-lou/LDSO.git && cd LDSO && \
    chmod +x ./make_project.sh && ./make_project.sh

RUN useradd -m usr && echo "usr:usr" | chpasswd && usermod -aG sudo usr && \
    runuser -l usr -c "echo \"if [[ -d '/home/usr/proj' ]] ; then cd ~/proj ; else cd ~; fi\" >> /home/usr/.bashrc" && \
    runuser -l usr -c "echo \"export CC=/usr/bin/clang\" >> /home/usr/.bashrc" && \
    runuser -l usr -c "echo \"export CXX=/usr/bin/clang++\" >> /home/usr/.bashrc"

RUN echo "#!/bin/bash\n/opt/LDSO/bin/run_dso_tum_mono preset=0 files=/mnt/data/images.zip vignette=/mnt/data/vignette.png calib=/mnt/data/camera.txt gamma=/mnt/data/pcalib.txt" >> /opt/LD\SO/bin/proc.sh && chmod +x /opt/LDSO/bin/proc.sh