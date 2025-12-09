ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO

RUN echo "Building docker image with ROS_DISTRO=$ROS_DISTRO"

RUN apt-get update -y --fix-missing
RUN apt-get install -y pip wget

# dev conveniences
RUN apt-get install -y vim mc \
                       iputils-ping net-tools iproute2 \
                       curl \
                       gdb gdbserver \
                       clangd

RUN apt-get install -y libjsoncpp-dev

RUN apt-get install -y python3-setuptools

RUN apt install -y libwebsocketpp-dev
RUN apt install -y libyaml-cpp-dev
RUN apt install -y libtinyxml2-dev
RUN apt install -y gpiod libgpiod-dev
RUN apt install -y uuid-dev
RUN apt install -y libcurl4-openssl-dev
RUN apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp

WORKDIR /root
RUN git clone https://github.com/chriskohlhoff/asio.git
WORKDIR /root/asio
RUN git checkout tags/asio-1-34-2

WORKDIR /root
RUN git clone --single-branch --branch support-new-asio https://github.com/toonetown/websocketpp.git

RUN apt install -y rapidjson-dev

# libdatachannel deps
RUN apt install -y meson libglib2.0-dev libssl-dev libsrtp2-dev libjansson-dev libgstreamer1.0-dev
WORKDIR /root
RUN wget https://libnice.freedesktop.org/releases/libnice-0.1.22.tar.gz
RUN tar -xvzf libnice-0.1.22.tar.gz
RUN rm libnice-0.1.22.tar.gz
WORKDIR /root/libnice-0.1.22/
RUN meson build
RUN ninja -C build
RUN ninja -C build install

# RUN apt install -y libmbedtls-dev libmbedtls14
# RUN apt install -y python3-jinja python3-jsonschema
# WORKDIR /root
# RUN git clone https://github.com/Mbed-TLS/mbedtls.git
# WORKDIR /root/mbedtls
# RUN git checkout v3.6.3.1
# RUN git submodule update --init --recursive --depth 1
# RUN python3 -m pip install --user -r scripts/basic.requirements.txt
# RUN sed -i 's|^//\(#define MBEDTLS_SSL_DTLS_SRTP\)|\1|' include/mbedtls/mbedtls_config.h
# # RUN cmake tf-psa-crypto/
# RUN cmake -DCMAKE_BUILD_TYPE=Release -DUSE_STATIC_MBEDTLS_LIBRARY=Off -DUSE_SHARED_MBEDTLS_LIBRARY=On -DENABLE_TESTING=Off -DCMAKE_INSTALL_PREFIX=/usr .
# RUN make && make install

RUN apt install -y libpcap-dev

# using w libnice bcs libjuice fails on asymetric responses
WORKDIR /root
RUN git clone https://github.com/PhantomCybernetics/libdatachannel.git
WORKDIR /root/libdatachannel
RUN git submodule update --init --recursive --depth 1
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release -DUSE_NICE=1 -DUSE_MBEDTLS=0 -DTEST_APPS=0 -DNO_TESTS=1 -DNO_EXAMPLES=1
WORKDIR /root/libdatachannel/build
RUN make -j2
RUN make install

# init workspace
ENV ROS_WS=/ros2_ws
RUN mkdir -p $ROS_WS/src

# generate entrypoint script
RUN echo '#!/bin/bash \n \
set -e \n \
\n \
# setup ros environment \n \
source "/opt/ros/'$ROS_DISTRO'/setup.bash" \n \
export PYTHON_VERSION_VENV=$(python3 -c '"'"'import sys; print(".".join(map(str, sys.version_info[:2])))'"'"') \n \
export PATH="/root/ros2_py_venv/bin:$PATH" \n \
export PYTHONPATH="/root/ros2_py_venv/lib/python${PYTHON_VERSION_VENV}/site-packages:${PYTHONPATH:-}" \n \
test -f "/ros2_ws/install/setup.bash" && source "/ros2_ws/install/setup.bash" \n \
\n \
exec "$@"' > /ros_entrypoint.sh

RUN chmod a+x /ros_entrypoint.sh

# source underlay on every login
RUN echo '\n# PhntmBridge {' >> /root/.bashrc
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /root/.bashrc
RUN echo 'test -f "/ros2_ws/install/setup.bash" && source "/ros2_ws/install/setup.bash"' >> /root/.bashrc

WORKDIR $ROS_WS

# allow to read git repo sha/tag without warnings
RUN git config --system --add safe.directory '*'

# clone and install Phntm Interfaces
RUN git clone https://github.com/PhantomCybernetics/phntm_interfaces.git /ros2_ws/src/phntm_interfaces
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src/phntm_interfaces --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --packages-select phntm_interfaces

# install Agent deps
RUN apt-get install -y pkg-config
RUN apt-get install -y python3-termcolor
RUN apt-get install -y python3-pyee
# Docker ctrl
RUN apt-get install -y python3-docker
RUN apt-get install -y iw wireless-tools libiw-dev
RUN apt-get install -y wpasupplicant

# create a python venv, install pip-only deps
RUN apt-get install -y python3-venv
RUN mkdir -p /root/ros2_py_venv
RUN python3 -m venv /root/ros2_py_venv

# activate python venv on ~/.bashrc source
# this must be both here (for the dev mode launch) and in the entrypoint (compose command launch)
RUN echo 'export PYTHON_VERSION_VENV=$(python3 -c '"'"'import sys; print(".".join(map(str, sys.version_info[:2])))'"'"')' >> /root/.bashrc
RUN echo 'export PATH="/root/ros2_py_venv/bin:$PATH"' >> /root/.bashrc
RUN echo 'export PYTHONPATH="/root/ros2_py_venv/lib/python${PYTHON_VERSION_VENV}/site-packages:${PYTHONPATH:-}"' >> /root/.bashrc

# Agent python deps and ROS python libs (used when building packages)
RUN . /root/ros2_py_venv/bin/activate && \
    pip install iwlib && \
    pip install empy catkin_pkg numpy lark && \
    deactivate

# video enc
RUN apt-get install -y libopencv-dev
RUN apt-get install -y libavdevice-dev

# clone and install Phntm Agent
RUN git clone https://github.com/PhantomCybernetics/phntm_agent.git /ros2_ws/src/phntm_agent
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src/phntm_agent --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --packages-select phntm_agent

# install Phntm Bridge Client
COPY ./ $ROS_WS/src/phntm_bridge
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    rosdep install -i --from-path src/phntm_bridge --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --packages-select phntm_bridge

# pimp up prompt with hostame and color
RUN echo "PS1='\${debian_chroot:+(\$debian_chroot)}\\[\\033[01;35m\\]\\u@\\h\\[\\033[00m\\] \\[\\033[01;34m\\]\\w\\[\\033[00m\\] '"  >> /root/.bashrc
RUN echo '# } PhntmBridge' >> /root/.bashrc

WORKDIR $ROS_WS

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "bash" ]
