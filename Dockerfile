ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO

RUN echo "Building docker image with ROS_DISTRO=$ROS_DISTRO"

RUN apt-get update -y --fix-missing
RUN apt-get install -y ssh \
                       vim mc \
                       iputils-ping net-tools iproute2 curl \
                       pip

# aiorc neeed pip update or fails on cffi version inconsistency
RUN pip install --upgrade pip

RUN apt-get install -y libjsoncpp-dev

RUN pip install setuptools

# socket.io cpp
# WORKDIR /root
# RUN git clone --recurse-submodules https://github.com/socketio/socket.io-client-cpp.git
# WORKDIR /root/socket.io-client-cpp
# RUN cmake ./
# RUN make install

RUN apt install -y libwebsocketpp-dev
RUN apt install -y libyaml-cpp-dev
RUN apt install -y libtinyxml2-dev
RUN apt install -y gpiod libgpiod-dev
RUN apt install -y uuid-dev
RUN apt install -y libcurl4-openssl-dev

# wget https://sourceforge.net/projects/asio/files/asio/1.30.2%20%28Stable%29/asio-1.30.2.tar.bz2/download
# RUN git clone https://github.com/chriskohlhoff/asio.git

WORKDIR /root
RUN git clone https://github.com/chriskohlhoff/asio.git
# RUN wget https://phoenixnap.dl.sourceforge.net/project/asio/asio/1.30.2%20%28Stable%29/asio-1.30.2.tar.gz
# RUN tar -xvzf asio-1.30.2.tar.gz
# WORKDIR /root/asio-1.30.2
# RUN ./configure
# RUN make
# RUN make install

WORKDIR /root
RUN git clone --single-branch --branch support-new-asio https://github.com/toonetown/websocketpp.git

# RUN apt install -y libasio-dev
RUN apt install -y rapidjson-dev

WORKDIR /root
RUN git clone https://github.com/paullouisageneau/libdatachannel.git
WORKDIR /root/libdatachannel
RUN git submodule update --init --recursive --depth 1
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release
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
test -f "/ros2_ws/install/setup.bash" && source "/ros2_ws/install/setup.bash" \n \
\n \
exec "$@"' > /ros_entrypoint.sh

RUN chmod a+x /ros_entrypoint.sh

# source underlay on every login
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /root/.bashrc
RUN echo 'test -f "/ros2_ws/install/setup.bash" && source "/ros2_ws/install/setup.bash"' >> /root/.bashrc

WORKDIR $ROS_WS

# allow to read git repo sha/tag without warnings
RUN git config --system --add safe.directory '*'

# clone and install phntm interfaces
RUN git clone https://github.com/PhantomCybernetics/phntm_interfaces.git /ros2_ws/src/phntm_interfaces
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src/phntm_interfaces --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --packages-select phntm_interfaces

# install phntm bridge and agent
COPY ./ $ROS_WS/src/phntm_bridge
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    rosdep install -i --from-path src/phntm_bridge --rosdistro $ROS_DISTRO -y
    # && \
    #colcon build --symlink-install --packages-select phntm_bridge

# pimp up prompt with hostame and color
RUN echo "PS1='\${debian_chroot:+(\$debian_chroot)}\\[\\033[01;35m\\]\\u@\\h\\[\\033[00m\\] \\[\\033[01;34m\\]\\w\\[\\033[00m\\] '"  >> /root/.bashrc

WORKDIR $ROS_WS

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "bash" ]
