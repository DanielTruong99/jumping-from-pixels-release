# syntax=docker/dockerfile:experimental

FROM nvidia/cudagl:11.1-runtime-ubuntu18.04


ENV LC_ALL C.UTF-8
ENV LANG C.UTF-8
ARG DEBIAN_FRONTEND=noninteractive

# Replacing shell with bash for later docker build commands
RUN mv /bin/sh /bin/sh-old && \
  ln -s /bin/bash /bin/sh

# install basic system stuff
COPY ./install_scripts/install_basic.sh /tmp/install_basic.sh
RUN chmod +x /tmp/install_basic.sh
RUN /tmp/install_basic.sh

# install python3
#RUN apt-get update && apt-get install -y cmake python3.7 python3.7-dev 
#RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 10
#RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3 get-pip.py
#RUN apt-get install -y libpython3.7-dev libeigen3-dev python3-numpy

RUN apt-get update && apt-get install -y cmake python3.7 python3.7-dev 
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1 
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 2
RUN update-alternatives --config python3
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3 get-pip.py
RUN apt-get install -y libpython3-dev libeigen3-dev python3-numpy
RUN apt-get install -y python3-gi-cairo libcairo2-dev pkg-config python3-dev


# install ROS stuff
ENV ROS_DISTRO melodic

COPY ./install_scripts/install_ros.sh /tmp/install_ros.sh
RUN chmod +x /tmp/install_ros.sh
RUN /tmp/install_ros.sh

# bootstrap rosdep
RUN rosdep init \
  && rosdep update

# create catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN source /opt/ros/melodic/setup.bash
RUN mkdir -p $CATKIN_WS/src
WORKDIR ${CATKIN_WS}
RUN catkin init
RUN catkin config --extend /opt/ros/$ROS_DISTRO \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
WORKDIR $CATKIN_WS/src



RUN apt-get update && apt-get install -y git
RUN apt-get install -y software-properties-common
RUN apt-get install -y gcc-8 g++-8
ENV CXX=/usr/bin/g++-8
ENV CC=/usr/bin/gcc-8

# ==================================================================
# create working directories
# ------------------------------------------------------------------
ENV WORKSPACE=/workspace
ENV LOCAL_BUILD=/usr/local
RUN mkdir -p $WORKSPACE
RUN mkdir -p $LOCAL_BUILD

# copy local requirements file for pip install python deps
RUN pip3 install --upgrade pip
COPY ./requirements.txt /workspace
WORKDIR /workspace
RUN pip3 install -r requirements.txt



# ==================================================================
# tensorflow
# ------------------------------------------------------------------
#RUN pip3 install tensorflow-gpu==1.14 setuptools


# ==================================================================
# add ld path
# ------------------------------------------------------------------

RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$LOCAL_BUILD/lib


# ==================================================================
# Cheetah Sim Tools
# ------------------------------------------------------------------
RUN mkdir /tools
WORKDIR /tools


RUN apt-get update && apt-get -y upgrade && apt-get -y install build-essential libglib2.0-dev libfontconfig python3-netifaces mesa-common-dev freeglut3-dev gcc wget unzip vim tmux ffmpeg


# LCM
RUN wget https://github.com/lcm-proj/lcm/releases/download/v1.3.1/lcm-1.3.1.zip
RUN unzip lcm-1.3.1.zip
RUN cd lcm-1.3.1 && ./configure && make -j && make install && ldconfig
RUN cd lcm-1.3.1/lcm-python && python3 setup.py install

# Eigen
RUN git clone https://gitlab.com/cantonios/eigen.git && cd eigen && mkdir build && cd build && cmake .. && make -j8 && make install

# qpoases
RUN git clone --recursive https://github.com/stephane-caron/qpOASES.git && cd qpOASES && mkdir build && cd build && cmake .. && make -j && make install && cd .. && make && cd interfaces/python && python3 setup.py install --user

# easyrl
RUN git clone --recursive https://github.com/gmargo11/easyrl.git
RUN cd easyrl && git checkout notxn && python3 setup.py install --user


# ==================================================================
# Useful Libraries for Development
# ------------------------------------------------------------------
RUN apt update && apt install -y apt-transport-https ca-certificates curl software-properties-common
#RUN curl -fsSL https://download.sublimetext.com/sublimehq-pub.gpg | apt-key add - && add-apt-repository "deb https://download.sublimetext.com/ apt/stable/" && apt update && apt install sublime-text


WORKDIR /tools
# ==================================================================
# LCM add-ons
# ------------------------------------------------------------------

RUN apt-get install -y default-jre default-jdk
RUN git clone https://github.com/lcm-proj/lcm.git ~/tools/lcm && cd ~/tools/lcm && mkdir build && cd build && cmake -DLCM_ENABLE_JAVA=ON .. && make -j && make install


# ==================================================================
# Install Control Library
# ------------------------------------------------------------------


# ipopt
RUN apt-get install -y liblapack-dev
RUN git clone https://github.com/coin-or-tools/ThirdParty-HSL.git && cd ThirdParty-HSL && ./configure --without-hsl && make && make install
RUN git clone https://github.com/coin-or/Ipopt.git && cd Ipopt && ./configure --without-hsl && make -j && make install
RUN apt install -y coinor-libipopt-dev

WORKDIR /workspace

# Qt
ENV QT_VERSION_A=5.10
ENV QT_VERSION_B=5.10.0
ENV QT_VERSION_SCRIPT=594
RUN wget https://download.qt.io/new_archive/qt/$QT_VERSION_A/$QT_VERSION_B/qt-opensource-linux-x64-$QT_VERSION_B.run
RUN chmod +x qt-opensource-linux-x64-$QT_VERSION_B.run
COPY qt-install-noninteractive.qs /qt-noninteractive.qs 
RUN ./qt-opensource-linux-x64-$QT_VERSION_B.run --script /qt-noninteractive.qs --platform minimal --verbose

RUN git clone https://github.com/mit-biomimetics/Cheetah-Software.git
ADD ./Cheetah-Software-Diff /workspace/Cheetah-Software/ 
RUN cd Cheetah-Software && git submodule add ../../pybind/pybind11 third-party/pybind11 -b stable && git submodule update --init

WORKDIR /workspace/Cheetah-Software

# pybind11
RUN mkdir third-party/pybind11/build && cd third-party/pybind11/build && cmake -DBUILD_TESTING=OFF .. && make -j8 && make install

RUN cd scripts && ./make_types.sh
RUN export QT_VERSION_B=5.10.0 && export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/qt/$QT_VERSION_B/gcc_64/lib/cmake/ && mkdir mc-build && cd mc-build && cmake -DNO_SIM=ON -DMINI_CHEETAH_BUILD=ON -DMACHINE_LEARNING_BUILD=OFF -DIPOPT_ENABLE=ON -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3.7 .. && make -j8 && make install

RUN cd /tools/qpOASES/build && make -j && make install

RUN apt-get install -y python3-cairocffi python3-tk
#RUN sudo ln -s /usr/lib/python3/dist-packages/gi/_gi.cpython-{36m,37m}-x86_64-linux-gnu.so





# ==================================================================
# interface setup
# ------------------------------------------------------------------


#RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/workspace/Robot-Software/mc-build/lib/" >> /root/.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/" >> /root/.bashrc
RUN echo "export PYTHONPATH=$PYTHONPATH:$LOCAL_BUILD/lib" >> /root/.bashrc
WORKDIR /

# Exposing the ports
EXPOSE 11311

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
  ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


RUN mkdir /workspace/jumping-from-pixels
WORKDIR /workspace/jumping-from-pixels

# ==================================================================
# run ssh daemon with X11 forwarding
# ------------------------------------------------------------------
#RUN apt install -y openssh-server \
#    && mkdir /var/run/sshd \
#    && mkdir /root/.ssh \
#    && chmod 700 /root/.ssh \
#    && ssh-keygen -A \
#    && sed -i "s/^.*PasswordAuthentication.*$/PasswordAuthentication no/" /etc/ssh/sshd_config \
#    && sed -i "s/^.*X11Forwarding.*$/X11Forwarding yes/" /etc/ssh/sshd_config \
#    && sed -i "s/^.*X11UseLocalhost.*$/X11UseLocalhost no/" /etc/ssh/sshd_config \
#    && grep "^X11UseLocalhost" /etc/ssh/sshd_config || echo "X11UseLocalhost no" >> /etc/ssh/sshd_config


# setup entrypoint
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
