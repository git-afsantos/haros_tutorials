# Use an official Python runtime as a parent image
FROM ros:noetic
LABEL maintainer="haros.framework@gmail.com"

# Set apt-get to run non-interactively (omit prompts)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# I think that ROS Desktop is not necessary. ROS Base should be enough.
# RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-desktop && apt upgrade -y

# Install external dependencies
RUN apt-get update && apt upgrade -y && apt-get install -y \
    cccc \
    clang-10 \
    cppcheck \
    git \
    libclang-10-dev \
    libmagic-dev \
    llvm-10-dev \
    netbase \
    python-is-python3 \
    python3-pip \
    wget

RUN pip3 install --upgrade pip
RUN pip3 install -Iv clang==10.0.1
RUN pip3 install -e git+https://github.com/timtadh/pyflwor.git#egg=pyflwor

SHELL ["/bin/bash", "-c"]

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/llvm-10/lib
ENV CMAKE_CXX_COMPILER=/usr/lib/llvm-10/bin/clang++

RUN mkdir -p /root/ws/src

WORKDIR /root/ws/src


ARG TUTORIAL_VER=latest

RUN git clone https://github.com/git-afsantos/haros_tutorials.git

WORKDIR /root/ws

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws/devel/setup.bash" >> /root/.bashrc && \
    echo "alias python=python3" >> /root/.bashrc


ARG HAROS_VER=latest

RUN pip3 install haros

RUN pip3 install \
    haros-plugin-pyflwor \
    haros-plugin-pbt-gen

RUN haros init

COPY configs.yaml /root/.haros

EXPOSE 8080

CMD ["/bin/bash"]

