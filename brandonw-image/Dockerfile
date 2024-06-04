ARG ROS_DISTRO=noetic
FROM osrf/ros:${ROS_DISTRO}-desktop-full

ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

ARG GITHUB_TOKEN
ARG GITHUB_ID_EMAIL
ARG GITHUB_ID_NAME

ENV DISPLAY=novnc:0.0

RUN apt-get update && apt-get install -y git wget python3-pip vim ros-${ROS_DISTRO}-rosserial-arduino ros-${ROS_DISTRO}-rosserial ros-${ROS_DISTRO}-ros-ign ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-velocity-controllers ros-${ROS_DISTRO}-robot-state-controller libeigen3-dev

RUN git config --global user.email ${GITHUB_ID_EMAIL} && \
    git config --global user.name ${GITHUB_ID_NAME}

RUN git clone https://${GITHUB_TOKEN}:@github.com/Prababameister/brandonw-src.git /catkin_ws
WORKDIR /catkin_ws

RUN pip3 install setuptools==58.2.0 keyboard

CMD ["bash"]
