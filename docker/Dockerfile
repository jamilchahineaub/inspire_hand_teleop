FROM ros:humble

ARG TARGETARCH

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV PIP_ROOT_USER_ACTION=ignore

RUN apt-get update && apt-get upgrade -y

# ROS 2 image pipeline + CycloneDDS (matches g1_upperbody_ros2 DDS stack)
RUN apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rqt-image-view \
    ros-humble-rmw-cyclonedds-cpp \
    python3-pip \
    sudo \
    tmux

# MediaPipe 0.10.x ships with opencv-python; cv_bridge uses the same OpenCV via system libs.
# numpy pinned to match g1_upperbody_ros2 environment.
RUN pip3 install mediapipe==0.10.14 numpy==1.26.4 pyserial

RUN mkdir -p /workspace/ros2_ws/src

WORKDIR /workspace/ros2_ws

SHELL ["/bin/bash", "-c"]

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

CMD ["/entrypoint.sh"]
