# Example build command:
# docker build --pull --no-cache -t gst_h264:foxy --build-arg=TARGET_ROS_DISTRO=foxy .

# Example run command:
# docker run -it gst_h264:foxy bash

ARG TARGET_ROS_DISTRO=foxy
ARG GST_H264_BRANCH=main
ARG H264_IMAGE_TRANSPORT_BRANCH=master

FROM osrf/ros:$TARGET_ROS_DISTRO-desktop

RUN apt-get update
RUN apt-get upgrade -y

# GStreamer
RUN apt-get install -y libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
 gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools \
 gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
 libgstreamer-plugins-base1.0-dev

WORKDIR /work/my_ws/src

ARG TARGET_ROS_DISTRO
ARG GST_H264_BRANCH
ARG H264_IMAGE_TRANSPORT_BRANCH

RUN git clone https://github.com/clydemcqueen/gst_h264.git -b $GST_H264_BRANCH
RUN git clone https://github.com/clydemcqueen/h264_image_transport.git -b $H264_IMAGE_TRANSPORT_BRANCH

WORKDIR /work/my_ws

RUN rosdep install -y --from-paths . --ignore-src

RUN /bin/bash -c "source /opt/ros/$TARGET_ROS_DISTRO/setup.bash && colcon build"

# source install/setup.bash
# ros2 launch gst_h264 example_launch.py
