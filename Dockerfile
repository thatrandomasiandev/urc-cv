FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-robot-localization \
    ros-humble-twist-mux \
    ros-humble-rmw-cyclonedds-cpp \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir ultralytics

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ros2_ws
COPY ros2_ws/ /ros2_ws/

RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
    --packages-select aruco_detector urc_autonomy visual_servo \
    search_behavior urc_localization"

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
