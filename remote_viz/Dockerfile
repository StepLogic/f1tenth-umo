# ROS 2 Humble + RViz on this Ubuntu 24.04 PC (Humble has no 24.04 packages).
# Run with host networking so the container sees the wg0 VPN interface.
FROM osrf/ros:humble-desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp ros-humble-nav2-msgs ros-humble-nav2-rviz-plugins \
        ros-humble-ackermann-msgs \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///vpn/cyclonedds_pc.xml \
    ROS_DOMAIN_ID=42
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
