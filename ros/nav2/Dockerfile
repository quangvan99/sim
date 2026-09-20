ARG ROS_IMAGE=ros:jazzy-ros-base
ARG ROS_DISTRO=jazzy
FROM ${ROS_IMAGE}
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/amr-nav2
COPY ros/nav2_gateway /opt/amr-nav2/ros/nav2_gateway
COPY ros/nav2/config /opt/amr-nav2/config
COPY tools/generate_nav2_map.py /opt/amr-nav2/tools/generate_nav2_map.py
COPY src/sim/maps/amr_world.json /opt/amr-nav2/source/amr_world.json

RUN python3 /opt/amr-nav2/tools/generate_nav2_map.py \
      /opt/amr-nav2/source/amr_world.json \
      /opt/amr-nav2/maps/amr_world.pgm \
      /opt/amr-nav2/maps/amr_world.yaml \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --base-paths /opt/amr-nav2/ros

CMD ["bash", "-lc", "source /opt/ros/$ROS_DISTRO/setup.bash && source /opt/amr-nav2/install/setup.bash && ros2 launch nav2_gateway nav2_sim.launch.py"]
