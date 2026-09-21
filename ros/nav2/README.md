# Nav2 simulator stack

This directory contains the first Nav2 bringup for the AMR world. It is kept
separate from the Python 3.14 application because ROS 2 and Gazebo bridge
packages have their own ABI and distribution constraints.

Generate the map before launching:

```bash
python3 tools/generate_nav2_map.py \
  src/sim/maps/amr_world.json \
  ros/nav2/maps/amr_world.pgm \
  ros/nav2/maps/amr_world.yaml
```

The ROS container builds this directory as the `nav2_gateway` package under
`/opt/amr-nav2/ros/nav2` and launches `nav2_gateway/nav2_sim.launch.py`. The initial TF tree intentionally uses identity
`map -> odom`, Gazebo ground-truth odometry for `odom -> base_footprint`, and static sensor
frames. SLAM and EKF are a later phase.

The Gazebo Jetty / ROS 2 bridge combination is a compatibility gate. Run the
bridge spike before treating `docker-compose.nav2.yml` as production-ready.
