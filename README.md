# Navigation_xrc

Gazebo simulation and autonomous navigation for a small differential-drive rover
(ROS 2 Jazzy, Gazebo Harmonic).

## What is in here

| File | Role |
|---|---|
| `worlds/my_world.sdf` | Test maze with the rover, a 360° LiDAR and a camera |
| `worlds/target_world.sdf` | Same maze with an orange target panel and an IMU on the chassis |
| `autonomous_nav.py` | First navigation node: QR-code target, stop in front of obstacles |
| `autonomous_nav_hsv.py` | Navigation node: HSV target detection, state machine, LiDAR gap finding |
| `covariance_relay.py` | Adds covariances to the simulator's odometry and IMU so the EKF accepts them |
| `config/ekf.yaml` | `robot_localization` EKF: wheel odometry + IMU |
| `camera_web.py` | Web control panel (camera, LiDAR view, teleoperation) |

## How the navigation works

```
SEARCH -> ALIGN -> APPROACH -> AVOID -> ARRIVED
```

- **SEARCH**: turn on the spot until the camera sees the panel (HSV threshold).
- **ALIGN**: centre the panel in the image.
- **APPROACH**: drive towards it. The panel's width in pixels gives its distance,
  so its position is remembered even when a wall hides it.
- **AVOID**: if the way is blocked, take the free LiDAR direction closest to the target,
  and resume once the line to the target is clear.
- **ARRIVED**: stop 0.75 m from the panel.

The pose comes from `/odometry/filtered`, the output of the EKF.

## Run it

```bash
gz sim -s -r worlds/target_world.sdf

ros2 run ros_gz_bridge parameter_bridge \
  '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU' \
  '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'

python3 covariance_relay.py
ros2 run robot_localization ekf_node --ros-args --params-file config/ekf.yaml
python3 autonomous_nav_hsv.py
```

In RViz, set the fixed frame to `my_robot/odom` and add `/scan`, `/camera/annotated`
and `/nav/markers`.

## Status

`autonomous_nav_hsv.py`, `covariance_relay.py` and `config/ekf.yaml` are a re-implementation
of the rover's navigation for the simulation. Tested on one run in `target_world.sdf`:
the rover reaches the panel from its start position in about 50 s of simulated time.
