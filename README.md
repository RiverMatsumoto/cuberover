# cuberover

The workspace for cuberover.

Hardware used:
- Roboclaw 2x7A - [Documentation](https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf)
- 32RPM Brushed Motor - [Documentation](https://www.servocity.com/32-rpm-hd-premium-planetary-gear-motor-w-encoder/)
- BNO055 IMU - [Code repository used](https://github.com/williamg42/BNO055-Linux-Library) - [Manual](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
- ZED Mini Stereocamera [Documentation](https://github.com/stereolabs/zed-ros-wrapper)
- Vicon Tracking Cameras

## Software Dependencies

ZED SDK
ROS

## Building/compiling the workspace

```
git clone --recursive git@github.com:RiverMatsumoto/cuberover.git
cd ./cuberover
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source ./devel/setup.bash
```

## Launching specific components of cuberover

##### Motor/Wheel Controls and IMU

WIP

##### ZED Camera

`roslaunch zed_wrapper zedm.launch`

##### Vicon Tracking Cameras (using vicon_bridge)

WIP
