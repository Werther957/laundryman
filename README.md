# laundryman SSY235 project
## Installation guide
Clone this repo to a catkin workspace

### Change camera resolution:
In simulation_ws/src/tiago_robot/tiago_description/urdf/sensors/openni.gazebo.xacro

Change the camera resolution to 1920x1080 to get a stable aruco detection and pose estimation.

### Install SLAM map
cp -r path/to/laundryman/map $HOME/.pal

### Install libtorch
curl https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.7.1%2Bcpu.zip

unzip it in the laundryman root dir

## Run simulation with tables, cubes and the robot model
roslaunch laundryman laundryman.launch

## Run navigation + Rviz + aruco detection
roslaunch laundryman navigation.launch

## Run the CNN node
rosrun laundryman vision_node

## Run motion control
rosrun laundryman motion_control_node

### control the robot manually
rostopic pub /move_to_bucket std_msgs/UInt32 "data: 0" --once

change the 0 to 0, 1, 2, 3 to select tables

## Run the laundry sorting process
rosrun laundryman main_node