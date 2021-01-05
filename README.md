# laundryman
SSY235 project


 1.Clone this repo to the same catkin workspace with tiago_public_ws


## run simulation with tables and cubes
roslaunch laundryman laundryman.launch

## run navigation
1. cp -r path/to/map $HOME/.pal
2. roslaunch laundryman navigation.launch

## run motion control
rosrun laundryman motion_control_node

## control the robot
rostopic pub /move_to_bucket std_msgs/UInt32 "data: 0" --once

change the 0 to 0, 1, 2, 3 to select tables

## Change camera resolution:
simulation_ws/src/tiago_robot/tiago_description/urdf/sensors/openni.gazebo.xacro
