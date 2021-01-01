# laundryman
SSY235 project


 1.Clone this repo to the same catkin workspace with tiago_public_ws

 2. Run:  roslaunch laundryman laundryman.launch  public_sim:=true robot:=steel

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
