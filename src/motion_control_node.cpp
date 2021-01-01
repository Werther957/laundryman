#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "moveit_msgs/Grasp.h"
#include "moveit_msgs/PickupAction.h"
#include "moveit_msgs/PlanningScene.h"
#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>

struct BucketInfo {
  unsigned int bucket_id;
  geometry_msgs::Pose pose;
};

class MoveControlller {
private:
  std::vector<BucketInfo> bucket_map;
  ros::NodeHandle nh;
  ros::Publisher base_goal_pub;
  ros::Subscriber base_goal_sub;

  ros::Publisher torso_controller_pub;
  ros::Publisher head_controller_pub;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
  // actionlib::SimpleActionClient<moveit_msgs::PickupAction> pickup_ac;

  // moveit_commander::PlanningSceneInterface scene;
  moveit::planning_interface::MoveGroupInterface group_arm_torso;

public:
  MoveControlller()
      : ac("/move_base", true), // pickup_ac("/pickup", true),
        group_arm_torso("arm_torso") {}

  void init() {
    torso_controller_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/torso_controller/command", 1);
    head_controller_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/head_controller/command", 1);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); // will wait for infinite time
    ROS_INFO("Server found.");

    /*
    ROS_INFO("Waiting for pickup action server to start.");
    pickup_ac.waitForServer(); // will wait for infinite time
    ROS_INFO("Server found.");
    */

    base_goal_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    base_goal_sub = nh.subscribe("move_to_bucket", 10,
                                 &MoveControlller::moveToBucket, this);

    bucket_map.resize(4);

    bucket_map[0].bucket_id = 0;
    bucket_map[0].pose.position.x = 0.0;
    bucket_map[0].pose.position.y = 2.0;
    bucket_map[0].pose.position.z = 0.0;
    bucket_map[0].pose.orientation.x = 0.0;
    bucket_map[0].pose.orientation.y = 0.0;
    bucket_map[0].pose.orientation.z = 0.0;
    bucket_map[0].pose.orientation.w = 1.0;

    bucket_map[1].bucket_id = 1;
    bucket_map[1].pose.position.x = 0.0;
    bucket_map[1].pose.position.y = 0.0;
    bucket_map[1].pose.position.z = 0.0;
    bucket_map[1].pose.orientation.x = 0.0;
    bucket_map[1].pose.orientation.y = 0.0;
    bucket_map[1].pose.orientation.z = 0.0;
    bucket_map[1].pose.orientation.w = 1.0;

    bucket_map[2].bucket_id = 2;
    bucket_map[2].pose.position.x = 0.0;
    bucket_map[2].pose.position.y = -2.0;
    bucket_map[2].pose.position.z = 0.0;
    bucket_map[2].pose.orientation.x = 0.0;
    bucket_map[2].pose.orientation.y = 0.0;
    bucket_map[2].pose.orientation.z = 0.0;
    bucket_map[2].pose.orientation.w = 1.0;

    bucket_map[3].bucket_id = 3;
    bucket_map[3].pose.position.x = -1.1;
    bucket_map[3].pose.position.y = 0.0;
    bucket_map[3].pose.position.z = 0.0;
    bucket_map[3].pose.orientation.x = 0.0;
    bucket_map[3].pose.orientation.y = 0.0;
    bucket_map[3].pose.orientation.z = 1.0;
    bucket_map[3].pose.orientation.w = 0.0;
  }
  void run() {
    init();
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
  }
  void moveToBucket(const std_msgs::UInt32 msg) {

    headUp();
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose = bucket_map[msg.data].pose;
    target_pose.header.stamp = ros::Time::now();
    target_pose.header.frame_id = "map";

    /*
    base_goal_pub.publish(target_pose);
    */
    ROS_INFO("sending goal");
    // send a goal to the action
    move_base_msgs::MoveBaseGoal goal;
    // goal.header.stamp = ros::Time::now();
    // goal.header.frame_id = "map";
    // goal.goal_id.stamp = goal.header.stamp;
    // goal.goal_id.id = std::to_string(msg.data);
    goal.target_pose = target_pose;
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(60.0));
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("goal reached");
      // look down
      torseUp();
      headDown();

      geometry_msgs::PoseStamped object_pose;
      object_pose.header.frame_id = "base_footprint";
      object_pose.pose.position.x = 0.53;
      object_pose.pose.position.y = -0.25;
      object_pose.pose.position.z = 0.9;
      object_pose.pose.orientation.x = 0.4996018;
      object_pose.pose.orientation.y = 0.4999998;
      object_pose.pose.orientation.z = 0.4999998;
      object_pose.pose.orientation.w = 0.5003982;
      moveArmToCube(object_pose);

    } else {
      ROS_INFO("goal reaching failed");
    }
  }

  void headDown() {
    ROS_INFO("Moving head down");
    trajectory_msgs::JointTrajectory jt;
    jt.joint_names = {"head_1_joint", "head_2_joint"};
    jt.points.resize(1);
    jt.points[0].positions = {0.0, -0.75};
    jt.points[0].time_from_start = ros::Duration(2.0);
    head_controller_pub.publish(jt);
    ROS_INFO("Done.");
  }

  void headUp() {
    ROS_INFO("Moving head up");
    trajectory_msgs::JointTrajectory jt;
    jt.joint_names = {"head_1_joint", "head_2_joint"};
    jt.points.resize(1);
    jt.points[0].positions = {0.0, 0.0};
    jt.points[0].time_from_start = ros::Duration(2.0);
    head_controller_pub.publish(jt);
    ROS_INFO("Done.");
  }

  void torseUp() {
    ROS_INFO("Moving torso up");
    trajectory_msgs::JointTrajectory jt;
    jt.joint_names = {"torso_lift_joint"};
    jt.points.resize(1);
    jt.points[0].positions = {0.34};
    jt.points[0].time_from_start = ros::Duration(2.5);
    torso_controller_pub.publish(jt);
    ROS_INFO("Done.");
  }

  void moveArmToCube(geometry_msgs::PoseStamped object_pose) {

    // choose your preferred planner
    ROS_INFO("Moving arm");
    group_arm_torso.setPlannerId("SBLkConfigDefault");
    group_arm_torso.setPoseReferenceFrame("base_footprint");
    group_arm_torso.setPoseTarget(object_pose);

    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(1.0);

    ROS_INFO("getting plan");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // set maximum time to find a plan
    group_arm_torso.setPlanningTime(5.0);
    bool success = bool(group_arm_torso.plan(my_plan));
    ROS_INFO("waiting plan");

    if (!success)
      ROS_INFO("No plan found");

    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

    // Execute the plan
    ros::Time start = ros::Time::now();

    moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
    if (!bool(e))
      ROS_INFO("Error executing plan");

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  }

  /*
  moveit_msgs::PickupGoal createPickupGoal(geometry_msgs::PoseStamped ps,
                                           moveit_msgs::Grasp g) {
    // Create a PickupGoal with the provided data
    moveit_msgs::PickupGoal pug;
    pug.target_name = "part";
    pug.group_name = "arm_torso";
    pug.possible_grasps.push_back(g);
    pug.allowed_planning_time = 35.0;
    pug.planning_options.planning_scene_diff.is_diff = true;
    pug.planning_options.planning_scene_diff.robot_state.is_diff = true;
    pug.planning_options.plan_only = false;
    pug.planning_options.replan = true;
    pug.planning_options.replan_attempts = 1;
    pug.attached_object_touch_links = {"gripper_left_finger_link",
                                       "gripper_right_finger_link",
                                       "gripper_link"};

    return pug;
  }
  */
  /*
  void
  createPlaceGoal(std::vector<moveit_msgs::PlaceLocation> place_locations) {
    // Create PlaceGoal with the provided data
    moveit_msgs::PlaceGoal placeg;
    placeg.group_name = "arm_torso";
    placeg.attached_object_name = "part";
    placeg.place_locations = place_locations;
    placeg.allowed_planning_time = 15.0;
    placeg.planning_options.planning_scene_diff.is_diff = true;
    placeg.planning_options.planning_scene_diff.robot_state.is_diff = true;
    placeg.planning_options.plan_only = false;
    placeg.planning_options.replan = true;
    placeg.planning_options.replan_attempts = 1;
    placeg.allowed_touch_objects = {"gripper_left_finger_link",
                                    "gripper_right_finger_link",
                                    "gripper_link"};

    return placeg;
  }*/

  /*
  std::vector<moveit_msgs::PlaceLocation>
  create_placings_from_object_pose(geometry_msgs::PoseStamped pose_stamped) {
    std::vector<moveit_msgs::PlaceLocation> place_locs(1);
    trajectory_msgs::JointTrajectory pre_grasp_posture;
    pre_grasp_posture.header.frame_id = "base_footprint";
    pre_grasp_posture.joint_names = {"gripper_left_finger_joint",
                                     "gripper_right_finger_joint"};
    pre_grasp_posture.points.resize(1);
    pre_grasp_posture.points[0].positions = {"0.038 0.038"};
    pre_grasp_posture.points[0].time_from_start =
        ros::Duration(self._time_pre_grasp_posture);

    moveit_msgs::PlaceLocation pl;
    pl.place_pose = pose_stamped;
    pl.post_place_posture = pre_grasp_posture;
    geometry_msgs::Vector3 pre_direction_vector;
    pre_direction_vector.x = 1.0;
    pre_direction_vector.y = 0.0;
    pre_direction_vector.z = 0.0;
    geometry_msgs::Vector3 post_direction_vector;
    post_direction_vector.x = -1.0;
    post_direction_vector.y = 0.0;
    post_direction_vector.z = 0.0;
    pl.pre_place_approach = createGripperTranslation(pre_direction_vector);
    pl.post_place_retreat = createGripperTranslation(post_direction_vector);

    return place_locs
  }
  */

  /*
  moveit_msgs::Grasp create_grasp(geometry_msgs::PoseStamped object_pose) {
    moveit_msgs::Grasp g;
    g.id = "grasp_id";

    trajectory_msgs::JointTrajectory pre_grasp_posture;
    pre_grasp_posture.header.frame_id = "arm_tool_link";
    pre_grasp_posture.joint_names = {"gripper_left_finger_joint",
                                     "gripper_right_finger_joint"};
    pre_grasp_posture.points.resize(1);
    pre_grasp_posture.points[0].positions = {0.038, 0.038};
    pre_grasp_posture.points[0].time_from_start = ros::Duration(2.0);

    trajectory_msgs::JointTrajectory grasp_posture = pre_grasp_posture;
    grasp_posture.points.resize(2);
    grasp_posture.points[0].time_from_start = ros::Duration(3.0);

    grasp_posture.points[1].positions = {0.038, 0.038};
    grasp_posture.points[1].time_from_start = ros::Duration(6.0);

    g.pre_grasp_posture = pre_grasp_posture;
    g.grasp_posture = grasp_posture;

    object_pose.pose.position.y += 0.2;
    g.grasp_pose = object_pose;
    g.grasp_quality = 0.1;

    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.direction.vector.y = 0.0;
    g.pre_grasp_approach.direction.vector.z = 0.0;
    g.pre_grasp_approach.direction.header.frame_id = "arm_tool_link";
    g.pre_grasp_approach.desired_distance = 0.20;
    g.pre_grasp_approach.min_distance = 0.0;

    g.post_grasp_retreat.direction.vector.x = -1.0;
    g.post_grasp_retreat.direction.vector.y = 0.0;
    g.post_grasp_retreat.direction.vector.z = 0.0;
    g.post_grasp_retreat.direction.header.frame_id = "arm_tool_link";
    g.post_grasp_retreat.desired_distance = 0.20;
    g.post_grasp_retreat.min_distance = 0.0;

    g.max_contact_force = 0.0;

    return g;
  }

  void grasp_object(geometry_msgs::PoseStamped object_pose) {
    ROS_INFO("Removing any previous 'part' object");
    // scene.remove_world_object("part");
    ROS_INFO("Adding new 'part' object");
    ROS_INFO("Object pose: ");
    // scene.add_box("part", object_pose,
    //(self.object_depth, self.object_width, self.object_height));
    ROS_INFO("Second ");

    moveit_msgs::Grasp grasp = create_grasp(object_pose);
    moveit_msgs::PickupGoal goal = createPickupGoal(object_pose, grasp);

    ROS_INFO("Sending goal");
    pickup_ac.sendGoal(goal);
    ROS_INFO("Waiting for result");
    pickup_ac.waitForResult();
    // result = pickup_ac.getResult();
    ROS_INFO("Pick result: ");
  }
  */
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_controller");
  MoveControlller move_controller;
  move_controller.run();

  return 0;
}