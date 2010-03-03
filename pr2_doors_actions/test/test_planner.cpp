/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: test_executive.cpp 15054 2009-05-07 21:21:18Z meeussen $
 *
 *********************************************************************/

/* Author: Wim Meeussen, Sachin Chitta*/


#include <boost/thread/thread.hpp>
#include <door_msgs/Door.h>
// #include <ros/node.h>
#include <ros/ros.h>
#include <robot_actions/action_client.h>
#include <pr2_robot_actions/Pose2D.h>
#include <pr2_robot_actions/DoorActionState.h>
#include <robot_actions/NoArgumentsActionState.h>
#include <pr2_robot_actions/SwitchControllersState.h>
#include <nav_robot_actions/MoveBaseState.h>
#include <pr2_doors_common/door_functions.h>

#include <pr2_robot_actions/MoveArmGoal.h>
#include <pr2_robot_actions/MoveArmState.h>

using namespace ros;
using namespace std;
using namespace pr2_doors_common;


// -----------------------------------
//              MAIN
// -----------------------------------

int
  main (int argc, char **argv)
{
/*  ros::init(argc, argv);
  ros::Node node("test_planner");*/
	ros::init(argc, argv,"test_planner");
	
  boost::thread* thread;

  door_msgs::Door door;
  door.frame_p1.x = 1.0;
  door.frame_p1.y = -0.5;
  door.frame_p2.x = 1.0;
  door.frame_p2.y = 0.5;
  door.door_p1.x = 1.0;
  door.door_p1.y = -0.5;
  door.door_p2.x = 1.0;
  door.door_p2.y = 0.5;
  door.travel_dir.x = 1.0;
  door.travel_dir.y = 0.0;
  door.travel_dir.z = 0.0;
//  door.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
  door.rot_dir = door_msgs::Door::ROT_DIR_CLOCKWISE;
  door.hinge = door_msgs::Door::HINGE_P2;
  door.header.frame_id = "base_footprint";
    
  pr2_robot_actions::SwitchControllers switchlist;
  std_msgs::Empty empty;

  Duration timeout_short = Duration().fromSec(2.0);
  Duration timeout_medium = Duration().fromSec(10.0);
  Duration timeout_long = Duration().fromSec(1000.0);

  robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty> tuck_arm("safety_tuck_arms");
  robot_actions::ActionClient<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState,  std_msgs::Empty> switch_controllers("switch_controllers");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> detect_door("detect_door");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> detect_handle("detect_handle_no_camera");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> grasp_handle("grasp_handle");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> touch_door("touch_door");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> unlatch_handle("unlatch_handle");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> open_door("open_door");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> push_door("push_door");
  robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty> release_handle("release_handle");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> move_base_door("move_base_door");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> sbpl_door_planner("sbpl_door_planner");
  robot_actions::ActionClient<geometry_msgs::PoseStamped, nav_robot_actions::MoveBaseState, geometry_msgs::PoseStamped> move_base_local("move_base_local_old");
  robot_actions::ActionClient<pr2_robot_actions::MoveArmGoal, pr2_robot_actions::MoveArmState, int32_t> move_arm("move_arm");

  door_msgs::Door tmp_door;

  cout << "before " << door << endl;
  cout << "frame id " << door.header.frame_id << endl;
  // tuck arm
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("r_arm_joint_trajectory_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_medium) != robot_actions::SUCCESS) return -1;
  if (tuck_arm.execute(empty, empty, timeout_medium) != robot_actions::SUCCESS) return -1;
  cout << "tuck arms " << door << endl;

  // detect door
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("laser_tilt_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
  while (detect_door.execute(door, tmp_door, timeout_long) != robot_actions::SUCCESS)
  {};
  cout << "frame id in returned message is" << tmp_door.header.frame_id << endl;
  door = tmp_door;
  cout << "detected door " << door << endl;
  cout << "frame id is" << door.header.frame_id << endl;
  bool open_by_pushing = (door.latch_state == door_msgs::Door::UNLATCHED);
  open_by_pushing = false;
  // detect handle if door is latched
  if (!open_by_pushing){
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.start_controllers.push_back("head_controller");
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    while (detect_handle.execute(door, tmp_door, timeout_long) != robot_actions::SUCCESS)
    {};
    door = tmp_door;
    cout << "detect handle " << door << endl;
  }

  // approach door
  geometry_msgs::PoseStamped goal_msg;
  tf::poseStampedTFToMsg(getRobotPose(door, -0.6), goal_msg);
  cout << "move to pose " << goal_msg.pose.position.x << ", " << goal_msg.pose.position.y << ", "<< goal_msg.pose.position.z << endl;
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
  if (move_base_local.execute(goal_msg, goal_msg, timeout_long) != robot_actions::SUCCESS) return -1;
  cout << "door approach finished" << endl;

  // touch door
  if (open_by_pushing){
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");
    switchlist.start_controllers.push_back("r_gripper_effort_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    if (touch_door.execute(door, tmp_door, timeout_long) != robot_actions::SUCCESS) return -1;
    cout << "door touched" << endl;

    // push door in separate thread
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    thread = new boost::thread(boost::bind(&robot_actions::ActionClient<door_msgs::Door, 
					   pr2_robot_actions::DoorActionState, door_msgs::Door>::execute, 
					   &push_door, door, tmp_door, timeout_long));
    // move throught door
    pr2_robot_actions::Pose2D pose2d;
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    if (move_base_door.execute(door, tmp_door) != robot_actions::SUCCESS) return -1;
    push_door.preempt();

  thread->join();
  delete thread;
  }
  else{
    // grasp handle
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");
    switchlist.start_controllers.push_back("r_gripper_effort_controller");

    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    if (grasp_handle.execute(door, tmp_door, timeout_long) != robot_actions::SUCCESS) return -1;

    // unlatch handle
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
    switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
    switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
    switchlist.start_controllers.push_back("r_arm_cartesian_tff_controller");
    cout << "Switching controllers for unlatch handle" << endl;
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;

    cout << "Switching controllers finished" << endl;

    if (unlatch_handle.execute(door, tmp_door, timeout_long) != robot_actions::SUCCESS) return -1;
    cout << "Unlatch handle finished" << endl;

    // open door in separate thread
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.stop_controllers.push_back("r_arm_cartesian_tff_controller");
    switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");
    switchlist.start_controllers.push_back("r_arm_joint_trajectory_controller");
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;

    cout << "Controllers for door planner ready" << endl;

    // move through door
    pr2_robot_actions::Pose2D pose2d;
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    if (sbpl_door_planner.execute(door, tmp_door) != robot_actions::SUCCESS) return -1;
    cout << "Door planner done" << endl;
  }    

  // release handle
  if (!open_by_pushing){
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();

    switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");

    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");


    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    if (release_handle.execute(empty, empty, timeout_long) != robot_actions::SUCCESS) return -1;
  }

  ROS_INFO("Door angle is now : %f",getDoorAngle(tmp_door));

  if(fabs(getDoorAngle(tmp_door)) < (M_PI/2.0-0.3))
  {
    // Repeat the door opening
    // First use move arm to grasp the other handle
    // Then use the door opening code to open the door
    switchlist.start_controllers.push_back("r_arm_joint_trajectory_controller");
    switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
    switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
    switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
    switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;

    tf::Stamped<tf::Pose> handle_pose = getHandlePose(tmp_door,-1);
    geometry_msgs::PoseStamped handle_msg;
    handle_pose.stamp_ = ros::Time::now();
    poseStampedTFToMsg(handle_pose, handle_msg);

    int32_t feedback_move_arm;
    pr2_robot_actions::MoveArmGoal goal_move_arm;
    pr2_robot_actions::MoveArmState state;

    goal_move_arm.goal_constraints.set_pose_constraint_size(1);

    goal_move_arm.goal_constraints.pose_constraint[0].pose = handle_msg;
    goal_move_arm.goal_constraints.pose_constraint[0].pose.header.stamp = ros::Time::now();
    goal_move_arm.goal_constraints.pose_constraint[0].pose.header.frame_id = "torso_lift_link";

    goal_move_arm.goal_constraints.pose_constraint[0].link_name = "r_wrist_roll_link";
		goal_move_arm.goal_constraints.pose_constraint[0].position_tolerance_above.x = 0.005;
		goal_move_arm.goal_constraints.pose_constraint[0].position_tolerance_below.x = 0.005;
		goal_move_arm.goal_constraints.pose_constraint[0].position_tolerance_above.y = 0.005;
		goal_move_arm.goal_constraints.pose_constraint[0].position_tolerance_below.y = 0.005;
		goal_move_arm.goal_constraints.pose_constraint[0].position_tolerance_above.z = 0.005;
		goal_move_arm.goal_constraints.pose_constraint[0].position_tolerance_below.z = 0.005;

		goal_move_arm.goal_constraints.pose_constraint[0].orientation_tolerance_above.x = 0.01;
		goal_move_arm.goal_constraints.pose_constraint[0].orientation_tolerance_below.x = 0.01;
		goal_move_arm.goal_constraints.pose_constraint[0].orientation_tolerance_above.y = 0.01;
		goal_move_arm.goal_constraints.pose_constraint[0].orientation_tolerance_below.y = 0.01;
		goal_move_arm.goal_constraints.pose_constraint[0].orientation_tolerance_above.z = 0.01;
		goal_move_arm.goal_constraints.pose_constraint[0].orientation_tolerance_below.z = 0.01;

    goal_move_arm.goal_constraints.pose_constraint[0].orientation_importance = 0.1;
		goal_move_arm.goal_constraints.pose_constraint[0].type = motion_planning_msgs::PoseConstraint::POSITION_X + motion_planning_msgs::PoseConstraint::POSITION_Y + motion_planning_msgs::PoseConstraint::POSITION_Z + 
				+ motion_planning_msgs::PoseConstraint::ORIENTATION_R + motion_planning_msgs::PoseConstraint::ORIENTATION_P + motion_planning_msgs::PoseConstraint::ORIENTATION_Y;   
    if(move_arm.execute(goal_move_arm,feedback_move_arm,timeout_long) != robot_actions::SUCCESS) return -1;    

    // grasp handle
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");

    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    if (grasp_handle.execute(tmp_door, tmp_door, timeout_long) != robot_actions::SUCCESS) return -1;

    cout << "Controllers for door planner ready" << endl;

    // move through door
    pr2_robot_actions::Pose2D pose2d;
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    if (sbpl_door_planner.execute(tmp_door, tmp_door) != robot_actions::SUCCESS) return -1;
    cout << "Door planner done" << endl;
  }
    
  // release handle
  if (!open_by_pushing){
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();

    switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");

    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");

    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    if (release_handle.execute(empty, empty, timeout_long) != robot_actions::SUCCESS) return -1;
  }



  // tuck arm
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();

  switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
  switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
  switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
  switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");
  switchlist.start_controllers.push_back("r_arm_joint_trajectory_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_medium) != robot_actions::SUCCESS) return -1;
  if (tuck_arm.execute(empty, empty, timeout_medium) != robot_actions::SUCCESS) return -1;

  // stop remaining controllers
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("laser_tilt_controller");
  switchlist.stop_controllers.push_back("head_controller");
  switchlist.stop_controllers.push_back("r_gripper_effort_controller");
  switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_medium) != robot_actions::SUCCESS) return -1;

  return (0);
}
