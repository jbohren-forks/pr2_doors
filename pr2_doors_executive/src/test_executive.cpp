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
 * $Id$
 *
 *********************************************************************/

/* Author: Wim Meeussen */

#include <cstdio>

#include <boost/thread/thread.hpp>
#include <door_msgs/Door.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <door_msgs/DoorAction.h>
#include <pr2_common_action_msgs/TuckArmsAction.h>
#include <pr2_common_action_msgs/SwitchControllersAction.h>

#include <pr2_doors_common/door_functions.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

using namespace ros;
using namespace std;
using namespace pr2_doors_common;
using namespace actionlib;

FILE* output = NULL;

Publisher test_output;

void writeString(std::string txt) {
  if (output)
    fprintf(output, "[Test Executive] %s at time %f sec.\n", txt.c_str(),ros::Time::now().toSec());
  // FIXME: @todo: dangerous, this does not check for bound...
  char tmp[2048];
  sprintf(tmp,"[Test Executive] %256s at time %30f sec.\n", txt.c_str(),ros::Time::now().toSec());
  std_msgs::String str;
  str.data = std::string(tmp);
  test_output.publish(str);
  ROS_INFO("[Test Executive] %s at time %f sec.", txt.c_str(),ros::Time::now().toSec());
}




void cleanup(actionlib::SimpleActionClient<pr2_common_action_msgs::SwitchControllersAction>& switch_controller)
{
  pr2_common_action_msgs::SwitchControllersGoal switch_goal;  
  switch_goal.start_controllers.clear();  switch_goal.stop_controllers.clear();
  switch_goal.start_controllers.push_back("r_arm_controller");
  switch_goal.stop_controllers.push_back("r_arm_cartesian_tff_controller");
  if (!ros::ok() || switch_controller.sendGoalAndWait(switch_goal, ros::Duration(5.0), ros::Duration(5.0)) != SimpleClientGoalState::SUCCEEDED)
  {
    writeString("Tuck arms: failed to start/stop r_arm_controller and r_arm_cartesian_tff_controller using switch_controller action.");
  }
}


// -----------------------------------
//              MAIN
// -----------------------------------

int
  main (int argc, char **argv)
{
  ros::init(argc, argv, "test_executive");
  boost::thread* thread;

  ros::NodeHandle node;
  test_output = node.advertise<std_msgs::String>("test_output", 10);

  writeString("Test executive started");

  door_msgs::Door prior_door;
  prior_door.frame_p1.x = 1.0;
  prior_door.frame_p1.y = -0.5;
  prior_door.frame_p2.x = 1.0;
  prior_door.frame_p2.y = 0.5;
  prior_door.door_p1.x = 1.0;
  prior_door.door_p1.y = -0.5;
  prior_door.door_p2.x = 1.0;
  prior_door.door_p2.y = 0.5;
  prior_door.travel_dir.x = 1.0;
  prior_door.travel_dir.y = 0.0;
  prior_door.travel_dir.z = 0.0;
  prior_door.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
  prior_door.hinge = door_msgs::Door::HINGE_P2;
  prior_door.header.frame_id = "base_footprint";
  Duration timeout = Duration().fromSec(5.0);

  writeString("Creating new action clients...");
  actionlib::SimpleActionClient<pr2_common_action_msgs::TuckArmsAction> tuck_arms("tuck_arms", true);
  actionlib::SimpleActionClient<pr2_common_action_msgs::SwitchControllersAction> switch_controller("switch_controllers", true);
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_local("pr2_move_base_local", true);
  actionlib::SimpleActionClient<door_msgs::DoorAction> detect_door("detect_door", true);
  actionlib::SimpleActionClient<door_msgs::DoorAction> detect_handle("detect_handle", true);
  actionlib::SimpleActionClient<door_msgs::DoorAction> touch_door("touch_door", true);
  actionlib::SimpleActionClient<door_msgs::DoorAction> push_door("push_door", true);
  actionlib::SimpleActionClient<door_msgs::DoorAction>  grasp_handle("grasp_handle", true);
  actionlib::SimpleActionClient<door_msgs::DoorAction> unlatch_handle("unlatch_handle", true);
  actionlib::SimpleActionClient<door_msgs::DoorAction> open_door("open_door", true);
  actionlib::SimpleActionClient<door_msgs::DoorAction> release_handle("release_handle", true);
  actionlib::SimpleActionClient<door_msgs::DoorAction> move_base_door("move_base_door", true);
  ros::ServiceClient move_base_clear = node.serviceClient<std_srvs::Empty>("move_base_local_node/clear_unknown_space");

  writeString("waiting for switch controller action server...");
  switch_controller.waitForServer();
  writeString("waiting for move base local action server...");
  move_base_local.waitForServer();
  writeString("waiting for detect door action server...");
  detect_door.waitForServer();
  writeString("waiting for detect handle action server...");
  detect_handle.waitForServer();
  //writeString("waiting for touch door action server...");
  //touch_door.waitForServer();
  //writeString("waiting for push door action server...");
  //push_door.waitForServer();
  writeString("waiting for grasp handle action server...");
  grasp_handle.waitForServer();
  writeString("waiting for unlatch handle action server...");
  unlatch_handle.waitForServer();
  writeString("waiting for open door action server...");
  open_door.waitForServer();
  writeString("waiting for release handle action server...");
  release_handle.waitForServer();
  writeString("waiting for move base door action server...");
  //move_base_door.waitForServer();
  //writeString("... Starting new action clients finished");

  door_msgs::DoorGoal door_goal;  
  pr2_common_action_msgs::SwitchControllersGoal switch_goal;  

  /************************************************************************/
  /*                                                                      */
  /*  based on all the repeated patterns in this executive                */
  /*  one should consider packaging the sendGoalAndWait into some macro   */
  /*  that does:                                                          */
  /*    1. verbose output of upcoming action                              */
  /*    2. sendGoalAndWait                                                */
  /*    3. verbose output of failure / retries / completion               */
  /*                                                                      */
  /************************************************************************/
  // tuck arm
  writeString("Tuck arms...");
  pr2_common_action_msgs::TuckArmsGoal  tuck_arms_goal;
  tuck_arms_goal.untuck = false;    tuck_arms_goal.left = true; tuck_arms_goal.right = true;
  if (!ros::ok() || tuck_arms.sendGoalAndWait(tuck_arms_goal, ros::Duration(30.0), ros::Duration(5.0)) != SimpleClientGoalState::SUCCEEDED)
  {
    writeString("Tuck arms: tuck_arms action failed to reach goal");
    return -1;
  }
  writeString("...Tuck arms finished");

  // detect door
  writeString("Detect door...");
  door_goal.door = prior_door;
  while (ros::ok() && detect_door.sendGoalAndWait(door_goal, ros::Duration(30.0), timeout) != SimpleClientGoalState::SUCCEEDED)
  {
    writeString("Detect door: detecting door in an potentially infinite loop.");
  }
  door_goal.door = detect_door.getResult()->door;
  writeString("...Detect door finished");
  std::ostringstream os2; os2 << door_goal.door;
  writeString("detect door " + os2.str());
  
  // detect handle if door is latched
  bool open_by_pushing = false;
  if (door_goal.door.latch_state == door_msgs::Door::UNLATCHED)
    open_by_pushing = true;

  if (!open_by_pushing){
    writeString("Detect handle...");
    while (ros::ok() && detect_handle.sendGoalAndWait(door_goal, ros::Duration(60.0), timeout) != SimpleClientGoalState::SUCCEEDED)
    {
      writeString("Detect handle: detecting handle in an potentially infinite loop.");
    }
    door_goal.door = detect_handle.getResult()->door;
    std::ostringstream os3; os3 << door_goal.door;
    writeString("detect handle " + os3.str());
  }


  // approach door
  move_base_msgs::MoveBaseGoal base_goal;
  tf::poseStampedTFToMsg(getRobotPose(door_goal.door, -0.7), base_goal.target_pose);
  std::ostringstream target; 
  target << base_goal.target_pose.pose.position.x << ", " << base_goal.target_pose.pose.position.y << ", " << base_goal.target_pose.pose.position.z;
  writeString("Move to pose " + target.str() + "...");
  while (ros::ok() && move_base_local.sendGoalAndWait(base_goal, ros::Duration(50.0), timeout) != SimpleClientGoalState::SUCCEEDED)
  {
    writeString("Move to pose: trying move_base_local in an potentially infinite loop.");
  }
  writeString("...Move to pose finished");

  // touch door
  if (open_by_pushing){
    writeString("Touch door...");
    if (!ros::ok() || touch_door.sendGoalAndWait(door_goal, ros::Duration(10.0), timeout) != SimpleClientGoalState::SUCCEEDED)
    {
      writeString("Touch door: failed to complete tuch_door action.");
      return -1;
    }
    writeString("...Touch door finished");

    // push door in separate thread
    writeString("Push door (separate thread)...");
    thread = new boost::thread(boost::bind(&SimpleActionClient<door_msgs::DoorAction>::sendGoalAndWait, 
					   &push_door, door_goal, ros::Duration(120.0), ros::Duration(40.0)));
  }
  else{
    // grasp handle
    writeString("Grasp handle...");
    if (!ros::ok() || grasp_handle.sendGoalAndWait(door_goal, ros::Duration(150.0), timeout) != SimpleClientGoalState::SUCCEEDED)
    {
      writeString("Grasp handle: failed to complete grasp_handle action.");
      return -1;
    }
    writeString("...Grasp handle finished");

    // unlatch handle
    writeString("Unlatch handle...");
    switch_goal.start_controllers.clear();  switch_goal.stop_controllers.clear();
    switch_goal.stop_controllers.push_back("r_arm_controller");
    switch_goal.start_controllers.push_back("r_arm_cartesian_tff_controller");
    if (!ros::ok() || switch_controller.sendGoalAndWait(switch_goal, ros::Duration(5.0), timeout) != SimpleClientGoalState::SUCCEEDED)
    {
      writeString("Unlatch handle: failed to start/stop r_arm_controller r_arm_cartesian_tff_controller with switch_controller action.");
      cleanup(switch_controller);
      return -1;
    }
    if (!ros::ok() || unlatch_handle.sendGoalAndWait(door_goal, ros::Duration(15.0), timeout) != SimpleClientGoalState::SUCCEEDED)
    {
      writeString("Unlatch handle: failed to complete unlatch_handle action.");
      cleanup(switch_controller);
      return -1;
    }
    writeString("...Unlatch handle finished");

    // open door in separate thread
    writeString("Open door (separate thread)...");
    thread = new boost::thread(boost::bind(&SimpleActionClient<door_msgs::DoorAction>::sendGoalAndWait, 
					   &open_door, door_goal, ros::Duration(120.0), ros::Duration(40.0)));
  }    

  // move throught door
  std::ostringstream os4; os4 << door_goal.door;
  writeString("Moving through door with door message " + os4.str());
  if (!ros::ok() || move_base_door.sendGoalAndWait(door_goal, ros::Duration(180.0), timeout) != SimpleClientGoalState::SUCCEEDED)
  {
    writeString("Moving through door: failed to complete move_base_door action.");
    cleanup(switch_controller);
    return -1;
  }
  writeString("...Moving through door finished");

  // preempt open/push door
  if (open_by_pushing){
    push_door.cancelAllGoals();
    writeString("Push door finished");
  }
  else{
    open_door.cancelAllGoals();
    writeString("Open door finished");

  }
  thread->join();
  delete thread;


  // release handle
  if (!open_by_pushing){
    writeString("Releasing handle...");
    switch_goal.start_controllers.clear();  switch_goal.stop_controllers.clear();
    switch_goal.stop_controllers.push_back("r_arm_cartesian_tff_controller");
    switch_goal.start_controllers.push_back("r_arm_controller");
    if (!ros::ok() || switch_controller.sendGoalAndWait(switch_goal, ros::Duration(5.0), timeout) != SimpleClientGoalState::SUCCEEDED)
    {
      writeString("Release handle: failed to start r_arm_cartesian_tff_controller r_arm_controller with switch_controller action.");
      return -1;
    }
    ros::Duration(2.0).sleep(); // workaround for ticket # 3954
    if (!ros::ok() || release_handle.sendGoalAndWait(door_goal, ros::Duration(10.0), timeout) != SimpleClientGoalState::SUCCEEDED)
    {
      writeString("Release handle: failed to complete release_handle action.");
      return -1;
    }
    writeString("...Releasing handle finished");
  }

  // tuck arm
  writeString("Tuck arm...");
  tuck_arms_goal.untuck = false;    tuck_arms_goal.left = false; tuck_arms_goal.right = true;
  if (!ros::ok() || tuck_arms.sendGoalAndWait(tuck_arms_goal, ros::Duration(30.0), ros::Duration(5.0)) != SimpleClientGoalState::SUCCEEDED)
  {
    writeString("Tuck arms: tuck_arms action failed to reach goal");
    return -1;
  }

  if (!ros::ok() || tuck_arms.sendGoalAndWait(tuck_arms_goal, ros::Duration(65.0), ros::Duration(5.0)) != SimpleClientGoalState::SUCCEEDED)
  {
    writeString("Tuck arm: (after release handle) failed to complete tuck_arms action.");
    return -1;
  }
  writeString("...Tuck arm finished");


  bool use_sim_time;
  node.param("use_sim_time", use_sim_time, false);

  // go to the last location
  if (use_sim_time){
    writeString("Clearing out the cost map.");
    std_srvs::Empty clear_srv;
    move_base_clear.call(clear_srv);


    double X = 27.3095662355 + 3 - 25.7, Y = 25.8414441058 - 25.7;
    std::ostringstream os6; os6 << "Moving to" << X << "," << Y;
    writeString(os6.str());
    base_goal.target_pose.header.frame_id = "/odom_combined";
    base_goal.target_pose.pose.position.x = X;
    base_goal.target_pose.pose.position.y = Y;
    base_goal.target_pose.pose.position.z = 0;
    base_goal.target_pose.pose.orientation.x = 0.0;
    base_goal.target_pose.pose.orientation.y = 0.0;
    base_goal.target_pose.pose.orientation.z = 0.0;
    base_goal.target_pose.pose.orientation.w = 1.0;
    writeString("Final phase: go to some goal past the door.");
    while (ros::ok() && move_base_local.sendGoalAndWait(base_goal, ros::Duration(60.0), timeout) != SimpleClientGoalState::SUCCEEDED)
      {
	writeString("Final phase: possibly infinite loop for move_base_local action.");
      };
    writeString("Final phase: finished.");
  }
  
  writeString("Done");
  return (0);
}
