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

#include "pr2_doors_actions/action_open_door.h"
#include <actionlib/client/simple_action_client.h>
#include <door_msgs/DoorAction.h>
#include <door_msgs/Door.h>
#include <ros/ros.h>

using namespace ros;
using namespace std;
using namespace door_handle_detector;

// -----------------------------------
//              MAIN
// -----------------------------------

int
  main (int argc, char **argv)
{
  ros::init(argc, argv, "trigger_doors_opener");

  // goal
  door_msgs::DoorGoal door_goal;
  door_goal.door.frame_p1.x = 1.0;
  door_goal.door.frame_p1.y = -0.5;
  door_goal.door.frame_p2.x = 1.0;
  door_goal.door.frame_p2.y = 0.5;
  door_goal.door.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
  door_goal.door.hinge = door_msgs::Door::HINGE_P2;
  door_goal.door.header.frame_id = "base_footprint";

  // create action server
  tf::TransformListener tf;
  door_handle_detector::OpenDoorAction door_opener(tf);

  // create action client
  actionlib::SimpleActionClient<door_msgs::DoorAction> action_client("open_door", true);
  cout << "waiting for action server to start..." << endl;
  boost::thread(boost::bind(&ros::spin));
  action_client.waitForServer();
  cout << "... started" << endl;

  return (0);
}
