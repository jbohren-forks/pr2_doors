/*********************************************************************
*
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
*   * Neither the name of the Willow Garage nor the names of its
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
* Author: Sachin Chitta
*********************************************************************/
#include <pr2_doors_actions/move_base_door_action.h>
#include <pr2_doors_actions/door_reactive_planner.h>
#include <pr2_doors_common/door_functions.h>
#include <kdl/frames.hpp>
#include <ros/rate.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PolygonStamped.h>

#include <diagnostic_msgs/DiagnosticArray.h>



using namespace base_local_planner;
using namespace costmap_2d;
using namespace door_reactive_planner;
using namespace pr2_doors_common;

namespace nav 
{
  MoveBaseDoorAction::MoveBaseDoorAction() : 
  action_server_(ros::NodeHandle(), 
		 "move_base_door", 
		 boost::bind(&MoveBaseDoorAction::execute, this, _1)),
    run_planner_(true), planner_cost_map_ros_(NULL),  
    planner_(NULL), valid_plan_(false) 
  {
    ros::NodeHandle root_node;
    //get some parameters that will be global to the move base door node
    ros_node_.param("global_frame", global_frame_, std::string("odom_combined"));
    ros_node_.param("control_frame", control_frame_, std::string("odom_combined"));
    ros_node_.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    ros_node_.param("controller_frequency", controller_frequency_, 20.0);
    ros_node_.param("xy_goal_tolerance", xy_goal_tolerance_, 0.05);
    ros_node_.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.1);
    ros_node_.param("diagnostics_expected_publish_time",diagnostics_expected_publish_time_,0.2);

    control_topic_name_ = "base_trajectory_controller/command";
    ros_node_.param("action_max_allowed_time", action_max_allowed_time_,5.0);

    //for display purposes
    global_plan_publisher_ = ros_node_.advertise<nav_msgs::Path>("global_plan", 1);
    local_plan_publisher_ = ros_node_.advertise<nav_msgs::Path>("local_plan", 1);
    robot_footprint_publisher_ = ros_node_.advertise<geometry_msgs::PolygonStamped>("robot_footprint", 1);
    diagnostics_publisher_ = ros_node_.advertise<diagnostic_msgs::DiagnosticArray> ("/diagnostics", 1) ;

    //pass on some parameters to the components of the move base node if they are not explicitly overridden 
    //(perhaps the controller and the planner could operate in different frames)
    ros_node_.param("inscribed_radius", inscribed_radius_, 0.305);
    ros_node_.param("circumscribed_radius", circumscribed_radius_, 0.46);
    ros_node_.param("inflation_radius", inflation_radius_, 0.70);

    empty_plan_.resize(0);

    //pass on inflation parameters to the planner's costmap if they're not set explicitly
    if(!ros_node_.hasParam("costmap/inscribed_radius")) ros_node_.setParam("ostmap/inscribed_radius", inscribed_radius_);
    if(!ros_node_.hasParam("costmap/circumscribed_radius")) ros_node_.setParam("costmap/circumscribed_radius", circumscribed_radius_);
    if(!ros_node_.hasParam("costmap/inflation_radius")) ros_node_.setParam("costmap/inflation_radius", inflation_radius_);
    if(!ros_node_.hasParam("costmap/global_frame")) ros_node_.setParam("costmap/global_frame", std::string("odom_combined"));
    if(!ros_node_.hasParam("costmap/robot_base_frame")) ros_node_.setParam("costmap/robot_base_frame", std::string("base_link"));

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_cost_map_ros_ = new Costmap2DROS("costmap", tf_);
    planner_cost_map_ros_->getCostmapCopy(planner_cost_map_);

    //we'll stop our costmap from running until we are activated
    planner_cost_map_ros_->stop();

    //initialize the door opening planner
    planner_ = new DoorReactivePlanner(ros_node_, tf_,&planner_cost_map_,control_frame_,global_frame_);
    ROS_INFO("MAP SIZE: %d, %d", planner_cost_map_.getSizeInCellsX(), planner_cost_map_.getSizeInCellsY());

    control_topic_publisher_ = root_node.advertise<manipulation_msgs::JointTraj>(control_topic_name_, 1);
    last_diagnostics_publish_time_ = ros::Time::now();

    ROS_INFO("Move base door action initialized");
  }

  MoveBaseDoorAction::~MoveBaseDoorAction()
  {
    if(planner_ != NULL)
      delete planner_;

    if(planner_cost_map_ros_ != NULL)
      delete planner_cost_map_ros_;
  }

  geometry_msgs::Pose2D MoveBaseDoorAction::getPose2D(const tf::Stamped<tf::Pose> &pose)
  {
    geometry_msgs::Pose2D tmp_pose;
    double useless_pitch, useless_roll, yaw;
    pose.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);
    tmp_pose.x = pose.getOrigin().x();
    tmp_pose.y = pose.getOrigin().y();
    tmp_pose.theta = yaw;
    return tmp_pose;
  }

  /*
  void MoveBaseDoorAction::clearRobotFootprint(Costmap2D& cost_map)
  {
    double useless_pitch, useless_roll, yaw;
    global_pose_.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);

    //get the oriented footprint of the robot
    std::vector<diagnostic_msgs::Point> oriented_footprint;
    geometry_msgs::Pose2D tmp_pose = getPose2D(global_pose_);
    planner_->computeOrientedFootprint(tmp_pose, planner_->footprint_, oriented_footprint);

    //set the associated costs in the cost map to be free
    if(!cost_map.setConvexPolygonCost(oriented_footprint, costmap_2d::FREE_SPACE))
      return;

    double max_inflation_dist = inflation_radius_ + inscribed_radius_;

    //make sure to re-inflate obstacles in the affected region
    cost_map.reinflateWindow(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), max_inflation_dist, max_inflation_dist);
  }
  */

  void MoveBaseDoorAction::makePlan()
  {
    //since this gets called on handle activate
    if(planner_cost_map_ros_ == NULL)
      return;

    //make a plan for controller
    planner_cost_map_ros_->clearRobotFootprint();
    planner_cost_map_ros_->getCostmapCopy(planner_cost_map_);

    //make sure we clear the robot's footprint from the cost map
    //clearRobotFootprint(planner_cost_map_);

    std::vector<geometry_msgs::Pose2D> global_plan;
    bool valid_plan = planner_->makePlan(getPose2D(global_pose_), global_plan,&planner_cost_map_);//makePlan(current_position, return_path, &planner_cost_map_);

    valid_plan_ = valid_plan;
    global_plan_ = global_plan;
    publishPath(global_plan,global_plan_publisher_, 0.0, 1.0, 0.0, 0.0);
  }

  void MoveBaseDoorAction::updateGlobalPose()
  {
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time();

    try{
      tf_.transformPose(global_frame_, robot_pose, global_pose_);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    }
  }

  double MoveBaseDoorAction::distance(double x1, double y1, double x2, double y2)
  {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  bool MoveBaseDoorAction::goalPositionReached()
  {
    double dist = distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), goal_.x, goal_.y);
    return fabs(dist) <= xy_goal_tolerance_;
  }

  bool MoveBaseDoorAction::goalOrientationReached()
  {
    double useless_pitch, useless_roll, yaw;
    global_pose_.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);
    return fabs(angles::shortest_angular_distance(yaw, goal_.theta)) <= yaw_goal_tolerance_;
  }

  bool MoveBaseDoorAction::goalReached()
  {
    KDL::Vector normal = getFrameNormal(door_);
    double goal_projection = normal(0)*(global_pose_.getOrigin().x()-goal_.x) + normal(1)*(global_pose_.getOrigin().y()-goal_.y);
    if((goalPositionReached() && goalOrientationReached()) || goal_projection > 0)
    { 
      ROS_INFO("Normal: %f %f, Global pose: %f %f, Goal: %f %f",normal(0),normal(1),global_pose_.getOrigin().x(),global_pose_.getOrigin().y(),goal_.x,goal_.y);
      ROS_INFO("Travel direction: %f %f",door_.travel_dir.x,door_.travel_dir.y);
      return true;
    }
    return false;
  }

  void MoveBaseDoorAction::prunePlan()
  {
    std::vector<geometry_msgs::Pose2D>::iterator it = global_plan_.begin();
    while(it != global_plan_.end()){
      const geometry_msgs::Pose2D& w = *it;
      // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
      double x_diff = global_pose_.getOrigin().x() - w.x;
      double y_diff = global_pose_.getOrigin().y() - w.y;
      double distance = sqrt(x_diff * x_diff + y_diff * y_diff);
      if(distance < 1){
        ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), w.x, w.y);
        break;
      }
      it = global_plan_.erase(it);
    }
  }

  void MoveBaseDoorAction::execute(const door_msgs::DoorGoalConstPtr& goal)
  {
    door_msgs::Door door_transformed;
    planner_->door_information_set_ = false;
    //update the global pose
    updateGlobalPose();

    //we'll start our costmap up now that we're active
    planner_cost_map_ros_->start();
    planner_cost_map_ros_->clearNonLethalWindow(circumscribed_radius_ * 4, circumscribed_radius_ * 4);


    planner_->setDoor(goal->door,getPose2D(global_pose_),door_transformed);//set the goal into the planner
    door_ = door_transformed;
    if(!planner_->getGoal(goal_)){
      //make sure to stop the costmap from running on return
      planner_cost_map_ros_->stop();
      action_server_.setPreempted();
      return;
    }

    updateGlobalPose();    //update the global pose
    makePlan();    //first... make a plan

    ros::Rate control_rate(controller_frequency_);

    double abort_action_timeout = 0.0;

    ros::Time last_time = ros::Time::now();

    while(ros::ok() && !action_server_.isPreemptRequested())
    {
      //get the start time of the loop
      ros::Time start_time = ros::Time::now();

      //update the global pose
      updateGlobalPose();

      //make sure to update the cost_map we'll use for this cycle
      //controller_cost_map_ros_->getCostmapCopy(controller_cost_map_);

      //make sure that we clear the robot footprint in the cost map
      //clearRobotFootprint(controller_cost_map_);

      //check for success
      if(goalReached())
      {
        ROS_INFO("REACHED GOAL");
        dispatchControl(empty_plan_);
        plan_state_ = "REACHED GOAL";
        publishDiagnostics(true);
        //make sure to stop the costmap from running on return
        planner_cost_map_ros_->stop();
	ros::Duration(3.0).sleep();
	action_result_.door = goal->door;
	action_server_.setSucceeded(action_result_);  
        return;
      }
      else 
      {
        //check that the observation buffers for the costmap are current
        if(!planner_cost_map_ros_->isCurrent()){
	  ROS_DEBUG("Sensor data is out of date, we're not going to allow commanding of the base for safety");
          plan_state_ = "Sensor data out of date";
          continue;
        }
        makePlan();
        //pass plan to controller
        if(valid_plan_)
        {
          plan_state_ = "Plan valid";
          dispatchControl(global_plan_);
          abort_action_timeout = 0.0; 
        }
        else
        {
          abort_action_timeout += (start_time - last_time).toSec();
          plan_state_ = "Plan invalid";
          dispatchControl(empty_plan_);
        }

        //for visualization purposes
        publishPath(global_plan_,global_plan_publisher_, 0.0, 1.0, 0.0, 0.0);
	publishFootprint();

      }
      if(!control_rate.sleep())
      {
        ROS_WARN("Control loop missed its desired cycle frequency of %.4f Hz", controller_frequency_);
        plan_state_ = "Control loop missed cycle time";
      }
      publishDiagnostics(false);
      if(abort_action_timeout > 2*action_max_allowed_time_)
	{
        ROS_ERROR("Move base door action timing out");
        planner_cost_map_ros_->stop();
	action_server_.setPreempted();
	return;
	}
      if(abort_action_timeout > action_max_allowed_time_)
      {
	planner_->cell_distance_from_obstacles_ = std::max(planner_->cell_distance_from_obstacles_-1,0);
	ROS_INFO("Resetting cell distance from obstacles to %d and trying again",planner_->cell_distance_from_obstacles_);
      }
      last_time = start_time;
    }
    //make sure to stop the costmap from running on return
    planner_cost_map_ros_->stop();
    action_server_.setPreempted();
  }

  void MoveBaseDoorAction::dispatchControl(const std::vector<geometry_msgs::Pose2D> &plan_in)
  {
    manipulation_msgs::JointTraj plan_out;
    current_distance_to_goal_ = distance(global_pose_.getOrigin().x(),global_pose_.getOrigin().y(),goal_.x,goal_.y);
    if((int)plan_in.size() <= 0)
    {
      plan_size_ = 0;
      plan_out.set_points_size(0);
      ROS_DEBUG("Sending empty plan");
    }
    else
    {
      int index_plan = std::max((int) plan_in.size() - 1, 0);
      plan_size_ = plan_in.size();
      plan_out.set_points_size(1);
      plan_out.points[0].set_positions_size(3);
      plan_out.points[0].positions[0] = plan_in[index_plan].x;
      plan_out.points[0].positions[1] = plan_in[index_plan].y;
      plan_out.points[0].positions[2] = plan_in[index_plan].theta;
      plan_out.points[0].time = 0.0;        
      ROS_DEBUG("Plan in had %d points, plan out has %d points",(int)plan_in.size(),(int)plan_out.points.size());
    }
    control_topic_publisher_.publish(plan_out);
  }

  void MoveBaseDoorAction::resetCostmaps(){
    planner_cost_map_ros_->resetMapOutsideWindow(5.0, 5.0);
  }

  void MoveBaseDoorAction::publishFootprint(){
    double useless_pitch, useless_roll, yaw;
    global_pose_.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);
    std::vector<geometry_msgs::Point> footprint;
    planner_->computeOrientedFootprint(getPose2D(global_pose_), planner_->footprint_, footprint);

    geometry_msgs::PolygonStamped footprint_msg;
    footprint_msg.header.frame_id = global_frame_;
    footprint_msg.polygon.set_points_size(footprint.size());
    for(unsigned int i = 0; i < footprint.size(); ++i){
      footprint_msg.polygon.points[i].x = footprint[i].x;
      footprint_msg.polygon.points[i].y = footprint[i].y;
      footprint_msg.polygon.points[i].z = footprint[i].z;
      ROS_DEBUG("Footprint:%d:: %f, %f\n",i,footprint[i].x,footprint[i].y);
    }
    robot_footprint_publisher_.publish(footprint_msg);
  }

  void MoveBaseDoorAction::publishPath(const std::vector<geometry_msgs::Pose2D>& path, const ros::Publisher &publisher, double r, double g, double b, double a){
    // Extract the plan in world co-ordinates
    nav_msgs::Path gui_path_msg;
    gui_path_msg.header.frame_id = global_frame_;
    gui_path_msg.set_poses_size(path.size());
    for(unsigned int i=0; i < path.size(); i++){
      gui_path_msg.poses[i].pose.position.x = path[i].x;
      gui_path_msg.poses[i].pose.position.y = path[i].y;
      gui_path_msg.poses[i].pose.position.z = 0;
    }

    publisher.publish(gui_path_msg);
  }

  void MoveBaseDoorAction::publishDiagnostics(bool force)
  {
    if((ros::Time::now() - last_diagnostics_publish_time_).toSec() <= diagnostics_expected_publish_time_ && !force)
    {
      return;
    }

    diagnostic_msgs::DiagnosticArray message;
    std::vector<diagnostic_msgs::DiagnosticStatus> statuses;

    diagnostic_msgs::DiagnosticStatus status_planner = planner_->getDiagnostics();
    status_planner.message = plan_state_ + " (" + door_.header.frame_id + ")";

    statuses.push_back(status_planner);

    message.header.stamp = ros::Time::now();
    message.status = statuses;
    diagnostics_publisher_.publish(message);
    last_diagnostics_publish_time_ = message.header.stamp;
  }
};


int main(int argc, char** argv)
{
  ros::init(argc,argv,"move_base_door_action"); 

  nav::MoveBaseDoorAction move_base_door;

  ros::spin();
  return 0;
}
