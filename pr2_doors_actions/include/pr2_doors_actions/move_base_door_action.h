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
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <pr2_doors_actions/door_reactive_planner.h>

#include <door_msgs/DoorGoal.h>
#include <door_msgs/DoorAction.h>
#include <actionlib/server/simple_action_server.h>
#include <manipulation_msgs/JointTraj.h>

#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <door_msgs/Door.h>

#include <angles/angles.h>
#include <vector>
#include <string>



namespace nav {
  /**
   * @class MoveBaseDoorAction
   * @brief A class adhering to the robot_actions::Action interface that moves the robot base to a goal location.
   */
  class MoveBaseDoorAction{
    public:
      /**
       * @brief  Constructor for the actions
       */
      MoveBaseDoorAction();

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBaseDoorAction();

      /**
       * @brief  Runs whenever a new goal is sent to the move_base
       * @param goal The goal to pursue 
       */
      void execute(const door_msgs::DoorGoalConstPtr& goal);

    private:
      /**
       * @brief  Publishes the footprint of the robot for visualization purposes
       */
      void publishFootprint();

      /**
       * @brief  Publish a path for visualization purposes
       */
      void publishPath(const std::vector<geometry_msgs::Pose2D>& path, const ros::Publisher &publisher, double r, double g, double b, double a);

      /**
       * @brief  Make a new global plan (the goal is specified using the setDoor() function while start is specified using the 
       * current position of the robot internally
       */
      void makePlan();

      /**
       * @brief  Trim off parts of the global plan that are far enough behind the robot
       */
      void prunePlan();

      /**
       * @brief  Check if the goal orientation has been achieved
       * @return True if achieved, false otherwise
       */
      bool goalOrientationReached();

      /**
       * @brief  Check if the goal position has been achieved
       * @return True if achieved, false otherwise
       */
      bool goalPositionReached();

      /**
       * @brief Check if the robot is inside the doorway
       */
      bool goalReached();

      /**
       * @brief  Compute the distance between two points
       * @param x1 The first x point 
       * @param y1 The first y point 
       * @param x2 The second x point 
       * @param y2 The second y point 
       * @return 
       */
      double distance(double x1, double y1, double x2, double y2);

      /**
       * @brief  Get the current pose of the robot in the global frame and set the global_pose_ variable
       */
      void updateGlobalPose();

      /**
       * @brief  Clear the footprint of the robot in a given cost map
       * @param cost_map The costmap to apply the clearing opertaion on
       */
      //void clearRobotFootprint(costmap_2d::Costmap2D& cost_map);

      /**
       * @brief  Resets the costmaps to the static map outside a given window
       */
      void resetCostmaps();

      actionlib::SimpleActionServer<door_msgs::DoorAction> action_server_;
      door_msgs::DoorResult action_result_;

      ros::NodeHandle ros_node_;
      tf::TransformListener tf_;
      bool run_planner_;
      costmap_2d::Costmap2DROS* planner_cost_map_ros_;
      costmap_2d::Costmap2D planner_cost_map_;

      door_reactive_planner::DoorReactivePlanner* planner_;
      std::vector<geometry_msgs::Pose2D> global_plan_;
      std::vector<geometry_msgs::Point> footprint_;
      std::string global_frame_, control_frame_, robot_base_frame_;
      bool valid_plan_;
      door_msgs::Door door_;
      geometry_msgs::Pose2D goal_;

      tf::Stamped<tf::Pose> global_pose_;
      double xy_goal_tolerance_, yaw_goal_tolerance_, min_abs_theta_vel_;
      double inscribed_radius_, circumscribed_radius_, inflation_radius_;
      double controller_frequency_;

      std::vector<geometry_msgs::Pose2D> empty_plan_;

      std::string control_topic_name_;

    /**
     * @brief Transform a tf::Stamped<tf::Pose> to geometry_msgs::Pose2D
     * @param pose The pose object to be transformed
     * @return Transformed object of type geometry_msgs::Pose2D
     */
    geometry_msgs::Pose2D getPose2D(const tf::Stamped<tf::Pose> &pose);

    /**
     * @brief dispatch control commands
     * @param plan_in The plan to be dispatched
     */
    void dispatchControl(const std::vector<geometry_msgs::Pose2D> &plan_in);

    int plan_size_;

    std::string plan_state_;

    void publishDiagnostics(bool force);

    double current_distance_to_goal_;

    double diagnostics_expected_publish_time_;

    ros::Time last_diagnostics_publish_time_;

    double action_max_allowed_time_;

    ros::Publisher global_plan_publisher_, local_plan_publisher_, robot_footprint_publisher_, diagnostics_publisher_;

    ros::Publisher control_topic_publisher_;

  };
};
#endif

