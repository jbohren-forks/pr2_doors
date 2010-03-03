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
 *********************************************************************/

#pragma once

#include <ros/ros.h>

//messages
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <door_msgs/Door.h>

// For transform support
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// costmap
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/costmap_model.h>

//angles
#include <angles/angles.h>

namespace door_reactive_planner
{

  /** @class DoorReactivePlanner
   *  @brief A reactive planner/controller to move the robot through a door
   *  @author Sachin Chitta <sachinc@willowgarage.com>
   */
  class DoorReactivePlanner
  {
    public:

    /**
     * @brief Constructor
     * @param ros_node A reference to a ros node
     * @param tf A reference to a transform listening object
     * @param cost_map A pointer to a costmap object
     * @param path_frame_id The name of the frame in which all planning and control is being performed
     * @param costmap_frame_id The name of the frame in which the costmap exists
     */
    DoorReactivePlanner(ros::NodeHandle &ros_node, tf::TransformListener& tf, costmap_2d::Costmap2D* cost_map, std::string path_frame_id, std::string costmap_frame_id);

    /**
     * @brief Make and return the plan 
     * @param start The position from which the robot is starting
     * @param best_path The best path returned by the planner (note this could be a zero length path if no plan is found
     */
    bool makePlan(const geometry_msgs::Pose2D &start, std::vector<geometry_msgs::Pose2D> &best_path, costmap_2d::Costmap2D *cost_map);

    /**
     * @brief Set door information for the planner
     * @param door_msg_in The door message containing information about the door
     */
    void setDoor(door_msgs::Door door_msg_in, const geometry_msgs::Pose2D &pose, door_msgs::Door &door_msg_out);

    /**
     * @brief compute the oriented footprint for a particular position of the robot 
     * @param position Position of the robot
     * @param footprint_spec The footprint of the robot
     * @param oriented_footprint The oriented footprint of the robot
     */
    bool computeOrientedFootprint(const geometry_msgs::Pose2D &position, const std::vector<geometry_msgs::Point>& footprint_spec, std::vector<geometry_msgs::Point>& oriented_footprint); 

    std::vector<geometry_msgs::Point> footprint_; /**< The footprint of the robot */

    /**
     * @brief Get the goal position from the planner
     * @return The goal position for the planner
     */
    bool getGoal(geometry_msgs::Pose2D &goal);

    diagnostic_updater::DiagnosticStatusWrapper getDiagnostics();

    bool door_information_set_ ; /**< Has door information been set before invoking the planner */

    int cell_distance_from_obstacles_; /**< Distance the robot should stay away from obstacles (in cells corresponding to the grid used for planning */

    private:

    std::string current_position_in_collision_;

    ros::NodeHandle &node_;/**< A reference to a ros node */
   
    tf::TransformListener &tf_; /**< A reference to a transform listener */

    costmap_2d::Costmap2D *cost_map_; /**< Pointer to the cost map used by the planner */

    base_local_planner::CostmapModel *cost_map_model_; /**< Pointer to a CostmapModel used by the planner */
   
    std::string control_frame_id_; /**< The name of the frame in which all planning and control is carried out */

    std::string costmap_frame_id_; /**< The name of the frame in which the cost map exists */

    bool choose_straight_line_trajectory_; /**< Externally set using a ROS param, default value is true, if true always pick the straightline path along the centerline if it exists */

    double inscribed_radius_; /**< Inscribed radius of the robot */

    double circumscribed_radius_; /**< Circumscribed radius of the robot */

    double min_distance_from_obstacles_; /**< Minimum distance that the robot must stay away from obstacles */

    double dist_waypoints_max_; /**< Maximum translational distance between consecutive points in the plan */

    double dist_rot_waypoints_max_; /**< Max rotational distance between consecutive points in the plan */

    double max_explore_distance_; /**< Maximum explore distance for laying out and exploring paths */

    double horizontal_explore_distance_; /**< Maximum horizonal distance (along the length of the door frame) that the robot can stray from the centerline of the doorway */

    double max_explore_delta_angle_; /**< Maximum angle (from the centerline of the doorway) to explore paths */

    double door_goal_distance_; /**< Distance from the midpoint of the door to the goal position on the other side of the door (from the robot) along the centerline */

    int num_explore_paths_; /**< Number of paths to explore */

    double max_inflated_cost_; /**< Maximum inflated cost for the point that the robot center is allowed to occupy */


    int cell_distance_robot_center_from_obstacles_;

    geometry_msgs::Vector3 vector_along_door_; /**< A unit vector along the doorway*/

    double centerline_angle_; /**< Angle that the normal to the door makes in the path_frame when the door is closed */

    double travel_angle_; /**< Angle that the normal to the door makes in the path_frame when the door is closed */

    geometry_msgs::Pose2D goal_; /**< Goal position on the other side of the doorway */

      geometry_msgs::Pose2D carrot_;
      double carrot_distance_;

    void getParams(); /**< Check ROS param server for parameters */

    /**
     * @brief Translational distance between two positions
     * @param p Position of the robot
     * @param q Position of the robot
     */
    double distance(const geometry_msgs::Pose2D &p, const geometry_msgs::Pose2D &q);

    /**
     * @brief Translational projected distance between two positions
     * @param p Position of the robot
     * @param q Position of the robot
     * @param angle angle of the vector to project onto of the robot
     */
    double projectedDistance(const geometry_msgs::Pose2D &p, const geometry_msgs::Pose2D &q, const double &angle);

    /**
     * @brief Create a linear path between two positions
     * @param cp First position
     * @param fp Final position
     * @param return_path The returned path
     */
    bool createLinearPath(const geometry_msgs::Pose2D &cp,const geometry_msgs::Pose2D &fp, std::vector<geometry_msgs::Pose2D> &return_path);

    /**
     * @brief Get the final position corresponding to motion along a straight line at angle (delta_angle) from the centerline_angle. The final position
     * is restricted to stay within a horizontal band with width 2*horizontal_explore_distance_
     * @param current_postion Current position of the robot
     * @param delta_angle The delta angle from the centerline of the doorway
     * @param distance_to_centerline shortest distance from the current position to the centerline of the doorway
     * @param end_position final position returned by this function
     */
    void getFinalPosition(const geometry_msgs::Pose2D &current_position, const double &delta_angle, const double &distance_to_centerline, geometry_msgs::Pose2D &end_position);

    /**
     * @brief The cost of a point in the cost map
     * @param position Position of the robot
     * @param cost cost of the input position
     * @return returns true if robot is not in collision when center of the robot is on the point, false otherwise
     */
    bool getPointCost(const geometry_msgs::Point &position, const std::vector<geometry_msgs::Point> &oriented_footprint, double &cost);

    /**
     * @brief Check for collisions along a path and remove all points beyond the first detected collision point
     * @param path The path to be checked
     * @param path_frame_id The frame in which planning and control is being done and the input path is specified
     * @param return_path The checked path to be returned
     * @param costmap_frame_id The frame in which the map is specified
     */
    void checkPath(const std::vector<geometry_msgs::Pose2D> &path, const std::string &path_frame_id, std::vector<geometry_msgs::Pose2D> &return_path, std::string &costmap_frame_id);

    /**
     * @brief Transform a path from one frame to another frame
     * @param path_in The path to be transformed
     * @param frame_in The frame in which the input path is specified
     * @param path_out The transformed path
     * @param frame_in The frame in which the output path is specified
     */
    void transformPath(const std::vector<geometry_msgs::Pose2D> &path_in, const std::string &frame_in, std::vector<geometry_msgs::Pose2D> &path_out, const std::string &frame_out);

    /**
     * @brief Transform a 2D point from one frame to another
     * @param point_in The point to be transformed
     * @param frame_in The frame in which the input point is specified
     * @param path_out The transformed point
     * @param frame_in The frame in which the output point is specified
     */
    void transform2DPose(const geometry_msgs::Pose2D &point_in, const std::string original_frame_id, geometry_msgs::Pose2D &point_out, const std::string &transform_frame_id);

    double delta_angle_;

    double distance_to_goal_;

    double centerline_distance_;

    int plan_length_;

    geometry_msgs::Pose2D current_position_;
    
  };
}

