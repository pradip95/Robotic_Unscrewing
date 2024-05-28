/**
 * @file 
 * @brief maintain MoveIt planing scene during station runtime, provide trajectory planning service
 * @author yuhanjin89@gmail.com
 * 
 */
#pragma once
// moveit lib
#include "geometry_msgs/Pose.h"
#include "moveit/kinematics_base/kinematics_base.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// ROS lib
#include "ros/callback_queue.h"
#include "ros/duration.h"
#include "ros/forwards.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/service_server.h"
#include "ros/spinner.h"
#include "ros/this_node.h"

// local lib
#include "agiprobot_msgs/moveit.h"

// std lib
#include <string>
#include <thread>
#include <vector>


class MoveItService
{
public:
  MoveItService(ros::NodeHandle* nh, ros::NodeHandle* priv_nh);
  ~MoveItService() = default;

private:
  //-------------------------------------------------------------------------------------
  // ros related variables
  //-------------------------------------------------------------------------------------
  ros::NodeHandle m_nh;
  ros::NodeHandle m_priv_nh;
  std::string m_node_name;
  ros::ServiceServer m_planTo_service;

  //-------------------------------------------------------------------------------------
  // callbacks
  //-------------------------------------------------------------------------------------
  bool servicePlanTo(agiprobot_msgs::moveit::Request& req, agiprobot_msgs::moveit::Response& res);

  //-------------------------------------------------------------------------------------
  // objects for MoveIt
  //-------------------------------------------------------------------------------------
  /**
   * @brief a move group interface wich talks with the move_group running on system
   *
   */
  moveit::planning_interface::MoveGroupInterface m_move_group;

  /**
   * @brief controls rviz visulization of robot
   *
   */
  moveit_visual_tools::MoveItVisualTools m_moveit_visual;

  /**
   * @brief store the planning scene for moveIt
   *
   */
  moveit::planning_interface::PlanningSceneInterface m_planning_scene;

  std::vector<moveit_msgs::CollisionObject> m_collision_objects;

  /**
   * @brief the planning algorithm used for motion planning
   * check MoveIt configuration for options e.g. RRTConnect
   */
  std::string m_planner_name;

};