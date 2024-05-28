/**
 * @file 
 * @brief provide services for manipulate the station actuators
 * @author yuhanjin89@gmail.com
 * 
 */

#pragma once
// ur_rtde lib
#include "control_lib/TaskInterface.h"

// agiprobot_msgs lib
#include "agiprobot_msgs/Database.h"
#include "agiprobot_msgs/goal.h"
#include "agiprobot_msgs/gotoTP.h"
#include "agiprobot_msgs/moveit.h"
#include "agiprobot_msgs/state.h"
#include "agiprobot_msgs/tcp.h"
#include "agiprobot_msgs/toolframeForce.h"
#include "agiprobot_msgs/teachMode.h"
#include "agiprobot_msgs/tool.h"
#include "agiprobot_msgs/unscrew.h"
#include "agiprobot_msgs/fsm.h"
#include "agiprobot_msgs/align.h"


// ROS lib
#include "ros/init.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include "ros/duration.h"

// local lib
#include "std_msgs/Int8.h"
#include "ur10e_disassembly_station/FSM.h"

// moveit lib
#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include <moveit_msgs/MoveGroupActionResult.h>

// std lib
#include <string>
#include <urdf_model/joint.h>
#include <unistd.h>

class StationControlService
{
public:
  /* ---------------------------------------------------------------
   * C'tor and D'tor
   * --------------------------------------------------------------- */
  StationControlService(ros::NodeHandle* nh, ros::NodeHandle* priv_nh);
  ~StationControlService();


  /* ---------------------------------------------------------------
   * tool functions to be called
   * --------------------------------------------------------------- */
  // setters
  void set_current_tool(TOOL tool);
  void set_speed(double speed);
  void set_acceleration(double acc);

  // getters
  /**
   * @brief get the actual tcp pose stored in this object
   *
   * @return tcp pose vector
   */
  TcpPose get_tcp_pose();

  /**
   * @brief get the actual toolframe force stored in this object
   *
   * @return toolframe force vector
   */
  TcpPose get_toolframe_force();

  /**
   * @brief get spiral trajectory
   *
   * @return array of spiral points in spiral path
   */
  PoseArray get_spiral_path();

  // communication with ur_rtde
  /**
   * @brief retrive from rtde_receive the current robot state
   *
   * @return true
   * @return false
   */
  bool fetch_robot_state();

  /**
   * @brief make robot move to target tcp pose using precise control
   *
   * @return true
   * @return false
   */
  bool push_tcp_pose();

  /**
   * @brief publish actual joint state of robot
   * make MoveIt knows as plannning context
   */
  void publish_joint_state();

private:
  //-------------------------------------------------------------------------------------
  // ROS handler
  //-------------------------------------------------------------------------------------
  // FSM
  ros::Subscriber m_fsm_state_sub;

  // rtde_receive
  ros::ServiceServer m_fetchTCPPose_service;
  ros::ServiceServer m_fetchToolframeForce_service;

  // rtde_control
  ros::ServiceServer m_pushTCPPose_service;
  ros::ServiceServer m_teachMode_service;
  ros::ServiceServer m_screw_alignment_service;

  // disassembly process
  ros::ServiceServer m_pick_tool_service;
  ros::ServiceServer m_drop_tool_service;
  ros::ServiceServer m_unscrew_service;

  // moveIt 
  ros::ServiceServer m_moveit_to_service;
  ros::ServiceClient m_plan_to_client;
  ros::Subscriber m_planned_path_sub;

  // visual-detecting

  //FSM interaction
  ros::ServiceClient m_trigger_fsm_to;

  //-------------------------------------------------------------------------------------
  // Callbacks
  //-------------------------------------------------------------------------------------
  // clang-format off
  
   /**
   * @brief change tool
   * 
   * @param req target tool
   * @param res current tool, success
   */
  bool serviceChangeTool(agiprobot_msgs::tool::Request& req, agiprobot_msgs::tool::Response& res);

  /**
   * @brief pick up target tool
   * 
   * @param req target tool
   * @param res current tool, success
   */
  bool servicePickTool(agiprobot_msgs::tool::Request& req, agiprobot_msgs::tool::Response& res);

  /**
   * @brief drop current tool to target bracket
   * 
   * @param req target bracket
   * @param res NO_TOOL, success
   */
  bool serviceDropTool(agiprobot_msgs::tool::Request& req, agiprobot_msgs::tool::Response& res);

  /**
   * @brief use MoveIt to plan path between current pose and target scenarios, then move robot to target scenario
   * 
   * @param req target scenario
   * @param res success
   */
  bool serviceMoveItTo(agiprobot_msgs::goal::Request& req, agiprobot_msgs::goal::Response& res);

  /**
   * @brief execute unscrew process
   * 
   * @param req screw type, unscrew loop number
   * @param res success 
   */
  bool serviceUnscrew(agiprobot_msgs::unscrew::Request& req, agiprobot_msgs::unscrew::Response& res);

  /**
   * @brief fetch current tcp pose from robot
   * 
   * @param res current tcp pose vector
   */
  bool serviceFetchTCPPose(agiprobot_msgs::tcp::Request& req, agiprobot_msgs::tcp::Response& res);

  /**
   * @brief fetch tool frame force from robot
   * 
   * @param res current tool frame force vector
   */
  bool serviceFetchToolframeForce(agiprobot_msgs::toolframeForce::Request& req, agiprobot_msgs::toolframeForce::Response& res);

  /**
   * @brief execute screw alignment process
   * 
   * @param res success
   */
  bool serviceScrewAlignment(agiprobot_msgs::align::Request& req, agiprobot_msgs::align::Response& res);

  /**
   * @brief send target tcp pose to robot, make robot move to target pose via precise control
   * 
   * @param req target tcp pose
   * @param res success
   */
  bool servicePushTCPPose(agiprobot_msgs::tcp::Request& req, agiprobot_msgs::tcp::Response& res);

  /**
   * @brief start teach mode, press any key on keyboard to stop 
   * 
   */
  bool serviceTeachMode(agiprobot_msgs::teachMode::Request& req, agiprobot_msgs::teachMode::Response& res);
  // clang-format on

  /**
   * @brief once a MoveIt planned path result comes, handle it and pass to robot.
   *
   * @param result planning result
   */
  void planResultCallback(const moveit_msgs::MoveGroupActionResult::ConstPtr& result);

  /**
   * @brief once a FSM state is published, sync the local FSM state to be identical to that
   * 
   * @param state FSM state, use static_cast<STATE>(state->data) when receiving
   */
  void fsmStateCallback(const std_msgs::Int8ConstPtr& state);

  //-------------------------------------------------------------------------------------
  // private varaibles
  //-------------------------------------------------------------------------------------

  // ROS related
  ros::NodeHandle m_nh;
  ros::NodeHandle m_priv_nh;
  std::string m_node_name;
  ros::Publisher m_joint_state_pub;

  /**
   * @brief interface to ur_rtde and phidget board
   *
   */
  TaskInterface* m_taskInterface;

  /**
   * @brief currently mounted tool type
   *
   */
  TOOL m_current_tool;

  /**
   * @brief robot execution speed
   *
   */
  double m_speed;

  /**
   * @brief robot execution acceleration
   *
   */
  double m_acceleration;

  /**
   * @brief 6D vector of actual tcp pose
   * x, y, z, rx, ry, rz
   * @brief 6D vector of actual Tool force
   */
  TcpPose m_tcp_pose, m_toolframe_force;

  /**
   * @brief array of 6D vector of tcp pose
   * x, y, z, rx, ry, rz
   */
  PoseArray m_spiral_path;
  /**
   * @brief 6D vector of actual Tool force
   */
  // ToolframeForce m_toolframe_force;

  /**
   * @brief 6D vector of actual joint pose
   * base, soulder, albow, wrist1, wrist2, wrist3
   */
  JointPose m_joint_pose;

  /**
   * @brief 6D vector of actual joint velocity
   * base, soulder, albow, wrist1, wrist2, wrist3
   */
  JointPose m_joint_velocity;

  /**
   * @brief internal FSM
   * to be synced with client
   */
  FSM m_fsm;

  /**
   * @brief give access to the database
   *
   */
  Database m_db;

  /**
   * @brief count how many times of fails of upateing robot tcp pose
   *
   */
  int m_tcp_fail_count;

  /**
   * @brief blend rate to be used when executing a path
   * a higher rate results a smoother path but lower accuracy
   */
  double m_path_blend_rate;
};