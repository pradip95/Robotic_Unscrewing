#pragma once

#include "control_lib/UsefullFunctions.h"

#include "control_lib/String_cmd.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

// ROS
#include <geometry_msgs/WrenchStamped.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
// Moveit
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// #include <moveit/robot_commander/robot_commander.h>

#include <boost/scoped_ptr.hpp>

// ur_rtde
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
using namespace ur_rtde;


enum Forcetype
{
  WRENCH_UP,
  WRENCH_DOWN,
  GETTOOLIN,
  GETTOOLOUT,
  EJECTTOOLIN,
  EJECTTOOLOUT
};
enum Goal
{
  HOME,
  CLAMPDEVICE,
  TOOLCHANGER
};
enum Slot
{
  SLOT_1,
  SLOT_2,
  SLOT_3
};

class RobotControl
{
private:
  const double PI = 3.141592;

  const std::string IP_ADRESS_ROBI = "172.22.132.15";

  // endeffector configurations
  double payload_mass_              = 5.160;                   // 4.060;
  std::vector<double> payload_CG_   = {-0.1, -0.007, 0.064}; // CG = Center of Gravity
  std::vector<double> tcp_location_ = {0.21303, -0.00161, 0.07744, 1.2082, -1.2343, 1.2280}; 

  // different positions for toolchange (the toolchange movement is L shaped)
  std::vector<double> top_L1_    = {-0.697976, 0.463191, 0.257999, -0.0624528, -2.21811, -2.18114};
  std::vector<double> bottom_L1_ = {-0.639855, 0.463264, 0.154337, -0.0629986, -2.21755, -2.18109};
  std::vector<double> top_L2_    = {-0.699322, 0.606547, 0.25411, -0.0663974, -2.21776, -2.18633};
  std::vector<double> bottom_L2_ = {-0.642678, 0.606694, 0.155602, -0.0673706, -2.2162, -2.18653};
  std::vector<double> top_L3_    = {-0.697613, 0.75327, 0.25327, -0.0830677, -2.206, -2.21078};
  std::vector<double> bottom_L3_ = {-0.64456, 0.753483, 0.155646, -0.0842987, -2.20411, -2.21124};

  // storage variables
  moveit_msgs::RobotTrajectory planned_plan;
  int number_of_points_plan_;

  // blend for gotoTP() method, if planned with moveit
  double path_blend_ = 0.03;

  // for extern FT-Sensor
  ros::AsyncSpinner* spinner_RobotControl_;
  ros::Subscriber sub_fts_;
  std::vector<double> TCP_force_sensor_ = {0, 0, 0, 0, 0, 0};
  void callbackFTS(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  // convert axis Angles to Quaternions
  std::vector<double> axisAngleToQuaternions(std::vector<double> axisAngle);
  // plans to go specified goal and stores it in planned_plan
  int moveitPlanTo(std::vector<double> goalTP);


public:
  // objects for ur_rtde
  RTDEControlInterface* rtde_control;
  RTDEReceiveInterface* rtde_receive;
  RTDEIOInterface* rtde_io;


  // constructor and destructor
  RobotControl();
  ~RobotControl();

  // hardcoded TCP positions for different locations
  std::vector<double> home_position_ = {-0.527512, 0.267283, 0.1789, 0.0972405, 2.20984, 2.21127};
  std::vector<double> clampdevice_position_ = {
    -0.139727,
    -0.530038,
    0.48,
    1.21277,
    -1.20451,
    -1.1598}; // 274.7, -66.34, -118.28. -175.35, -87.88, -89.68
  std::vector<double> toolchanger_position_ = {
    -0.30000, 0.611285, 0.50000, -0.0620725, -2.21022, -2.21479};
  // std::vector<double> toolchanger_position_ = {-0.36120, 0.58880, 0.20000, -0.0620725, -2.21022,
  // -2.21479}; // {-0.500547, 0.611285, 0.323413, -0.0620725, -2.21022, -2.21479}
  std::vector<double> toolchanger_position_3_ = {
    -0.30000, 0.75400, 0.50000, -0.0620725, -2.21022, -2.21479};
  std::vector<double> toolchanger_position_1_ = {
    -0.30000, 0.46150, 0.50000, -0.0620725, -2.21022, -2.21479};
  /**
   * @brief Calculates a point given in baseframe coordinates to the toolframe coordinates.
   * @param baseFramePosition 3D-Vector with x,y,z coordinates respective to baseframe.
   * @returns a 3D-Vector containing the x,y,z coordinates respective to the toolframe.
   */
  std::vector<double> TransformBaseToToolFrame(std::vector<double> baseFramePosition);

  /**
   * @brief Moves robot to defined goal.
   * @param goal HOME, CLAMPDEVICE or TOOLCHANGER.
   * @param useMoveit true = use moveit to calculate trajectory, false = linear movement, without
   * collision detection but more accuracy!!!
   * @param velocity Velocity of TCP.
   * @param acceleration Acceleration of TCP.
   */
  void gotoTP(Goal goal, bool useMoveit, double velocity, double acceleration);

  /**
   * @brief Moves robot to defined goal.
   * @param goal 6D-Vecotor with position in x,y,z coordinates and orientation in axis angles, both
   * respective to base frame.
   * @param useMoveit true = use moveit to calculate trajectory, false = linear movement, without
   * collision detection but more accuracy!!!
   * @param velocity Velocity of TCP.
   * @param acceleration Acceleration of TCP.
   */
  bool gotoTP(std::vector<double> goal, bool useMoveit, double velocity, double acceleration);

  /**
   * @brief Calculates the distance between the x,z plane of the toolframe and a given point.
   * @param relativeTCPPose 6D-Vecotor with position in x,y,z coordinates and orientation in axis
   * angles, both respective to base frame.
   * @returns a double indicating the depth/distance.
   */
  double calcDepth(std::vector<double> relativeTCPPose);

  // for extern FT-Sensor
  void nullFTS();
  std::vector<double> getTCPForce();

  /**
   * @brief Performes predefined movements in forcemode.
   * @param type WRENCH_UP, WRENCH_DOWN, GETTOOLIN, GETTOOLOUT, EJECTTOOLIN, EJECTTOOLOUT.
   * @param strength strenth in Nm.
   * @returns just a bool ... no reason.
   */
  bool startForcemode(Forcetype type, double strength);

  /**
   * @brief Meassures TCP Force respective to the tool frame.
   * @returns 6D-Vector with force and wrench.
   */
  std::vector<double> getToolFrameForce();

  /**
   * @brief get a tool
   * @param fromSlot target slot to get the tool from
   */
  void getTool(int fromSlot);

  /**
   * @brief release a tool
   * @param toSlot slot to place the tool
   */
  void releaseTool(Slot toSlot);

  /**
   * @brief Performs toolchange.
   * @param fromSlot free slot to put current tool in.
   * @param toSlot slot to get tool from
   */
  void changeTool(Slot fromSlot, Slot toSlot);

  /**
   * @brief Puts robot into freedrive mode, after pressing a key it will reset to normal mode.
   * @param repeatHowMany number of times to execute procedure: forcemode -> back to normal mode
   * after key was pressed.
   */
  void freeDrive(int repeatHowMany);

  void setVelocity(double velocity);
  void setAcceleration(double acceleration);

  void moveTo(Goal, bool);
};