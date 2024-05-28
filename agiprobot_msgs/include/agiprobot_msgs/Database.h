/**
 * @file 
 * @brief Define all shared data structure for the projects
 * @author yuhanjin89@gmail.com
 * 
 */
 
#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <ros/ros.h>
#include <unordered_map>
#include <urdf_model/pose.h>
#include <vector>


/* ---------------------------------------------------------------
 *  global enum classes
 * --------------------------------------------------------------- */
/**
 * @brief fsm state names
 *
 */
enum class STATE : int8_t
{
  // system states
  STANDBY,
  CALIBRATE,
  INSPECT,
  RECYCLE,
  PICK_TOOL,
  DROP_TOOL,
  // manipulation states
  SERVO,
  UNSCREW,
  GRASP,
  DROP_SCREW,
  SAFE,
  UNKNOWN
};

enum class TOOL : int8_t
{
  NO_TOOL,
  TOOL1,
  TOOL2,
  TOOL3
};

enum class GRIPPER : int8_t
{
  UP,
  DOWN,
  ERROR,
  SLEEP
};

/**
 * @brief screw type names
 *
 */
enum class SCREW_TYPE : int8_t
{
  PHILLIPS,
  TORX,
  POZIDRIV,
  HEX_WASHER,
  HEXAGONAL_BOLT,
  FLAT_HEAD,
  UNKNOWN,
  // markers for debugging
  RED_MARKER,
  GREEN_MARKER,
  BLUE_MARKER
};

enum class HEADING : int8_t
{
  FRONT,
  BACK,
  LEFT,
  RIGHT
};

/* ---------------------------------------------------------------
 *  global structs
 * --------------------------------------------------------------- */
/**
 * @brief related informaition of a screw
 *
 */
struct Screw
{
  int x;
  int y;
  double depth;
  int radius;
  SCREW_TYPE type;
  TOOL tool;
  double unscrew_loops;
  double torque;
};

struct Screw_list
{
  std::vector<Screw> screws;
  std::vector<double> tcp_pose;
};

struct Screw_position_map
{
  std::map<int, Screw_list> screw_lists;
  double x_min;
  double x_max;
  double y_min;
  double y_max;
};

/**
 * @brief mapping of the xbox controller
 *
 */
struct JoyInput
{
  /* ---------------------------------------------------------------
   * axes array [8]
   * --------------------------------------------------------------- */
  double button_axes_y    = .0;
  double button_axes_x    = .0;
  double left_trigger     = 1.0;
  double left_joy_axes_y  = .0;
  double left_joy_axes_x  = .0;
  double right_trigger    = 1.0; // -1->max, 1->min
  double right_joy_axes_y = .0;
  double right_joy_axes_x = .0;
  /* ---------------------------------------------------------------
   * buttons array [11]
   * --------------------------------------------------------------- */
  double a_button         = .0;
  double b_button         = .0;
  double x_button         = .0;
  double y_button         = .0;
  double left_button      = .0;
  double right_button     = .0;
  double window           = .0;
  double menu             = .0;
  double xbox             = .0;
  double left_joy_button  = .0;
  double right_joy_button = .0;
};

struct KeyInput
{
  uint a           = 0;
  uint b           = 0;
  uint c           = 0;
  uint d           = 0;
  uint e           = 0;
  uint f           = 0;
  uint g           = 0;
  uint h           = 0;
  uint i           = 0;
  uint j           = 0;
  uint k           = 0;
  uint l           = 0;
  uint m           = 0;
  uint n           = 0;
  uint o           = 0;
  uint p           = 0;
  uint q           = 0;
  uint r           = 0;
  uint s           = 0;
  uint t           = 0;
  uint u           = 0;
  uint v           = 0;
  uint w           = 0;
  uint x           = 0;
  uint y           = 0;
  uint z           = 0;
  uint num_0       = 0;
  uint num_1       = 0;
  uint num_2       = 0;
  uint num_3       = 0;
  uint num_4       = 0;
  uint num_5       = 0;
  uint num_6       = 0;
  uint num_7       = 0;
  uint num_8       = 0;
  uint num_9       = 0;
  uint left_ctrl   = 0;
  uint space       = 0;
  uint enter       = 0;
  uint backspace   = 0;
  uint escape      = 0;
  uint home        = 0;
  uint arrow_up    = 0;
  uint arrow_down  = 0;
  uint arrow_left  = 0;
  uint arrow_right = 0;
};

/* ---------------------------------------------------------------
 * global typedefs
 * --------------------------------------------------------------- */
/**
 * @brief catrecsian space: x, y, z, row, pitch, yaw
 *                     in     meter, radians
 *        joint sapce: base, shoulder, elbow, wrist1, wrist2, wrist3
 *                     in     radians
 */
using TcpPose    = std::vector<double>; // 6D vector
using JointPose  = std::vector<double>; // 6D vector
using ToolframeForce = std::vector<double>; // 6D vector
using Position3D = std::vector<double>; // 3D vector
using AxisAngle  = std::vector<double>; // 3D vecotr
using Quat       = std::vector<double>; // 4D vector
using Yaw        = double;              // 1D element
using PoseArray  = std::vector<std::vector<double>>; // 6D vector array

/**
 * @brief m/s
 * 0 - 3
 */
using ViaSpeed = std::vector<double>;

/**
 * @brief m/s2
 * 0 - 150
 */
using ViaAcceleration = std::vector<double>;

/**
 * @brief store the waypoints, content can either be TCPPose, JointPose, Speed, Acceleration
 *
 */
using Waypoints = std::vector<std::vector<double> >;

/* ---------------------------------------------------------------
 *  global helper functions
 * --------------------------------------------------------------- */
/**
 * @brief stick 2 vector together
 *
 * @param position first vector
 * @param orientation second vector
 */
inline TcpPose createPose6D(Position3D position, AxisAngle axisAngle)
{
  position.insert(position.end(), axisAngle.begin(), axisAngle.end());
  return position;
}

/**
 * @brief transfer 3D axisAngle value to quaternion
 *
 * @param axisAngle  rx, ry, rz
 * @return Quat   x,y,z,w
 */
inline Quat axisAngleToQuaternions(AxisAngle axisAngle)
{
  double magnitude = sqrt(pow(axisAngle[0], 2) + pow(axisAngle[1], 2) + pow(axisAngle[2], 2));
  std::vector<double> normAxisAngle = {
    axisAngle[0] / magnitude, axisAngle[1] / magnitude, axisAngle[2] / magnitude};

  double angle = 0;
  if (axisAngle[0] != 0)
    angle = axisAngle[0] / normAxisAngle[0];
  else if (axisAngle[1] != 0)
    angle = axisAngle[1] / normAxisAngle[1];
  else if (axisAngle[2] != 0)
    angle = axisAngle[2] / normAxisAngle[2];

  Quat quaternion;
  if (angle)
  {
    quaternion.push_back(normAxisAngle[0] * sin(angle / 2));
    quaternion.push_back(normAxisAngle[1] * sin(angle / 2));
    quaternion.push_back(normAxisAngle[2] * sin(angle / 2));
    quaternion.push_back(cos(angle / 2));
  }
  return quaternion;
}

/**
 * @brief convert a quatarion to axis angle format
 *
 * @param quat input
 * @return AxisAngle output
 */
inline AxisAngle quatToAxisAngle(Quat quat)
{
  AxisAngle axis_angle;
  // todo
  return axis_angle;
}

/**
 * @brief comare rule defined for sorting the screw list
 *
 */
inline bool compareXY(const Screw& a, const Screw& b)
{
  if (a.y != b.y)
  {
    return a.y < b.y; // sort by x in ascending order
  }
  else
  {
    return a.x < b.x; // if x is equal, then sort by y in ascending order
  }
}

/**
 * @brief sort input screw list
 *
 * @param screws input screw list
 */
inline void sortScrews(std::vector<Screw>& screw_list)
{
  std::sort(screw_list.begin(), screw_list.end(), compareXY);
}

/* ---------------------------------------------------------------
 *  Define data structures in Database
 * --------------------------------------------------------------- */
class Database
{
public:
  /**
   * @brief Constructor
   *
   */
  Database();
  ~Database() = default;

  /* ---------------------------------------------------------------
   * functions
   * --------------------------------------------------------------- */
  /**
   * @brief record waypoints in cartesian space
   *
   * @param index
   * @param tcp_pose  current tcp pose
   * @return true
   * @return false
   */
  bool record_tcp_path(int index, TcpPose tcp_pose);

  /**
   * @brief record waypoints in joint space
   *
   * @param index
   * @param joint_pose current joint pose
   * @return true
   * @return false
   */
  bool record_joint_path(int index, TcpPose joint_pose);

  /**
   * @brief decide tool type needed for a screw input, write the tool type back into the checked screw object 
   * @param screw input screw
   * @return TOOL tool type
   */
  TOOL check_tool_type(Screw& screw);

  /**
   * @brief decide torque needed for a screw input, write it back to the input screw and return
   *
   * @param screw input screw
   * @return double torque value
   */
  double check_torque(Screw& screw);

  /**
   * @brief decide unscrew loops needed for a screw input, write it back to the input screw and
   * return
   *
   * @param screw input screw
   * @return double unscrew loop number
   */
  double check_unscrew_loops(Screw& screw);

  /**
   * @brief convert string classification into enum screw type
   *
   * @return SCREW_TYPE screw type enum
   */
  SCREW_TYPE convert_string_to_screw_type(std::string type);

  /**
   * @brief out put string of screw type of input screw
   *
   * @return std::string
   */
  std::string output_screw_type(Screw);

  /**
   * @brief convert string input into enum STATE
   *
   * @param state string input
   * @return STATE
   */
  STATE convert_string_to_state(std::string state);

  /* ---------------------------------------------------------------
   * variables
   * --------------------------------------------------------------- */
  /**
   * @brief the distance from camera to drill tip
   *
   */
  static const double tcp_to_camera_dist;

  /**
   * @brief screw database
   *
   * read-only
   */
  static const std::unordered_map<SCREW_TYPE, Screw> screw_meta;

  /**
   * @brief cartesian space pose database
   *
   * read-only
   */
  static const std::unordered_map<STATE, TcpPose> fixed_tcp_poses;

  /**
   * @brief cartesian space orientation database
   *
   * read-only
   */
  static const std::unordered_map<HEADING, AxisAngle> fixed_tcp_headings;

  /**
   * @brief joint space pose database
   *
   * read-only
   */
  static const std::unordered_map<STATE, JointPose> fixed_joint_poses;

  /**
   * @brief cartesian space path database
   *
   * read-only
   */
  static const std::unordered_map<std::string, Waypoints> fixed_tcp_paths;

  /**
   * @brief joint space path database
   *
   * read-only
   */
  static const std::unordered_map<std::string, Waypoints> fixed_joint_paths;


  /**
   * @brief cartesian path stored at runtime
   *
   * read-write
   */
  std::unordered_map<int, Waypoints> temporary_tcp_paths;

  /**
   * @brief joint path stored at runtime
   *
   * read-write
   */
  std::unordered_map<int, Waypoints> temporary_joint_paths;
};
