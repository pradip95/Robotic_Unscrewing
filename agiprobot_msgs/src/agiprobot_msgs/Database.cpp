/**
 * @file 
 * @brief store all database contents 
 * @author yuhanjin89@gmail.com
 * 
 */
#include "agiprobot_msgs/Database.h"
/* ---------------------------------------------------------------
 * Database sources
 * --------------------------------------------------------------- */
/**
 * @brief tcp orientation databse
 *
 */
const std::unordered_map<HEADING, AxisAngle> Database::fixed_tcp_headings = {
  {HEADING::BACK, {0.0863348, 2.2045, 2.21301}}, {HEADING::RIGHT, {1.187, -1.233, -1.196}}};

/**
 * @brief tcp pose database
 *
 */
const std::unordered_map<STATE, TcpPose> Database::fixed_tcp_poses = {
  {STATE::CALIBRATE, {-0.17399, -0.76549, 0.46110, 1.187, -1.233, -1.196}},
  {STATE::RECYCLE, {-0.1286, -0.75387, 0.45250, 1.2245, -1.203, -1.1247}},
  {STATE::INSPECT, {-0.269, -0.4556, 0.3248, 1.187, -1.233, -1.196}},
  {STATE::STANDBY, {-0.556983, 0.0994184, 0.201043, 0.0863348, 2.2045, 2.21301}},
  {STATE::PICK_TOOL, {-0.460241, 0.513454, 0.218813, 0.0864398, 2.20445, 2.21333}},
  {STATE::DROP_TOOL, {-0.460241, 0.513454, 0.218813, 0.0864398, 2.20445, 2.21333}},
  {STATE::DROP_SCREW, {-0.15030, -0.83294, 0.56056, 1.333, -0.999, -0.974}},
  {STATE::SAFE, {-0.269, -0.4556, 0.3658, 1.187, -1.233, -1.196}},
};

/**
 * @brief joint pose database
 *
 */
const std::unordered_map<STATE, JointPose> Database::fixed_joint_poses = {
  {STATE::CALIBRATE, {1.5 * M_PI, -M_PI / 2, -M_PI / 2, M_PI, -M_PI / 2, -M_PI / 2}},
  {STATE::STANDBY, {3.38809, -1.25104, -2.30394, 3.54941, -1.28697, -1.57896}},
  {STATE::PICK_TOOL, {2.33539, -1.61848, -1.97222, 3.60171, -2.33669, -1.57661}},
  {STATE::DROP_TOOL, {2.33539, -1.61848, -1.97222, 3.60171, -2.33669, -1.57661}},
  {STATE::INSPECT, {4.40416, -1.179842, -2.11952, 3.298672, -1.87884, -1.57122}},
  {STATE::RECYCLE, {4.76766, -1.53188, -1.62708, 3.17529, -1.58587, -1.55135}}};


/**
 * @brief tcp path database
 *
 */
// clang-format off
const std::vector<double> tool1_1 = {-0.5057, 0.5374, 0.2085, 0.0863348, 2.2045, 2.21301}; // start
const std::vector<double> tool1_2 = {-0.7196, 0.5374, 0.2085, 0.0863348, 2.2045, 2.21301}; // left side above
const std::vector<double> tool1_3 = {-0.7196, 0.4908, 0.2085, 0.0863348, 2.2045, 2.21301}; // right above
const std::vector<double> tool1_4 = {-0.7196, 0.4908, 0.0961, 0.0863348, 2.2045, 2.21301}; // aligned
//const std::vector<double> tool1_5 = {-0.6159, 0.4908, 0.0961, 0.0863348, 2.2045, 2.21301}; // move away
const std::vector<double> tool1_5 = {-0.5057, 0.4908, 0.0961, 0.0863348, 2.2045, 2.21301}; // move away test
const Waypoints pick_tool1 = {tool1_1, tool1_2, tool1_3, tool1_4, tool1_5};
                        
                                   //{x,y,z, rotx, roty, rotz}
const std::vector<double> tool3_1 = {-0.5057, 0.7780, 0.2085, 0.0863348, 2.2045, 2.21301};  //start
const std::vector<double> tool3_2 = {-0.7215, 0.7780, 0.2085, 0.0863348, 2.2045, 2.21301};  //left side above
const std::vector<double> tool3_3 = {-0.7215, 0.7780, 0.2085, 0.0863348, 2.2045, 2.21301};  //right above 
const std::vector<double> tool3_4 = {-0.7215, 0.7780, 0.0961, 0.0863348, 2.2045, 2.21301};  //aligned
const std::vector<double> tool3_5 = {-0.5057, 0.7780, 0.0961, 0.0863348, 2.2045, 2.21301};  //move away

const Waypoints pick_tool3 = {tool3_1, tool3_2, tool3_3, tool3_4, tool3_5};

const std::unordered_map<std::string, Waypoints> Database::fixed_tcp_paths = {{"tool1", pick_tool1},{"tool3", pick_tool3}};

const double Database::tcp_to_camera_dist = 0.1955;
// clang-format on

/**
 * @brief Construct a new Database:: Database object
 *
 */
Database::Database()
  // : fixed_tcp_headings(TCP_ORIENTATION_DB)
  // , fixed_tcp_poses(TCP_POSE_DB)
  // , fixed_joint_poses(JOINT_POSE_DB)
  // , fixed_tcp_paths(TCP_PATH_DB)
  // , tcp_to_camera_dist(0.1955)
{
}

bool Database::record_tcp_path(int index, TcpPose pose)
{
  temporary_tcp_paths[index].push_back(pose);
  if (!pose.empty())
    return true;
  else
    return false;
};

bool Database::record_joint_path(int index, JointPose pose)
{
  temporary_joint_paths[index].push_back(pose);
  if (!pose.empty())
    return true;
  else
    return false;
};

TOOL Database::check_tool_type(Screw& screw)
{
  // add the mapping of tool station to specific screw at here...
  // example:
  switch (screw.type)
  {
    case SCREW_TYPE::HEXAGONAL_BOLT:
      screw.tool = TOOL::TOOL1;
      return TOOL::TOOL1;
    case SCREW_TYPE::TORX:
      screw.tool = TOOL::TOOL3;
      return TOOL::TOOL3;
    default:
      screw.tool = TOOL::NO_TOOL;
      return TOOL::NO_TOOL;
  }
}

double Database::check_unscrew_loops(Screw& screw)
{
  // add the mapping of the unscrew loops needed for weber controller based on screw type at here...
  // example:
  switch (screw.type) 
  {
    case SCREW_TYPE::HEXAGONAL_BOLT:
      screw.unscrew_loops = 1;
      return 1;
    case SCREW_TYPE::TORX:
      screw.unscrew_loops = 1;
      return 1;
    default:
      screw.unscrew_loops = 1;
      return 1;
  }
}

SCREW_TYPE Database::convert_string_to_screw_type(std::string type)
{
  if (type == "Philips Screw")
    return SCREW_TYPE::PHILLIPS;
  else if (type == "Torx Screw")
    return SCREW_TYPE::TORX;
  else if (type == "Pozidriv Screw")
    return SCREW_TYPE::POZIDRIV;
  else if (type == "Hex Washer Screw")
    return SCREW_TYPE::HEX_WASHER;
  else if (type == "Hexagonal Bolt")
    return SCREW_TYPE::HEXAGONAL_BOLT;
  else if (type == "Flat Head Screw")
    return SCREW_TYPE::FLAT_HEAD;
  else
  {
    ROS_ERROR_STREAM("found undefined screw class: " << type);
    return SCREW_TYPE::UNKNOWN;
  }
}

STATE Database::convert_string_to_state(std::string state)
{
  if (state == "standby")
    return STATE::STANDBY;
  else if (state == "calibrate")
    return STATE::CALIBRATE;
  else if (state == "inspect")
    return STATE::INSPECT;
  else if (state == "recycle")
    return STATE::RECYCLE;
  else if (state == "pick tool")
    return STATE::PICK_TOOL;
  else if (state == "drop tool")
    return STATE::DROP_TOOL;
  else if (state == "servo")
    return STATE::SERVO;
  else if (state == "unscrew")
    return STATE::UNSCREW;
  else if (state == "grasp")
    return STATE::GRASP;
  else
    return STATE::UNKNOWN;
}

std::string Database::output_screw_type(Screw screw)
{
  if (screw.type == SCREW_TYPE::HEX_WASHER)
    return "hex washer";
  if (screw.type == SCREW_TYPE::HEXAGONAL_BOLT)
    return "hexagonal bolt";
  if (screw.type == SCREW_TYPE::PHILLIPS)
    return "phillips";
  if (screw.type == SCREW_TYPE::POZIDRIV)
    return "pozidriv";
  if (screw.type == SCREW_TYPE::TORX)
    return "torx";
  if (screw.type == SCREW_TYPE::FLAT_HEAD)
    return "flat head";
  if (screw.type == SCREW_TYPE::UNKNOWN)
    return "unknown type";
  else
    return "undefined type";
}