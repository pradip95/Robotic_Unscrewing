/**
 * @file 
 * @brief Central control node for the station, including most of data processing and algorithms execution 
 * @author yuhanjin89@gmail.com
 * 
 */
#pragma once
// self defined ros msgs/srvs
#include "agiprobot_msgs/BoundingBox.h"
#include "agiprobot_msgs/BoundingBoxes.h"
#include "agiprobot_msgs/Keyboard.h"
#include "agiprobot_msgs/detection.h"
#include "agiprobot_msgs/goal.h"
#include "agiprobot_msgs/gotoTP.h"
#include "agiprobot_msgs/tcp.h"
#include "agiprobot_msgs/teachMode.h"
#include "agiprobot_msgs/tool.h"
#include "agiprobot_msgs/unscrew.h"
#include "agiprobot_msgs/fsm.h"
#include "agiprobot_msgs/tool.h"
#include "agiprobot_msgs/align.h"

// ur_rtde lib
#include "control_lib/TaskInterface.h"

#include "screw_gripper/Execute.h"
#include "screw_gripper/Initiate.h"

// database
#include "agiprobot_msgs/Database.h"

// local header
#include "ur10e_disassembly_station/FSM.h"
#include "ur10e_disassembly_station/PIDController.h"

// ROS includes
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Trigger.h>

// cv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// c++ includes
#include <cmath>
#include <limits>
#include <string>
#include <vector>

//-------------------------------------------------------------------------------------
// ROS Callbacks
//-------------------------------------------------------------------------------------
void joyCommandCb(const sensor_msgs::Joy::ConstPtr& msg);
void keyCommandCb(const agiprobot_msgs::Keyboard::ConstPtr& msg);
void depthImageCb(const sensor_msgs::Image::ConstPtr& msg);
void rgbImageCb(const sensor_msgs::Image::ConstPtr& msg);
bool motorPlacedCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
bool fsmCb(agiprobot_msgs::fsm::Request& req, agiprobot_msgs::fsm::Response& res);
bool toolCb(agiprobot_msgs::tool::Request& req, agiprobot_msgs::tool::Response& res);


//-------------------------------------------------------------------------------------
// locall functions
//-------------------------------------------------------------------------------------
/**
 * @brief execute gripper once, can be pick or drop
 *
 */
bool execute_gripper();

/**
 * @brief fetch actual TCP pose from robot
 *
 */
bool fetch_tcp_pose();

/**
 * @brief push g_tcp_pose to robot, make robot TCP move to this point
 *
 */
bool push_tcp_pose();

/**
 * @brief lift TCP a certaub distance
 *
 * @param dist distance in meter, negative value means drop TCP
 */
bool lift_tcp(double dist);

/**
 * @brief do circular_inspect
 *
 * @param center_x circle center coordinate
 * @param center_y circle center coordinate
 * @param radius   circle radius
 * @param steps    move circle in how many steps
 */
void circular_inspect(double center_x, double center_y, double radius, int steps);

/**
 * @brief calls pick tool or drop tool service
 *
 */
void handle_tool_request();

/**
 * @brief handle and respond to gamepad input
 *
 */
void handle_joy_input();

/**
 * @brief handle and respond to keyboard input
 *
 */
void handle_key_input();

/**
 * @brief use moveIt to transit between scenario points with trajectroy planning
 *
 */
void moveit_to(STATE);

/**
 * @brief advertise the current FSM state system-wide
 *
 */
void publish_fsm_state();

/**
 * @brief trigger FSM state trasition, execute the target FSM behaviors, and publish the actual FSM
 * state system-wide
 *
 * @param target_state target FSM state
 */
void trigger_fsm_to(const STATE target_state);

/**
 * @brief show the realtime RGB and depth debug stream in 2 windows
 *
 */
void output_debug_image_stream();

/**
 * @brief detect printed red dot markers in RGB frame and add it into screw list
 *        (for debug/test purpose)
 *
 */
void detect_markers();

/**
* @brief set the heading direction of robot automatically based on different manipulation scene

*/
void auto_set_heading();

/**
 * @brief execute final tool-to-screw engagement based on last pixel distance and scaling factor
 */
bool engage_screw_head();

/**
 * @brief kills all the running nodes in ROS, call it ONLY when you want to shutdown the system
 * completely
 *
 */
void kill_all_nodes();
