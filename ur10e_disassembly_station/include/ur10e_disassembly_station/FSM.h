/**
 * @file 
 * @brief FSM which facilate state transition of the system
 * @author yuhanjin89@gmail.com
 * 
 */
#pragma once

#include "ros/ros.h"
#include <agiprobot_msgs/Database.h>

/* ---------------------------------------------------------------
  // FSM states
  STANDBY, //0
  INSPECT, //2
  SERVO,
  UNSCREW,
  GRASP,
  RECYCLE,  //3
  PICK_TOOL,
  DROP_TOOL,
  
  // other states
  CALIBRATE, //1
  UNKNOWN
* --------------------------------------------------------------- */

class FSM
{
public:
  FSM();
  ~FSM() = default;

  void set_state(STATE);
  STATE get_current_state();
  void print_current_state();

private:
  STATE m_current_state;
};
