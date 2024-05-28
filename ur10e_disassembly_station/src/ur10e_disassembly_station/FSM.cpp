#include "ur10e_disassembly_station/FSM.h"
/* ---------------------------------------------------------------
* CALIBRATE,
  STANDBY,
  INSPECT,
  SERVO,
  UNSCREW,
  RECYCLE,
  TOOL_CHANGE,
  TOOL1,
  TOOL2,
  TOOL3
* --------------------------------------------------------------- */
FSM::FSM()
  : m_current_state(STATE::STANDBY)
{
}

// getter
STATE FSM::get_current_state()
{
  return m_current_state;
}

// setter
void FSM::set_state(STATE state)
{
  m_current_state = state;
}

// printer
void FSM::print_current_state()
{
  switch (m_current_state)
  {
    case STATE::STANDBY:
      ROS_WARN("FSM state: STANDBY");
      break;
    case STATE::CALIBRATE:
      ROS_WARN("FSM state: CALIBRATE");
      break;
    case STATE::PICK_TOOL:
      ROS_WARN("FSM state: PICK_TOOL");
      break;
    case STATE::DROP_TOOL:
      ROS_WARN("FSM state: DROP_TOOL");
      break;
    case STATE::INSPECT:
      ROS_WARN("FSM state: INSPECT");
      break;
    case STATE::UNSCREW:
      ROS_WARN("FSM state: UNSCREW");
      break;
    case STATE::SERVO:
      ROS_WARN("FSM state: SERVO");
      break;
    case STATE::GRASP:
      ROS_WARN("FSM state: GRASP");
      break;
    case STATE::RECYCLE:
      ROS_WARN("FSM state: RECYCLE");
      break;
    default:
      ROS_WARN("FSM state: UNKNOWN");
      break;
  }
  return;
}