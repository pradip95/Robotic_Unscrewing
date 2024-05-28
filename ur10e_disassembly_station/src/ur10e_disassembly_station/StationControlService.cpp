#include "ur10e_disassembly_station/StationControlService.h"

StationControlService::StationControlService(ros::NodeHandle* nh, ros::NodeHandle* priv_nh)
  : m_nh(*nh)
  , m_priv_nh(*priv_nh)
  , m_node_name(ros::this_node::getName())
  , m_joint_state_pub(m_nh.advertise<sensor_msgs::JointState>("/joint_states", 1))
  , m_taskInterface(new TaskInterface())
  , m_fsm(FSM())
  , m_db(Database())
  , m_current_tool(TOOL::NO_TOOL)
  , m_tcp_fail_count(0)
{
  /* ---------------------------------------------------------------
   * Register publisher, subscriber and services
   * --------------------------------------------------------------- */
  // clang-format off
  m_pick_tool_service =     m_nh.advertiseService("pickTool", &StationControlService::servicePickTool, this);
  m_drop_tool_service =     m_nh.advertiseService("dropTool", &StationControlService::serviceDropTool, this);
  m_moveit_to_service =       m_nh.advertiseService("moveItTo", &StationControlService::serviceMoveItTo, this);
  m_unscrew_service =       m_nh.advertiseService("unscrew", &StationControlService::serviceUnscrew, this);
  m_pushTCPPose_service = m_nh.advertiseService("pushTCPPose", &StationControlService::servicePushTCPPose, this);
  m_fetchTCPPose_service =   m_nh.advertiseService("fetchTCPPose", &StationControlService::serviceFetchTCPPose, this);
  m_fetchToolframeForce_service = m_nh.advertiseService("fetchToolframeForce", &StationControlService::serviceFetchToolframeForce, this);
  m_teachMode_service =     m_nh.advertiseService("teachMode", &StationControlService::serviceTeachMode, this);
  m_screw_alignment_service = m_nh.advertiseService("screw_alignment", &StationControlService::serviceScrewAlignment, this);

  m_plan_to_client =        m_nh.serviceClient<agiprobot_msgs::moveit>("planTo"); 
  m_trigger_fsm_to =        m_nh.serviceClient<agiprobot_msgs::fsm>("trigger_fsm_to");
   
  m_planned_path_sub =      m_nh.subscribe("move_group/result", 1, &StationControlService::planResultCallback, this);
  m_fsm_state_sub =         m_nh.subscribe("FSM_state", 5, &StationControlService::fsmStateCallback, this );
  // clang-format on
  /* ---------------------------------------------------------------
   * Retrieve ros parameters defined in launch file
   * --------------------------------------------------------------- */
  int start_tool;
  m_priv_nh.getParam("starting_tool", start_tool);
  m_priv_nh.getParam("speed", m_speed);
  m_priv_nh.getParam("acceleration", m_acceleration);
  m_priv_nh.getParam("path_blend_rate", m_path_blend_rate);

  //-------------------------------------------------------------------------------------
  // Set starting tool states
  //-------------------------------------------------------------------------------------
  switch (start_tool)
  {
    case 0:
      m_current_tool = TOOL::NO_TOOL;
      break;
    case 1:
      m_current_tool = TOOL::TOOL1;
      break;
    case 2:
      m_current_tool = TOOL::TOOL2;
      break;
    case 3:
      m_current_tool = TOOL::TOOL3;
      break;
    default:
      ROS_ERROR_STREAM(m_node_name << ": There is no valid tool given, please check launch file.");
      break;
  }

  //-------------------------------------------------------------------------------------
  // Communicate with the ur_rtde to setup intial robot state stored in object
  //-------------------------------------------------------------------------------------
  fetch_robot_state();
}

StationControlService::~StationControlService()
{
  // destroy taskInterface, avoid dangling pointer
  delete m_taskInterface;
  m_taskInterface = nullptr;
}

//-------------------------------------------------------------------------------------
// setters
//-------------------------------------------------------------------------------------
void StationControlService::set_current_tool(TOOL tool)
{
  m_current_tool = tool;
}

void StationControlService::set_speed(double speed)
{
  if (speed >= 0 && speed < 3)
    m_speed = speed;
}

void StationControlService::set_acceleration(double acceleration)
{
  if (acceleration >= 0 && acceleration < 150)
    m_acceleration = acceleration;
}

//-------------------------------------------------------------------------------------
// getters
//-------------------------------------------------------------------------------------
TcpPose StationControlService::get_tcp_pose()
{
  return m_tcp_pose;
}

TcpPose StationControlService::get_toolframe_force() {
  
  return m_toolframe_force;
}

PoseArray StationControlService::get_spiral_path()
{
  return m_spiral_path;
}
//-------------------------------------------------------------------------------------
// helpers
//-------------------------------------------------------------------------------------
bool StationControlService::fetch_robot_state()
{
  //-------------------------------------------------------------------------------------
  // communicate with ur_rtde
  //-------------------------------------------------------------------------------------
  m_tcp_pose       = m_taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
  m_joint_pose     = m_taskInterface->Robot_Task->rtde_receive->getActualQ();
  m_joint_velocity = m_taskInterface->Robot_Task->rtde_receive->getActualQd();

  ROS_INFO_STREAM(m_node_name <<"Robot joint state: " << m_joint_pose[0] << " " << m_joint_pose[1] << " "
                                  << m_joint_pose[2] << " " << m_joint_pose[3] << " "
                                  << m_joint_pose[4] << " " << m_joint_pose[5]);
  //-------------------------------------------------------------------------------------
  // tell moveIt the robot joint state
  //-------------------------------------------------------------------------------------
  publish_joint_state();

  return !m_tcp_pose.empty() && !m_joint_pose.empty();
}

bool StationControlService::push_tcp_pose()
{
  //-------------------------------------------------------------------------------------
  // send target tcp pose to ur_rtde
  //-------------------------------------------------------------------------------------
  bool success =
    m_taskInterface->Robot_Task->rtde_control->moveL(m_tcp_pose, m_speed, m_acceleration);

  //-------------------------------------------------------------------------------------
  // read current robot state
  //-------------------------------------------------------------------------------------
  fetch_robot_state();
  return success;
}

void StationControlService::publish_joint_state()
{
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();
  joint_state_msg.name.resize(6);
  joint_state_msg.position.resize(6);
  joint_state_msg.velocity.resize(6);
  joint_state_msg.effort.resize(6);

  joint_state_msg.header.stamp = ros::Time::now();
  joint_state_msg.name         = {"shoulder_pan_joint",
                                  "shoulder_lift_joint",
                                  "elbow_joint",
                                  "wrist_1_joint",
                                  "wrist_2_joint",
                                  "wrist_3_joint"};
  joint_state_msg.position     = m_joint_pose;
  joint_state_msg.velocity     = m_joint_velocity;
  for (int i = 0; i < joint_state_msg.effort.size(); ++i)
    joint_state_msg.effort[i] = 0.0; // because we don't use MoveIt controller, we don't need effort info
  m_joint_state_pub.publish(joint_state_msg);
}

//-------------------------------------------------------------------------------------
// ros service handlers
//-------------------------------------------------------------------------------------

bool StationControlService::serviceMoveItTo(agiprobot_msgs::goal::Request& req,
                                            agiprobot_msgs::goal::Response& res)
{
  //-------------------------------------------------------------------------------------
  // get from ur_rtde the actual robot joint state as planning context of MoveIt
  //-------------------------------------------------------------------------------------
  fetch_robot_state();

  //-------------------------------------------------------------------------------------
  // convert goal point to joint space target
  //-------------------------------------------------------------------------------------
  STATE goal           = static_cast<STATE>(req.goal);
  JointPose joint_goal = m_db.fixed_joint_poses.at(goal);

  //-------------------------------------------------------------------------------------
  // call up moveit to plan trajectory and move to joint target
  //-------------------------------------------------------------------------------------
  agiprobot_msgs::moveit moveit_msg;
  moveit_msg.request.joint_pose = joint_goal;
  m_plan_to_client.call(moveit_msg);
  return moveit_msg.response.success;
}

bool StationControlService::servicePickTool(agiprobot_msgs::tool::Request& req,
                                            agiprobot_msgs::tool::Response& res)
{
  //-------------------------------------------------------------------------------------
  // Start picking tool
  //-------------------------------------------------------------------------------------
  if (static_cast<TOOL>(req.fromSlot) == TOOL::TOOL1)
  {
    ROS_WARN_STREAM(m_node_name << " : Start picking tool 1");
    //-------------------------------------------------------------------------------------
    // Execute path with position controller
    //-------------------------------------------------------------------------------------
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool1")[0]; // starting point
    push_tcp_pose();
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool1")[1]; // left side above
    push_tcp_pose();
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool1")[2]; // aligned above
    push_tcp_pose();
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool1")[3]; // aligned above
    push_tcp_pose();
    //-------------------------------------------------------------------------------------
    // Force controller, since tcp will interact with the bracket
    //-------------------------------------------------------------------------------------
    ROS_INFO_STREAM(m_node_name << " Interacting with bracket in force mode.");
    m_taskInterface->Robot_Task->startForcemode(GETTOOLIN, 25);
    m_taskInterface->Robot_Task->startForcemode(GETTOOLOUT, 50);
    m_taskInterface->Robot_Task->startForcemode(GETTOOLIN, 0);
 
    // push_tcp_pose();
    //-------------------------------------------------------------------------------------
    // Exit process, store the current tool in hand
    //-------------------------------------------------------------------------------------
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool1")[4]; // aligned above
    push_tcp_pose();
    m_current_tool = TOOL::TOOL1;
   
    //-------------------------------------------------------------------------------------
    // Back to starting pose
    //-------------------------------------------------------------------------------------
    m_tcp_pose = m_db.fixed_tcp_poses.at(STATE::PICK_TOOL);
    push_tcp_pose();

    res.success = true;
 
  }
  else if (static_cast<TOOL>(req.fromSlot) == TOOL::TOOL3)
  {
    ROS_WARN_STREAM(m_node_name << " : Start picking tool 3");
    //-------------------------------------------------------------------------------------
    // Execute path with position controller
    //-------------------------------------------------------------------------------------
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool3")[0]; // starting point
    push_tcp_pose();
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool3")[1]; // left side above
    push_tcp_pose();
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool3")[2]; // aligned above
    push_tcp_pose();
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool3")[3]; // aligned above
    push_tcp_pose();
    //-------------------------------------------------------------------------------------
    // Start force controller, since tcp will interact with the bracket
    //-------------------------------------------------------------------------------------
    ROS_INFO_STREAM(m_node_name << " Interacting with bracket in force mode.");
    m_taskInterface->Robot_Task->startForcemode(GETTOOLIN, 25);
    m_taskInterface->Robot_Task->startForcemode(GETTOOLOUT, 50);
    //-------------------------------------------------------------------------------------
    // Exit process, set current tool index
    //-------------------------------------------------------------------------------------
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool3")[4]; // aligned above
    push_tcp_pose();
    m_current_tool = TOOL::TOOL3;
    //-------------------------------------------------------------------------------------
    // Back to starting pose
    //-------------------------------------------------------------------------------------
    m_tcp_pose = m_db.fixed_tcp_poses.at(STATE::PICK_TOOL);
    push_tcp_pose();

    res.success = true;
  }
  else
  {
    ROS_ERROR_STREAM(m_node_name << " : Requesting for a not availiabe tool bracket, skipped.");
    res.success = false;
  }
  return res.success;
}


bool StationControlService::serviceDropTool(agiprobot_msgs::tool::Request& req,
                                            agiprobot_msgs::tool::Response& res)
{
  if (static_cast<TOOL>(req.fromSlot) == TOOL::TOOL1)
  {
    ROS_WARN_STREAM(m_node_name << " : Start dropping tool 1");
    //-------------------------------------------------------------------------------------
    // Execute path with position controller
    //-------------------------------------------------------------------------------------
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool1")[4]; // TODO: measure again
    push_tcp_pose();
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool1")[3]; // TODO: measure again
    m_tcp_pose[0] -= 0.003;
    m_tcp_pose[1] += 0.002;
    push_tcp_pose();
     //-------------------------------------------------------------------------------------
    // Start force controller, since tcp will interact with the bracket
    //-------------------------------------------------------------------------------------
    int force = 20;
    ROS_INFO_STREAM(m_node_name << " Interacting with bracket using force: " << force);
    m_taskInterface->Robot_Task->startForcemode(EJECTTOOLIN, force);
    force = 200;
    ROS_INFO_STREAM(m_node_name << " Interacting with bracket using force: " << force);
    m_taskInterface->Robot_Task->startForcemode(EJECTTOOLOUT, force);
    //m_taskInterface->Robot_Task->startForcemode(GETTOOLIN, 0); //optional forcemode with zero force to make the program counter return
    //-------------------------------------------------------------------------------------
    // Position controller again
    //-------------------------------------------------------------------------------------
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool1")[2]; // aligned above
    push_tcp_pose();
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool1")[1]; // left side above
    push_tcp_pose();
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool1")[0]; // get out
    push_tcp_pose();
    //-------------------------------------------------------------------------------------
    // Exit process, back to staring point, set current tool index
    //-------------------------------------------------------------------------------------
    m_tcp_pose = m_db.fixed_tcp_poses.at(STATE::DROP_TOOL);
    push_tcp_pose();
    m_current_tool = TOOL::NO_TOOL;
    
    res.success    = true;
  }
  else if (static_cast<TOOL>(req.fromSlot) == TOOL::TOOL3)
  {
    ROS_WARN_STREAM(m_node_name << " : Start dropping tool 3");
    //-------------------------------------------------------------------------------------
    // Execute path with position controller
    //-------------------------------------------------------------------------------------
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool3")[4]; // TODO: measure again
    push_tcp_pose();
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool3")[3]; // TODO: measure again
    m_tcp_pose[0] += 0.005;
    m_tcp_pose[1] += 0.002;
    push_tcp_pose();
    //-------------------------------------------------------------------------------------
    // Start force controller, since tcp will interact with the bracket
    //-------------------------------------------------------------------------------------
    int force = 20;
    ROS_INFO_STREAM(m_node_name << " Interacting with bracket using force: " << force);
    m_taskInterface->Robot_Task->startForcemode(EJECTTOOLIN, force);
    force = 200;
    ROS_INFO_STREAM(m_node_name << " Interacting with bracket using force: " << force);
    m_taskInterface->Robot_Task->startForcemode(EJECTTOOLOUT, force);
    m_taskInterface->Robot_Task->startForcemode(GETTOOLIN, 0);
    //-------------------------------------------------------------------------------------
    // Position controller again
    //-------------------------------------------------------------------------------------
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool3")[2]; // aligned above
    push_tcp_pose();
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool3")[1]; // left side above
    push_tcp_pose();
    m_tcp_pose = m_db.fixed_tcp_paths.at("tool3")[0]; // get out
    push_tcp_pose();
    //-------------------------------------------------------------------------------------
    // Exit process, back to staring point, set current tool index
    //-------------------------------------------------------------------------------------
    m_tcp_pose = m_db.fixed_tcp_poses.at(STATE::DROP_TOOL);
    push_tcp_pose();
    m_current_tool = TOOL::NO_TOOL;
    res.success    = true;
  }
  else
  {
    ROS_ERROR_STREAM(m_node_name << " : Requesting for a not availiabe tool, skipped.");
    res.success = false;
  }
  return res.success;
}

bool StationControlService::serviceUnscrew(agiprobot_msgs::unscrew::Request& req,
                                           agiprobot_msgs::unscrew::Response& res)
{
  // check if the phiget board is ready
  if (!m_taskInterface->Drill_Task->isDrillReady())
  {
    ROS_ERROR_STREAM(m_node_name << ": drill is not ready, skipping this service..");
    res.sucess = false;
    return false;
  }

  ROS_INFO_STREAM(m_node_name << ": drill is ready, start unscrewing for screw type: "
                              << req.screwType);
  int loops = req.loops;

  for (int i = 0; i < loops; ++i)
  {
    m_taskInterface->unscrew();
  }

  fetch_robot_state();
  m_tcp_pose[2] += 0.001; // lift 1mm
  push_tcp_pose();

  res.sucess = true;
  return true;
}

bool StationControlService::serviceFetchTCPPose(agiprobot_msgs::tcp::Request& req,
                                                agiprobot_msgs::tcp::Response& res)
{
  fetch_robot_state();
  res.pose = m_tcp_pose;

  // error checking
  if (!res.pose.empty())
  {
    ROS_INFO_STREAM(m_node_name << ": success reading tcp pose.");
    res.success = true;
    return true;
  }
  else
  {
    ROS_ERROR_STREAM(m_node_name << ": failed reading tcp pose: ");
    for (auto i : res.pose)
    {
      ROS_ERROR_STREAM(i << " ");
    }
    res.success = false;
    return false;
  }
}

bool StationControlService::serviceFetchToolframeForce(agiprobot_msgs::toolframeForce::Request& req,
                                                agiprobot_msgs::toolframeForce::Response& res)
{
  m_taskInterface->Robot_Task->rtde_control->zeroFtSensor();
  m_toolframe_force = m_taskInterface->Robot_Task->getToolFrameForce();
  res.forceVector = m_toolframe_force;

  // error checking
  if (!res.forceVector.empty())
  {
    ROS_INFO_STREAM(m_node_name << ": success reading force vector.");
    res.success = true;
    return true;
  }
  else
  {
    ROS_ERROR_STREAM(m_node_name << ": failed reading force vector: ");
    for (auto i : res.forceVector)
    {
      ROS_ERROR_STREAM(i << " ");
    }
    res.success = false;
    return false;
  }
}

bool StationControlService::serviceScrewAlignment(agiprobot_msgs::align::Request& req,
                                                agiprobot_msgs::align::Response& res)
{
  m_taskInterface->setGeneralVariables(0.004, 7, 4, 0.0015, 0.005);
  //m_taskInterface->sp_search_.testDrill(m_taskInterface);
  //m_taskInterface->sp_search_.executeWithLateralControl(0.0005, m_taskInterface);
  //m_taskInterface->sp_search_.testScrewAlignment_hex(0.0007, m_taskInterface);
  //double unscrew_service_check = m_taskInterface->sp_search_.testScrewAlignment_torx20_withSensor(0.0006, m_taskInterface);
  //double unscrew_service_check = m_taskInterface->sp_search_.testScrewAlignment_torx25_withSensor(0.0006, m_taskInterface);
  double unscrew_service_check = m_taskInterface->sp_search_.testScrewAlignment_hex_withSensor(0.0006, m_taskInterface);

 // Define screw head points
  std::vector<std::vector<double>> screwHeadPoints = {
      {-0.19567, -0.52933, 0.330, 1.194, -1.257, -1.217},  // Original center screw1 near robot center
      {-0.18939, -0.46543, 0.330, 1.199, -1.251, -1.219}   // Original center screw2 away
      // Add more screw head points as needed
  };
  
  std::cout << "unscrew check: " << unscrew_service_check << std::endl;
  if (unscrew_service_check==1) {
    // Assuming the service was successful, set the response accordingly
    res.success = true;
    return true;
  }
  // If unscrew_service_check is not equal to 1, you may set the response accordingly
  res.success = false;
  return false; 
}

bool StationControlService::servicePushTCPPose(agiprobot_msgs::tcp::Request& req,
                                               agiprobot_msgs::tcp::Response& res)
{
  if (req.pose.size() != 6)
  { 
    //-------------------------------------------------------------------------------------
    // Reject when the data received in not a 6D vector
    //-------------------------------------------------------------------------------------
    res.success = false;
    ROS_ERROR_STREAM(m_node_name << ": got a wrong dimension of tcp update request: "
                                 << req.pose.size() << " neglected.");
    push_tcp_pose(); // push the last step tcp pose instead of the recieved one
  }
  else
  {
    //-------------------------------------------------------------------------------------
    // Push the requested tcp pose to robot
    //-------------------------------------------------------------------------------------
    m_tcp_pose  = req.pose;
    res.success = push_tcp_pose();
  }

  if (res.success)
  {
    //-------------------------------------------------------------------------------------
    // Update success
    //-------------------------------------------------------------------------------------
    ROS_WARN_STREAM(m_node_name << ": success pushing tcp pose, speed: " << m_speed
                                << ", acce: " << m_acceleration);
    m_tcp_fail_count = 0;
  }
  else
  {
    //-------------------------------------------------------------------------------------
    // If update keeps failing, respawn the node to rebuild communication
    //-------------------------------------------------------------------------------------
    ROS_ERROR_STREAM(m_node_name << ": pushing tcp pose failed, already " << m_tcp_fail_count++
                                 << " fail trails, going to respawn after 1000fails");
    if (m_tcp_fail_count > 1000)
    {
      ros::shutdown();
    }
  }

  res.pose = m_tcp_pose; // send back actual tcp pose
  return res.success;
}

bool StationControlService::serviceTeachMode(agiprobot_msgs::teachMode::Request& req,
                                             agiprobot_msgs::teachMode::Response& res)
{
  //-------------------------------------------------------------------------------------
  // switch on and off teach mode
  //-------------------------------------------------------------------------------------
  if (req.switchOn == true)
  {
    res.success = m_taskInterface->Robot_Task->rtde_control->teachMode();
    // press a key to exit teach mode
    char input;
    std::cout << "\nPress any letter to stop freedrive:\n";
    std::cin >> input;
    res.success = m_taskInterface->Robot_Task->rtde_control->endTeachMode();
  }
  else
  {
    res.success = m_taskInterface->Robot_Task->rtde_control->endTeachMode();

    // restart server
    ros::shutdown();
  }
  return res.success;
}

void StationControlService::planResultCallback(
  const moveit_msgs::MoveGroupActionResult::ConstPtr& result)
{
  ROS_WARN_STREAM(m_node_name << ": " << result->status.text);
  ROS_WARN_STREAM(m_node_name << " : planned a path by MoveIt with " << result->result.planning_time
                              << " seconds, starting execution.");
  //-------------------------------------------------------------------------------------
  // unwrap joint waypoints from moveit trajectory, feed them into ur_rtde joint path
  //-------------------------------------------------------------------------------------
  Waypoints rtde_joint_path;
  for (auto i : result->result.planned_trajectory.joint_trajectory.points)
  {
    // construct a waypoint
    JointPose joint_pose;
    for (auto j : i.positions) // joint positions
    {
      joint_pose.push_back(j);
    }

    joint_pose.push_back(m_speed);
    joint_pose.push_back(m_acceleration);
    joint_pose.push_back(m_path_blend_rate);
    // add to path
    rtde_joint_path.push_back(joint_pose);
  }

  //-------------------------------------------------------------------------------------
  // finally, call up ur_rtde to execute the path
  //-------------------------------------------------------------------------------------
  if (m_taskInterface->Robot_Task->rtde_control->moveJ(rtde_joint_path))
  {
    ROS_WARN_STREAM(m_node_name << " : Done execution.");
    fetch_robot_state();
  }
  else
    ROS_ERROR_STREAM(m_node_name << " : Path execution failed.");
  return;
}

void StationControlService::fsmStateCallback(const std_msgs::Int8ConstPtr& state)
{
  m_fsm.set_state(static_cast<STATE>(state->data));
  ROS_INFO_STREAM("FSM state heard at: " << m_node_name);
  m_fsm.print_current_state();
  return;
}
