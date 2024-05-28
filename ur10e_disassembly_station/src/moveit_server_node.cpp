#include "ur10e_disassembly_station/MoveItService.h"

int main(int argc, char** argv)
{
  /* ---------------------------------------------------------------
   * 1. Initialize ros node
   * --------------------------------------------------------------- */
  ros::init(argc, argv, "ur10e_moveit_server_node");
  std::string node_name = ros::this_node::getName();
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  /* ---------------------------------------------------------------
   * 2. Read in ros parameters
   * --------------------------------------------------------------- */
  int node_frequency;
  std::string planner;
  priv_nh.getParam("planner", planner);
  priv_nh.getParam("frequency", node_frequency);


  ROS_WARN(
    "\033[32m (%s): \n=============================================================================\n  "
    "STARTED MOVEIT SERVER "
    "FOR PLANNING REQUESTS OF UR10E ROBOT"
    "\n=============================================================================\n "
    "USING PLANNER: %s, \n NODE FREQUENCY: %d \033[0m",
    node_name.c_str(),
    planner.c_str(),
    node_frequency);

  /* ---------------------------------------------------------------
   * 3. Initialize MoveIt service server
   * --------------------------------------------------------------- */
  MoveItService moveit(&nh, &priv_nh);

  /* ---------------------------------------------------------------
   * 4. Start handling incoming requests in a constant rate
   * --------------------------------------------------------------- */
  ros::Rate loop_rate(node_frequency);
  while (ros::ok())
  {
    //--------------------------------------------------------------------
    // ROS synchro
    //--------------------------------------------------------------------
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO_STREAM("SHUTED DOWN " << node_name << " BY USER.";);
  return 0;
}