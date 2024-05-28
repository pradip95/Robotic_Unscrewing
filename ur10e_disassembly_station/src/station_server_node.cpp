#include "ur10e_disassembly_station/StationControlService.h"

int main(int argc, char** argv)
{
  /* ---------------------------------------------------------------
   * 1. initialize ros node
   * --------------------------------------------------------------- */
  ros::init(argc, argv, "ur10e_server_node");
  std::string node_name = ros::this_node::getName();
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  /* ---------------------------------------------------------------
   * 2. read in ros parameters
   * --------------------------------------------------------------- */
  int current_tool;
  int node_frequency;
  double speed;
  double acceleration;
  double blend_rate;
  priv_nh.getParam("frequency", node_frequency);
  priv_nh.getParam("starting_tool", current_tool);
  priv_nh.getParam("speed", speed);
  priv_nh.getParam("acceleration", acceleration);
  priv_nh.getParam("path_blend_rate", blend_rate);

  ROS_WARN("\033[32m (%s): "
           "\n=============================================================================\n  "
           "STARTED STATION SERVER "
           "FOR EXECUTING THE UR10E ROBOT DISASSEMBLE STATION "
           "\n=============================================================================\n "
           "EXECUTION SPEED: %f, ACCE: %f, PATH BLEND RATE: %f \n EQUIPPED TOOL: %d, \033[0m",
           node_name.c_str(),
           acceleration,
           speed,
           blend_rate,
           current_tool);

  /* ---------------------------------------------------------------
   * 3. initialize station contol service server
   * --------------------------------------------------------------- */
  StationControlService station(&nh, &priv_nh);

  /* ---------------------------------------------------------------
   * 4. start handling incoming requests in a constant rate
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

  ROS_ERROR_STREAM("SHUTED DOWN " << node_name << " BY USER.");
  return 0;
}