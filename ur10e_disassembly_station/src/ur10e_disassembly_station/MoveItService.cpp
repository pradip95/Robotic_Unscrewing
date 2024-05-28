#include "ur10e_disassembly_station/MoveItService.h"
#include "ros/init.h"

MoveItService::MoveItService(ros::NodeHandle* nh, ros::NodeHandle* priv_nh)
  : m_nh(*nh)
  , m_priv_nh(*priv_nh)
  , m_node_name(ros::this_node::getName())
  , m_move_group("ur10e")
  , m_moveit_visual("world")
  , m_planning_scene()
{
  m_nh.setParam("MOVEIT_ASYNC_PARAM", true);
  m_priv_nh.getParam("planner", m_planner_name);
  m_planTo_service = m_nh.advertiseService("planTo", &MoveItService::servicePlanTo, this);

  //-------------------------------------------------------------------------------------
  // Configure the MoveIt planner (optional)
  //-------------------------------------------------------------------------------------
  // m_move_group.setPlanningTime(0.1);
  // m_move_group.setNumPlanningAttempts(10);
  // m_move_group.setPlanningPipelineId("ompl");
  // // Set the tolerance for each joint of the robot's pose
  // m_move_group.setGoalJointTolerance(0.01);
  // // Set the tolerance for the end effector's position
  // m_move_group.setGoalPositionTolerance(0.01);
  // // Set the tolerance for the end effector's orientation
  // m_move_group.setGoalOrientationTolerance(0.01);

  //-------------------------------------------------------------------------------------
  // Setup rviz visualization
  //-------------------------------------------------------------------------------------
  // m_rviz.loadCollisionSceneFromFile(); // (if you have a file) 
  m_moveit_visual.loadMarkerPub();
  m_moveit_visual.deleteAllMarkers();
  m_moveit_visual.trigger();
  m_moveit_visual.loadPlanningSceneMonitor();
  m_moveit_visual.loadRemoteControl();

  //-------------------------------------------------------------------------------------
  // Setup planning scene
  //-------------------------------------------------------------------------------------
  // create collision objects
  moveit_msgs::CollisionObject collision_object_table;
  collision_object_table.header.frame_id = m_move_group.getPlanningFrame();
  collision_object_table.id              = "table";

  shape_msgs::SolidPrimitive primitive_table;
  primitive_table.type = primitive_table.BOX;
  primitive_table.dimensions.resize(3);
  primitive_table.dimensions[primitive_table.BOX_X] = 5;
  primitive_table.dimensions[primitive_table.BOX_Y] = 5;
  primitive_table.dimensions[primitive_table.BOX_Z] = 0.02;

  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x    = -0.360298;
  table_pose.position.y    = -0.645357;
  table_pose.position.z    = -0.01001 + 0.08; // added a base for robot

  collision_object_table.primitives.push_back(primitive_table);
  collision_object_table.primitive_poses.push_back(table_pose);
  collision_object_table.operation = collision_object_table.ADD;

  moveit_msgs::CollisionObject collision_object_upper_bound;
  collision_object_upper_bound.header.frame_id = m_move_group.getPlanningFrame();
  collision_object_upper_bound.id              = "upper_bound";

  shape_msgs::SolidPrimitive primitive_upper_bound;
  primitive_upper_bound.type = primitive_upper_bound.BOX;
  primitive_upper_bound.dimensions.resize(3);
  primitive_upper_bound.dimensions[primitive_upper_bound.BOX_X] = 5.0;
  primitive_upper_bound.dimensions[primitive_upper_bound.BOX_Y] = 5.0;
  primitive_upper_bound.dimensions[primitive_upper_bound.BOX_Z] = 0.01;

  geometry_msgs::Pose upper_bound_pose;
  upper_bound_pose.orientation.w = 1.0;
  upper_bound_pose.position.x    = -0.360298;
  upper_bound_pose.position.y    = -0.645357;
  upper_bound_pose.position.z    = 1.4; 

  collision_object_upper_bound.primitives.push_back(primitive_upper_bound);
  collision_object_upper_bound.primitive_poses.push_back(upper_bound_pose);
  collision_object_upper_bound.operation = collision_object_upper_bound.ADD;

  moveit_msgs::CollisionObject collision_object_clamp;
  collision_object_clamp.header.frame_id = m_move_group.getPlanningFrame();
  collision_object_clamp.id              = "clamp";

  shape_msgs::SolidPrimitive primitive_clamp;
  primitive_clamp.type = primitive_clamp.BOX;
  primitive_clamp.dimensions.resize(3);
  primitive_clamp.dimensions[primitive_clamp.BOX_X] = 0.5;
  primitive_clamp.dimensions[primitive_clamp.BOX_Y] = 0.5;
  primitive_clamp.dimensions[primitive_clamp.BOX_Z] = 0.35;

  geometry_msgs::Pose clamp_pose;
  clamp_pose.orientation.w = 1.0;
  clamp_pose.position.x    = -0.360298 + 0.25;
  clamp_pose.position.y    = -0.645357 + 0.25;
  clamp_pose.position.z    = -0.01001 + 0.14;

  collision_object_clamp.primitives.push_back(primitive_clamp);
  collision_object_clamp.primitive_poses.push_back(clamp_pose);
  collision_object_clamp.operation = collision_object_clamp.ADD;

  moveit_msgs::CollisionObject collision_object_zividPlate;
  collision_object_zividPlate.header.frame_id = m_move_group.getPlanningFrame();
  collision_object_zividPlate.id              = "zividPlate";

  shape_msgs::SolidPrimitive primitive_zividPlate;
  primitive_zividPlate.type = primitive_zividPlate.BOX;
  primitive_zividPlate.dimensions.resize(3);
  primitive_zividPlate.dimensions[primitive_zividPlate.BOX_X] = 0.5;
  primitive_zividPlate.dimensions[primitive_zividPlate.BOX_Y] = 0.5;
  primitive_zividPlate.dimensions[primitive_zividPlate.BOX_Z] = 0.06;

  geometry_msgs::Pose zividPlate_pose;
  zividPlate_pose.orientation.w = 1.0;
  zividPlate_pose.position.x    = -0.360298 - 0.25;
  zividPlate_pose.position.y    = -0.645357 + 0.25;
  zividPlate_pose.position.z    = -0.01001 + 0.03;

  collision_object_zividPlate.primitives.push_back(primitive_zividPlate);
  collision_object_zividPlate.primitive_poses.push_back(zividPlate_pose);
  collision_object_zividPlate.operation = collision_object_zividPlate.ADD;

  moveit_msgs::CollisionObject collision_object_zividStand;
  collision_object_zividStand.header.frame_id = m_move_group.getPlanningFrame();
  collision_object_zividStand.id              = "zividStand";

  shape_msgs::SolidPrimitive primitive_zividStand;
  primitive_zividStand.type = primitive_zividStand.BOX;
  primitive_zividStand.dimensions.resize(3);
  primitive_zividStand.dimensions[primitive_zividStand.BOX_X] = 0.19;
  primitive_zividStand.dimensions[primitive_zividStand.BOX_Y] = 0.5;
  primitive_zividStand.dimensions[primitive_zividStand.BOX_Z] = 1.1;

  geometry_msgs::Pose zividStand_pose;
  zividStand_pose.orientation.w = 1.0;
  zividStand_pose.position.x    = -0.360298 - 0.5 + 0.095;
  zividStand_pose.position.y    = -0.645357 + 0.25;
  zividStand_pose.position.z    = -0.01001 + 0.06 + 0.55;

  collision_object_zividStand.primitives.push_back(primitive_zividStand);
  collision_object_zividStand.primitive_poses.push_back(zividStand_pose);
  collision_object_zividStand.operation = collision_object_zividStand.ADD;

  moveit_msgs::CollisionObject collision_object_zividCam;
  collision_object_zividCam.header.frame_id = m_move_group.getPlanningFrame();
  collision_object_zividCam.id              = "zividCam";

  shape_msgs::SolidPrimitive primitive_zividCam;
  primitive_zividCam.type = primitive_zividCam.BOX;
  primitive_zividCam.dimensions.resize(3);
  primitive_zividCam.dimensions[primitive_zividCam.BOX_X] = 0.2;
  primitive_zividCam.dimensions[primitive_zividCam.BOX_Y] = 0.5;
  primitive_zividCam.dimensions[primitive_zividCam.BOX_Z] = 0.3;

  geometry_msgs::Pose zividCam_pose;
  zividCam_pose.orientation.w = 1.0;
  zividCam_pose.position.x    = -0.360298 - 0.5 + 0.19 + 0.1;
  zividCam_pose.position.y    = -0.645357 + 0.25;
  zividCam_pose.position.z    = -0.01001 + 0.06 + 1.1 - 0.15;

  collision_object_zividCam.primitives.push_back(primitive_zividCam);
  collision_object_zividCam.primitive_poses.push_back(zividCam_pose);
  collision_object_zividCam.operation = collision_object_zividCam.ADD;

  moveit_msgs::CollisionObject collision_object_toolPlate;
  collision_object_toolPlate.header.frame_id = m_move_group.getPlanningFrame();
  collision_object_toolPlate.id              = "toolPlate";

  shape_msgs::SolidPrimitive primitive_toolPlate;
  primitive_toolPlate.type = primitive_toolPlate.BOX;
  primitive_toolPlate.dimensions.resize(3);
  primitive_toolPlate.dimensions[primitive_toolPlate.BOX_X] = 0.5;
  primitive_toolPlate.dimensions[primitive_toolPlate.BOX_Y] = 0.5;
  primitive_toolPlate.dimensions[primitive_toolPlate.BOX_Z] = 0.08;

  geometry_msgs::Pose toolPlate_pose;
  toolPlate_pose.orientation.w = 1.0;
  toolPlate_pose.position.x    = -0.360298 - 0.25;
  toolPlate_pose.position.y    = -0.645357 + 0.25 + 1;
  toolPlate_pose.position.z    = -0.01001 + 0.04;

  collision_object_toolPlate.primitives.push_back(primitive_toolPlate);
  collision_object_toolPlate.primitive_poses.push_back(toolPlate_pose);
  collision_object_toolPlate.operation = collision_object_toolPlate.ADD;

  moveit_msgs::CollisionObject collision_object_toolStand;
  collision_object_toolStand.header.frame_id = m_move_group.getPlanningFrame();
  collision_object_toolStand.id              = "toolStand";

  shape_msgs::SolidPrimitive primitive_toolStand;
  primitive_toolStand.type = primitive_toolStand.BOX;
  primitive_toolStand.dimensions.resize(3);
  primitive_toolStand.dimensions[primitive_toolStand.BOX_X] = 0.18;
  primitive_toolStand.dimensions[primitive_toolStand.BOX_Y] = 0.5;
  primitive_toolStand.dimensions[primitive_toolStand.BOX_Z] = 0.35;

  geometry_msgs::Pose toolStand_pose;
  toolStand_pose.orientation.w = 1.0;
  toolStand_pose.position.x    = -0.360298 - 0.5 + 0.09;
  toolStand_pose.position.y    = -0.645357 + 0.25 + 1;
  toolStand_pose.position.z    = -0.01001 + 0.08 + 0.175;

  collision_object_toolStand.primitives.push_back(primitive_toolStand);
  collision_object_toolStand.primitive_poses.push_back(toolStand_pose);
  collision_object_toolStand.operation = collision_object_toolStand.ADD;

  // push them to planning scene
  m_collision_objects.push_back(collision_object_table);
  m_collision_objects.push_back(collision_object_upper_bound);
  m_collision_objects.push_back(collision_object_clamp);
  m_collision_objects.push_back(collision_object_zividPlate);
  m_collision_objects.push_back(collision_object_zividStand);
  m_collision_objects.push_back(collision_object_zividCam);
  m_collision_objects.push_back(collision_object_toolPlate);
  m_collision_objects.push_back(collision_object_toolStand);

  m_planning_scene.addCollisionObjects(m_collision_objects);
};

bool MoveItService::servicePlanTo(agiprobot_msgs::moveit::Request& req,
                                  agiprobot_msgs::moveit::Response& res)
{
  ROS_WARN_STREAM(
    m_node_name << " : Received a planning request, starting to plan for trajectory.");
  //-------------------------------------------------------------------------------------
  // start planning process in a new thread, so it won't block the main control loop
  //-------------------------------------------------------------------------------------
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //-------------------------------------------------------------------------------------
  // retrieve current robot state from /joint_state
  //-------------------------------------------------------------------------------------
  m_move_group.setStartStateToCurrentState();

  //-------------------------------------------------------------------------------------
  // set target state, show it in rviz
  //-------------------------------------------------------------------------------------
  m_move_group.setJointValueTarget(req.joint_pose);
  m_planning_scene.applyCollisionObjects(m_collision_objects);

  //-------------------------------------------------------------------------------------
  // plan the trajectory from current state to target state
  //-------------------------------------------------------------------------------------
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  m_move_group.plan(plan);

  // contact ROS master
  ros::spinOnce();

  //-------------------------------------------------------------------------------------
  // service will return immediately, letting planning executed in background
  //-------------------------------------------------------------------------------------
  return true;
}

