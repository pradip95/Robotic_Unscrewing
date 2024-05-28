#include "control_lib/RobotControl.h"

/*
Describtion:
RobotControl uses the ur_rtde library with its rtde_control and rtde_receive classes to control and
communicate with the robot Visit their website for information on implemented methods:
https://sdurobotics.gitlab.io/ur_rtde/api/api.html
*/

// ROS callback function, that stores received messages
void RobotControl::callbackFTS(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  TCP_force_sensor_.at(0) = msg->wrench.force.x;
  TCP_force_sensor_.at(1) = msg->wrench.force.y;
  TCP_force_sensor_.at(2) = msg->wrench.force.z;
  TCP_force_sensor_.at(3) = msg->wrench.torque.x;
  TCP_force_sensor_.at(4) = msg->wrench.torque.y;
  TCP_force_sensor_.at(5) = msg->wrench.torque.z;
}

RobotControl::RobotControl()
{
  ROS_INFO_STREAM("Starting RobotControl");

  // create ur_rtde objects
  rtde_control = new RTDEControlInterface(IP_ADRESS_ROBI);
  rtde_receive = new RTDEReceiveInterface(IP_ADRESS_ROBI);

  // define parameters of endeffector
  rtde_control->setPayload(payload_mass_, payload_CG_);
  rtde_control->zeroFtSensor();
  rtde_control->setTcp(tcp_location_);

  // initiate ROS
  const std::string NODE_NAME = "move_group_ur10e";
  int null = 0;
  ros::init(null, NULL, NODE_NAME);
  ros::NodeHandle nh_fts;

  // create subscriber and async spinner to upade messages
  // sub_fts_ = nh_fts.subscribe("ft_sensor/netft_data", 1, &RobotControl::callbackFTS, this);
  spinner_RobotControl_ = new ros::AsyncSpinner(0);
  spinner_RobotControl_->start();
}

// destructor deletes all pointes, frees storage and stopps RTDE Skript
RobotControl::~RobotControl()
{
  delete rtde_control;
  rtde_control = nullptr;

  delete rtde_receive;
  rtde_receive = nullptr;

  delete spinner_RobotControl_;
  spinner_RobotControl_ = nullptr;

  rtde_control->stopScript();
}

std::vector<double> RobotControl::TransformBaseToToolFrame(std::vector<double> baseFramePoint)
{
  // calculate current toolFrame realative to baseframe
  std::vector<double> currentTCPPose  = rtde_receive->getActualTCPPose();
  std::vector<double> xDircetionPoint = rtde_control->poseTrans(currentTCPPose, {1, 0, 0, 0, 0, 0});
  std::vector<double> yDircetionPoint = rtde_control->poseTrans(currentTCPPose, {0, 1, 0, 0, 0, 0});
  std::vector<double> zDircetionPoint = rtde_control->poseTrans(currentTCPPose, {0, 0, 1, 0, 0, 0});

  // initiate matrices
  int BaseFrame[3][3]                = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // unit matrix
  std::vector<double> xToolFrameAxis = {-currentTCPPose.at(0) + xDircetionPoint.at(0),
                                        -currentTCPPose.at(1) + xDircetionPoint.at(1),
                                        -currentTCPPose.at(2) + xDircetionPoint.at(2)};
  std::vector<double> yToolFrameAxis = {-currentTCPPose.at(0) + yDircetionPoint.at(0),
                                        -currentTCPPose.at(1) + yDircetionPoint.at(1),
                                        -currentTCPPose.at(2) + yDircetionPoint.at(2)};
  std::vector<double> zToolFrameAxis = {-currentTCPPose.at(0) + zDircetionPoint.at(0),
                                        -currentTCPPose.at(1) + zDircetionPoint.at(1),
                                        -currentTCPPose.at(2) + zDircetionPoint.at(2)};
  double ToolFrame[3][3] = {{xToolFrameAxis.at(0), yToolFrameAxis.at(0), zToolFrameAxis.at(0)},
                            {xToolFrameAxis.at(1), yToolFrameAxis.at(1), zToolFrameAxis.at(1)},
                            {xToolFrameAxis.at(2), yToolFrameAxis.at(2), zToolFrameAxis.at(2)}};


  // calculate determinant of ToolFrame through spat product
  double detToolFrame =
    ToolFrame[0][2] * (ToolFrame[1][0] * ToolFrame[2][1] - ToolFrame[2][0] * ToolFrame[1][1]) +
    ToolFrame[1][2] * (ToolFrame[2][0] * ToolFrame[0][1] - ToolFrame[0][0] * ToolFrame[2][1]) +
    ToolFrame[2][2] * (ToolFrame[0][0] * ToolFrame[1][1] - ToolFrame[1][0] * ToolFrame[0][1]);
  // inverse determinant
  double invDetToolFrame = 1 / detToolFrame;

  // adjunct procedure to calculate inverse Matrix
  double invToolFrame[3][3];
  invToolFrame[0][0] =
    (ToolFrame[1][1] * ToolFrame[2][2] - ToolFrame[2][1] * ToolFrame[1][2]) * invDetToolFrame;
  invToolFrame[0][1] =
    -(ToolFrame[0][1] * ToolFrame[2][2] - ToolFrame[2][1] * ToolFrame[0][2]) * invDetToolFrame;
  invToolFrame[0][2] =
    (ToolFrame[0][1] * ToolFrame[1][2] - ToolFrame[1][1] * ToolFrame[0][2]) * invDetToolFrame;
  invToolFrame[1][0] =
    -(ToolFrame[1][0] * ToolFrame[2][2] - ToolFrame[2][0] * ToolFrame[1][2]) * invDetToolFrame;
  invToolFrame[1][1] =
    (ToolFrame[0][0] * ToolFrame[2][2] - ToolFrame[2][0] * ToolFrame[0][2]) * invDetToolFrame;
  invToolFrame[1][2] =
    -(ToolFrame[0][0] * ToolFrame[1][2] - ToolFrame[1][0] * ToolFrame[0][2]) * invDetToolFrame;
  invToolFrame[2][0] =
    (ToolFrame[1][0] * ToolFrame[2][1] - ToolFrame[2][0] * ToolFrame[1][1]) * invDetToolFrame;
  invToolFrame[2][1] =
    -(ToolFrame[0][0] * ToolFrame[2][1] - ToolFrame[2][0] * ToolFrame[0][1]) * invDetToolFrame;
  invToolFrame[2][2] =
    (ToolFrame[0][0] * ToolFrame[1][1] - ToolFrame[1][0] * ToolFrame[0][1]) * invDetToolFrame;

  // transforamtion matrix from BaseFrame to ToolFrame (Matrixprodukt)
  double TransBaseToTask[3][3];
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      TransBaseToTask[i][j] = 0;
      for (int k = 0; k < 3; k++)
      {
        TransBaseToTask[i][j] += invToolFrame[i][k] * BaseFrame[k][j];
      }
    }
  }

  // rotate base frame so it matches toolframe orientation and calculate point
  std::vector<double> pointPositionAfterRotation;
  for (int j = 0; j < 3; j++)
  {
    double value = 0;
    for (int k = 0; k < 3; k++)
    {
      value += TransBaseToTask[j][k] * baseFramePoint[k];
    }
    pointPositionAfterRotation.push_back(value);
  }

  // now translate matrix
  std::vector<double> baseFrameTCP = {currentTCPPose[0], currentTCPPose[1], currentTCPPose[2]};
  std::vector<double> TCPPositionAfterRotation;
  for (int j = 0; j < 3; j++)
  {
    double value = 0;
    for (int k = 0; k < 3; k++)
    {
      value += TransBaseToTask[j][k] * baseFrameTCP[k];
    }
    TCPPositionAfterRotation.push_back(value);
  }

  // store calculated force inside vector
  std::vector<double> toolFramePoint;
  toolFramePoint.push_back(pointPositionAfterRotation[0] - TCPPositionAfterRotation[0]);
  toolFramePoint.push_back(pointPositionAfterRotation[1] - TCPPositionAfterRotation[1]);
  toolFramePoint.push_back(pointPositionAfterRotation[2] - TCPPositionAfterRotation[2]);
  for (int i = 3; i < 6; i++)
    toolFramePoint.push_back(0);

  return toolFramePoint;
}

std::vector<double> RobotControl::axisAngleToQuaternions(std::vector<double> axisAngle)
{
  double magnitude = sqrt(pow(axisAngle[0], 2) + pow(axisAngle[1], 2) + pow(axisAngle[2], 2));
  std::vector<double> normAxisAngle = {
    axisAngle[0] / magnitude, axisAngle[1] / magnitude, axisAngle[2] / magnitude};

  double angle;
  if (axisAngle[0] != 0)
    angle = axisAngle[0] / normAxisAngle[0];
  else if (axisAngle[1] != 0)
    angle = axisAngle[1] / normAxisAngle[1];
  else if (axisAngle[2] != 0)
    angle = axisAngle[2] / normAxisAngle[2];

  std::vector<double> quaternion;
  quaternion.push_back(normAxisAngle[0] * sin(angle / 2));
  quaternion.push_back(normAxisAngle[1] * sin(angle / 2));
  quaternion.push_back(normAxisAngle[2] * sin(angle / 2));
  quaternion.push_back(cos(angle / 2));

  return quaternion;
}

// visit the moveit tutorial page, especially the c++ interface for detailed information
// https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html
int RobotControl::moveitPlanTo(std::vector<double> goalTP)
{
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner_RobotControl_(1);
  spinner_RobotControl_.start();

  const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =
    move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "control_lib: Start", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // collision object table
  moveit_msgs::CollisionObject collision_object_table;
  collision_object_table.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object_table.id              = "table";

  shape_msgs::SolidPrimitive primitive_table;
  primitive_table.type = primitive_table.BOX;
  primitive_table.dimensions.resize(3);
  primitive_table.dimensions[primitive_table.BOX_X] = 1.0;
  primitive_table.dimensions[primitive_table.BOX_Y] = 3.0;
  primitive_table.dimensions[primitive_table.BOX_Z] = 0.02;

  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x    = -0.360298;
  table_pose.position.y    = -0.645357;
  table_pose.position.z    = -0.01001;

  collision_object_table.primitives.push_back(primitive_table);
  collision_object_table.primitive_poses.push_back(table_pose);
  collision_object_table.operation = collision_object_table.ADD;

  // collision object calmpingdevice
  moveit_msgs::CollisionObject collision_object_clamp;
  collision_object_clamp.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object_clamp.id              = "calmp";

  shape_msgs::SolidPrimitive primitive_clamp;
  primitive_clamp.type = primitive_clamp.BOX;
  primitive_clamp.dimensions.resize(3);
  primitive_clamp.dimensions[primitive_clamp.BOX_X] = 0.5;
  primitive_clamp.dimensions[primitive_clamp.BOX_Y] = 0.5;
  primitive_clamp.dimensions[primitive_clamp.BOX_Z] = 0.28;

  geometry_msgs::Pose clamp_pose;
  clamp_pose.orientation.w = 1.0;
  clamp_pose.position.x    = -0.360298 + 0.25;
  clamp_pose.position.y    = -0.645357 + 0.25;
  clamp_pose.position.z    = -0.01001 + 0.14;

  collision_object_clamp.primitives.push_back(primitive_clamp);
  collision_object_clamp.primitive_poses.push_back(clamp_pose);
  collision_object_clamp.operation = collision_object_clamp.ADD;

  // collision object zivid base plate
  moveit_msgs::CollisionObject collision_object_zividPlate;
  collision_object_zividPlate.header.frame_id = move_group_interface.getPlanningFrame();
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

  // collision object zivid stand
  moveit_msgs::CollisionObject collision_object_zividStand;
  collision_object_zividStand.header.frame_id = move_group_interface.getPlanningFrame();
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

  // collision object zivid cam
  moveit_msgs::CollisionObject collision_object_zividCam;
  collision_object_zividCam.header.frame_id = move_group_interface.getPlanningFrame();
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

  // collision object toolchange base plate
  moveit_msgs::CollisionObject collision_object_toolPlate;
  collision_object_toolPlate.header.frame_id = move_group_interface.getPlanningFrame();
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

  // collision object toolchange stand
  moveit_msgs::CollisionObject collision_object_toolStand;
  collision_object_toolStand.header.frame_id = move_group_interface.getPlanningFrame();
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

  // ADD
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object_table);
  collision_objects.push_back(collision_object_clamp);
  collision_objects.push_back(collision_object_zividPlate);
  collision_objects.push_back(collision_object_zividStand);
  collision_objects.push_back(collision_object_zividCam);
  collision_objects.push_back(collision_object_toolPlate);
  collision_objects.push_back(collision_object_toolStand);

  ROS_INFO_NAMED("RobotControl", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
  std::vector<double> startPose = rtde_receive->getActualQ();
  start_state.setJointGroupPositions(joint_model_group, startPose);
  move_group_interface.setStartState(start_state);

  std::vector<double> myQuaternion = axisAngleToQuaternions({goalTP[3], goalTP[4], goalTP[5]});

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose target_pose;
  target_pose.orientation.x = myQuaternion[0];
  target_pose.orientation.y = myQuaternion[1];
  target_pose.orientation.z = myQuaternion[2];
  target_pose.orientation.w = myQuaternion[3];
  target_pose.position.x    = goalTP[0];
  target_pose.position.y    = goalTP[1];
  target_pose.position.z    = goalTP[2];
  move_group_interface.setPoseTarget(target_pose);

  ROS_INFO_STREAM("Calculated goal Quaternions: " << myQuaternion[0] << ", " << myQuaternion[1]
                                                  << ", " << myQuaternion[2] << ", "
                                                  << myQuaternion[3]);
  ROS_INFO_STREAM("Calculated goal Position: " << goalTP[0] << ", " << goalTP[1] << ", "
                                               << goalTP[2]);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success =
    (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("RobotControl", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

  ROS_INFO_NAMED("RobotControl", "Visualizing plan as trajectory line");
  visual_tools.publishAxisLabeled(target_pose, "pose");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  planned_plan = my_plan.trajectory_;

  number_of_points_plan_ = planned_plan.joint_trajectory.points.size();
  ROS_INFO_STREAM("Joint_positions: " << number_of_points_plan_);
  for (int i = 0; i < number_of_points_plan_; i++)
  {
    ROS_INFO_STREAM(planned_plan.joint_trajectory.points[i].positions[0]
                    << ", " << planned_plan.joint_trajectory.points[i].positions[1] << ", "
                    << planned_plan.joint_trajectory.points[i].positions[2] << ", "
                    << planned_plan.joint_trajectory.points[i].positions[3] << ", "
                    << planned_plan.joint_trajectory.points[i].positions[4] << ", "
                    << planned_plan.joint_trajectory.points[i].positions[5]);
  }
  return 0;
}

std::vector<double> RobotControl::getTCPForce()
{
  return TCP_force_sensor_;
}

// method using receive messages via ROS
void RobotControl::nullFTS()
{
  ros::NodeHandle nh;
  ros::ServiceClient ft_client = nh.serviceClient<control_lib::String_cmd>("/ft_sensor/bias_cmd");
  control_lib::String_cmd srv;
  srv.request.cmd  = "bias";
  srv.response.res = "";
  if (ft_client.call(srv))
  {
    ROS_INFO_STREAM("net_ft res: " << srv.response.res);
  }
  else
  {
    ROS_ERROR("Failed to call netft bias service");
  }
}

//input the target tcp pose in base coordinate
double RobotControl::calcDepth(std::vector<double> relativeTCPPose)
{
  // determine position and orientation of tool frame
  std::vector<double> currentTCPPose = rtde_receive->getActualTCPPose();
  std::vector<double> offsetAlongY   = rtde_control->poseTrans(currentTCPPose, {0, 1, 0, 0, 0, 0});
  std::vector<double> normalVector   = {offsetAlongY.at(0) - currentTCPPose.at(0),
                                      offsetAlongY.at(1) - currentTCPPose.at(1),
                                      offsetAlongY.at(2) - currentTCPPose.at(2)};
  double magnitudeOfNormalVector =
    sqrt(pow(normalVector.at(0), 2) + pow(normalVector.at(1), 2) + pow(normalVector.at(2), 2));
  std::vector<double> standardizedNormalVector = {normalVector.at(0) / magnitudeOfNormalVector,
                                                  normalVector.at(1) / magnitudeOfNormalVector,
                                                  normalVector.at(2) / magnitudeOfNormalVector};

  // calculate depth with plane in Hessesche Normalform
  double depth = standardizedNormalVector.at(0) * (relativeTCPPose.at(0) - currentTCPPose.at(0)) +
                 standardizedNormalVector.at(1) * (relativeTCPPose.at(1) - currentTCPPose.at(1)) +
                 standardizedNormalVector.at(2) * (relativeTCPPose.at(2) - currentTCPPose.at(2));
  return depth;
}

bool RobotControl::gotoTP(std::vector<double> goal,
                          bool useMoveit,
                          double velocity,
                          double acceleration)
{
  if (useMoveit == true)
  {
    std::vector<double> waypoint;
    std::vector<std::vector<double> > path;

    moveitPlanTo(goal);


    for (int n = 0; n < number_of_points_plan_; n++)
    {
      for (int m = 0; m < 6; m++)
      {
        waypoint.push_back(planned_plan.joint_trajectory.points[n].positions[m]);
      }
      waypoint.push_back(velocity);
      waypoint.push_back(acceleration);
      waypoint.push_back(path_blend_);
      path.push_back(waypoint);

      waypoint.clear();
    }

    return rtde_control->moveJ(path);
  }
  else
  {
    return rtde_control->moveL(goal, velocity, acceleration);
  }
}

void RobotControl::gotoTP(Goal goal, bool useMoveit, double velocity, double acceleration)
{
  if (useMoveit == true)
  {
    std::vector<double> waypoint;
    std::vector<std::vector<double> > path;

    if (goal == HOME)
    {
      moveitPlanTo(home_position_);
    }
    if (goal == CLAMPDEVICE)
    {
      moveitPlanTo(clampdevice_position_);
    }
    if (goal == TOOLCHANGER)
    {
      moveitPlanTo(toolchanger_position_);
    }

    for (int n = 0; n < number_of_points_plan_; n++)
    {
      for (int m = 0; m < 6; m++)
      {
        waypoint.push_back(planned_plan.joint_trajectory.points[n].positions[m]);
      }
      waypoint.push_back(velocity);
      waypoint.push_back(acceleration);
      waypoint.push_back(path_blend_);
      path.push_back(waypoint);

      waypoint.clear();
    }

    rtde_control->moveJ(path);
  }
  else
  {
    switch (goal)
    {
      case HOME:
        rtde_control->moveL(home_position_, velocity, acceleration);
        break;
      case CLAMPDEVICE:
        rtde_control->moveL(clampdevice_position_, velocity, acceleration);
        break;
      case TOOLCHANGER:
        rtde_control->moveL(toolchanger_position_, velocity, acceleration);
        break;
    }
  }
}

bool RobotControl::startForcemode(Forcetype type, double strength)
{
  rtde_control->forceModeSetDamping(1);
  std::vector<double> forceFrame;
  std::vector<double> taskFrame;
  std::vector<int> selectionVector;
  std::vector<double> wrench;
  int forceType = 2;
  std::vector<double> limits;

  switch (type)
  {
    case WRENCH_UP:
      forceFrame      = rtde_receive->getActualTCPPose();
      taskFrame       = {forceFrame.at(0),
                   forceFrame.at(1),
                   forceFrame.at(2),
                   forceFrame.at(3),
                   forceFrame.at(4),
                   forceFrame.at(5)};
      selectionVector = {0, 1, 0, 0, 0, 0};
      wrench          = {0, strength, 0, 0, 0, 0};
      limits          = {2, 0.02, 2, 1, 1, 1};
    case WRENCH_DOWN:
      forceFrame      = rtde_receive->getActualTCPPose();
      taskFrame       = {forceFrame.at(0),
                   forceFrame.at(1),
                   forceFrame.at(2),
                   forceFrame.at(3),
                   forceFrame.at(4),
                   forceFrame.at(5)};
      selectionVector = {1, 1, 1, 0, 0, 0};
      wrench          = {0, 0, -strength, 0, 0, 0};
      limits          = {0.05, 0.05, 0.05, 0.17, 0.17, 0.17};
      break;
    case GETTOOLIN:
      taskFrame       = {0, 0, 0, 0, 0, 0};
      selectionVector = {0, 0, 1, 0, 0, 0};
      wrench          = {0, 0, -strength, 0, 0, 0};
      limits          = {2, 2, 0.02, 1, 1, 1};
      break;
    case GETTOOLOUT:
      taskFrame       = {0, 0, 0, 0, 0, 0};
      selectionVector = {1, 0, 0, 0, 0, 0};
      wrench          = {strength, 0, 0, 0, 0, 0};
      limits          = {0.02, 2, 2, 1, 1, 1};
      break;
    case EJECTTOOLIN:
      taskFrame       = {0, 0, 0, 0, 0, 0};
      selectionVector = {1, 0, 0, 0, 0, 0};
      wrench          = {-strength, 0, 0, 0, 0, 0};
      limits          = {0.02, 2, 2, 1, 1, 1};
      break;
    case EJECTTOOLOUT:
      taskFrame       = {0, 0, 0, 0, 0, 0};
      selectionVector = {0, 0, 1, 0, 0, 0};
      wrench          = {0, 0, strength, 0, 0, 0};
      limits          = {2, 2, 0.02, 1, 1, 1};
      break;
  }

  rtde_control->forceMode(taskFrame, selectionVector, wrench, forceType, limits);


  bool goalReached   = false;
  int vectorPosition = 0;


  if (type == GETTOOLIN)
    vectorPosition = 2;
  if (type == EJECTTOOLIN)
    vectorPosition = 0;
  if (type == GETTOOLOUT)
    vectorPosition = 0;
  if (type == EJECTTOOLOUT)
    vectorPosition = 2;

  if (type == GETTOOLIN || type == EJECTTOOLIN)
  {
    rtde_control->zeroFtSensor();
    while (!goalReached)
    {
      std::vector<double> TCPForce = rtde_receive->getActualTCPForce();
      if (TCPForce[vectorPosition] > strength)
      {
        goalReached = true;
        rtde_control->forceModeStop();
      }
    }
  }
  if (type == GETTOOLOUT || type == EJECTTOOLOUT)
  {
    rtde_control->zeroFtSensor();
    if (type == GETTOOLOUT)
    {
      while (goalReached == false)
      {
        std::vector<double> currentTCPPos = rtde_receive->getActualTCPPose();
        if (currentTCPPos[vectorPosition] > -0.65)
        {
          goalReached = true;
          rtde_control->forceModeStop();
        }
      }
    }
    if (type == EJECTTOOLOUT)
    {
      while (!goalReached)
      {
        std::vector<double> currentTCPPos = rtde_receive->getActualTCPPose();
        if (currentTCPPos[2] > 0.19)
        {
          goalReached = true;
          rtde_control->forceModeStop();
        }
      }
    }
  }

  return true;
}

std::vector<double> RobotControl::getToolFrameForce()
{
  // calculate current toolFrame realative to baseframe
  std::vector<double> currentTCPPose  = rtde_receive->getActualTCPPose();
  std::vector<double> xDircetionPoint = rtde_control->poseTrans(currentTCPPose, {1, 0, 0, 0, 0, 0});
  std::vector<double> yDircetionPoint = rtde_control->poseTrans(currentTCPPose, {0, 1, 0, 0, 0, 0});
  std::vector<double> zDircetionPoint = rtde_control->poseTrans(currentTCPPose, {0, 0, 1, 0, 0, 0});

  // initiate matrices
  int BaseFrame[3][3]                = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // unit matrix
  std::vector<double> xToolFrameAxis = {-currentTCPPose.at(0) + xDircetionPoint.at(0),
                                        -currentTCPPose.at(1) + xDircetionPoint.at(1),
                                        -currentTCPPose.at(2) + xDircetionPoint.at(2)};
  std::vector<double> yToolFrameAxis = {-currentTCPPose.at(0) + yDircetionPoint.at(0),
                                        -currentTCPPose.at(1) + yDircetionPoint.at(1),
                                        -currentTCPPose.at(2) + yDircetionPoint.at(2)};
  std::vector<double> zToolFrameAxis = {-currentTCPPose.at(0) + zDircetionPoint.at(0),
                                        -currentTCPPose.at(1) + zDircetionPoint.at(1),
                                        -currentTCPPose.at(2) + zDircetionPoint.at(2)};
  double ToolFrame[3][3] = {{xToolFrameAxis.at(0), yToolFrameAxis.at(0), zToolFrameAxis.at(0)},
                            {xToolFrameAxis.at(1), yToolFrameAxis.at(1), zToolFrameAxis.at(1)},
                            {xToolFrameAxis.at(2), yToolFrameAxis.at(2), zToolFrameAxis.at(2)}};


  // calculate determinant of ToolFrame through spat product
  double detToolFrame =
    ToolFrame[0][2] * (ToolFrame[1][0] * ToolFrame[2][1] - ToolFrame[2][0] * ToolFrame[1][1]) +
    ToolFrame[1][2] * (ToolFrame[2][0] * ToolFrame[0][1] - ToolFrame[0][0] * ToolFrame[2][1]) +
    ToolFrame[2][2] * (ToolFrame[0][0] * ToolFrame[1][1] - ToolFrame[1][0] * ToolFrame[0][1]);
  // inverse determinant
  double invDetToolFrame = 1 / detToolFrame;

  // adjunct procedure to calculate inverse Matrix
  double invToolFrame[3][3];
  invToolFrame[0][0] =
    (ToolFrame[1][1] * ToolFrame[2][2] - ToolFrame[2][1] * ToolFrame[1][2]) * invDetToolFrame;
  invToolFrame[0][1] =
    -(ToolFrame[0][1] * ToolFrame[2][2] - ToolFrame[2][1] * ToolFrame[0][2]) * invDetToolFrame;
  invToolFrame[0][2] =
    (ToolFrame[0][1] * ToolFrame[1][2] - ToolFrame[1][1] * ToolFrame[0][2]) * invDetToolFrame;
  invToolFrame[1][0] =
    -(ToolFrame[1][0] * ToolFrame[2][2] - ToolFrame[2][0] * ToolFrame[1][2]) * invDetToolFrame;
  invToolFrame[1][1] =
    (ToolFrame[0][0] * ToolFrame[2][2] - ToolFrame[2][0] * ToolFrame[0][2]) * invDetToolFrame;
  invToolFrame[1][2] =
    -(ToolFrame[0][0] * ToolFrame[1][2] - ToolFrame[1][0] * ToolFrame[0][2]) * invDetToolFrame;
  invToolFrame[2][0] =
    (ToolFrame[1][0] * ToolFrame[2][1] - ToolFrame[2][0] * ToolFrame[1][1]) * invDetToolFrame;
  invToolFrame[2][1] =
    -(ToolFrame[0][0] * ToolFrame[2][1] - ToolFrame[2][0] * ToolFrame[0][1]) * invDetToolFrame;
  invToolFrame[2][2] =
    (ToolFrame[0][0] * ToolFrame[1][1] - ToolFrame[1][0] * ToolFrame[0][1]) * invDetToolFrame;

  // transforamtion matrix from BaseFrame to ToolFrame (Matrixprodukt)
  double TransBaseToTask[3][3];
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      TransBaseToTask[i][j] = 0;
      for (int k = 0; k < 3; k++)
      {
        TransBaseToTask[i][j] += invToolFrame[i][k] * BaseFrame[k][j];
      }
    }
  }

  std::vector<double> TCPForceBaseFrame = rtde_receive->getActualTCPForce();
  std::vector<double> TCPForceToolFrame;
  // transform force/wrench relative to BaseFrame into force/wrench realtive to ToolFrame
  for (int i = 0; i <= 3; i = i + 3)
  {
    for (int j = 0; j < 3; j++)
    {
      double value = 0;
      for (int k = 0; k < 3; k++)
      {
        value += TransBaseToTask[j][k] * TCPForceBaseFrame[k + i];
      }
      TCPForceToolFrame.push_back(value);
    }
  }

  return TCPForceToolFrame;
}

void RobotControl::moveTo(Goal goal, bool moveit)
{
  gotoTP(goal, moveit, 0.1, 0.2);
}

void RobotControl::getTool(int fromSlot)
{
  std::vector<double> fromSlotPos;
  switch (fromSlot)
  {
    case 1:
      fromSlotPos = top_L1_;
      break;
    case 2:
      fromSlotPos = top_L2_;
      break;
    case 3:
      fromSlotPos = top_L3_;
      break;
  }
  // gotoTP(TOOLCHANGER, true, 0.8, 0.2);
  gotoTP(fromSlotPos, false, 0.25, 0.1);
  startForcemode(GETTOOLIN, 25);
  startForcemode(GETTOOLOUT, 30);
  // gotoTP(TOOLCHANGER, false, 0.25, 0.1);
}

void RobotControl::releaseTool(Slot toSlot)
{
  std::vector<double> toSlotPos;
  switch (toSlot)
  {
    case SLOT_1:
      toSlotPos = bottom_L1_;
      break;
    case SLOT_2:
      toSlotPos = bottom_L2_;
      break;
    case SLOT_3:
      toSlotPos = bottom_L3_;
      break;
  }

  gotoTP(TOOLCHANGER, true, 0.8, 0.2);
  gotoTP(toSlotPos, false, 0.25, 0.1);
  startForcemode(EJECTTOOLIN, 20);
  startForcemode(EJECTTOOLOUT, 20); // 35
  gotoTP(TOOLCHANGER, false, 0.25, 0.1);
}

void RobotControl::changeTool(Slot fromSlot, Slot toSlot)
{
  std::vector<double> fromSlotPos;
  std::vector<double> toSlotPos;

  switch (fromSlot)
  {
    case SLOT_1:
      fromSlotPos = bottom_L1_;
      break;
    case SLOT_2:
      fromSlotPos = bottom_L2_;
      break;
    case SLOT_3:
      fromSlotPos = bottom_L3_;
      break;
  }

  switch (toSlot)
  {
    case SLOT_1:
      toSlotPos = top_L1_;
      break;
    case SLOT_2:
      toSlotPos = top_L2_;
      break;
    case SLOT_3:
      toSlotPos = top_L3_;
      break;
  }

  gotoTP(TOOLCHANGER, true, 0.03, 0.2);
  gotoTP(fromSlotPos, false, 0.03, 0.2);
  startForcemode(EJECTTOOLIN, 20);
  startForcemode(EJECTTOOLOUT, 35);

  gotoTP(TOOLCHANGER, false, 0.03, 0.2);
  gotoTP(toSlotPos, false, 0.03, 0.2);
  startForcemode(GETTOOLIN, 25);
  startForcemode(GETTOOLOUT, 30);

  gotoTP(TOOLCHANGER, false, 0.03, 0.2);
}

void RobotControl::freeDrive(int repeatHowmany)
{
  char input;
  rtde_control->teachMode();
  for (int i = 0; i < repeatHowmany; i++)
  {
    std::cout << "\nPress any letter to stop freedrive:\n";
    std::cin >> input;
  }

  rtde_control->endTeachMode();
}

void RobotControl::setVelocity(double velocity)
{
  velocity = velocity;
}

void RobotControl::setAcceleration(double acceleration)
{
  acceleration = acceleration;
}
