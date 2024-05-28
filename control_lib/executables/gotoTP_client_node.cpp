#include "ros/ros.h"
#include "station_control/TCPPose.h"
#include "station_control/gotoTP.h"
#include <cstdlib>

// copy pasted from
// "https://stackoverflow.com/questions/25444449/how-do-i-convert-a-stdstring-containing-doubles-to-a-vector-of-doubles"
std::vector<double> getVertexIndices(std::string const& pointLine)
{
  std::istringstream iss(pointLine);

  return std::vector<double>{std::istream_iterator<double>(iss), std::istream_iterator<double>()};
}

void deleteParam(ros::NodeHandle* nh)
{
  nh->deleteParam("/gotoTP_client/goalPose");
  nh->deleteParam("/gotoTP_client/useMoveit");
  nh->deleteParam("/gotoTP_client/velocity");
  nh->deleteParam("/gotoTP_client/acceleration");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gotoTP_client");

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<station_control::gotoTP>("gotoTP");
  station_control::gotoTP srv;

  // check if all needed params are given
  if (!nh.hasParam("/gotoTP_client/goalPose"))
  {
    ROS_ERROR_STREAM("param goalPose missing");
    deleteParam(&nh);
    return 1;
  }
  else if (!nh.hasParam("/gotoTP_client/useMoveit"))
  {
    ROS_ERROR_STREAM("param useMoveit missing");
    deleteParam(&nh);
    return 1;
  }
  else if (!nh.hasParam("/gotoTP_client/velocity"))
  {
    ROS_ERROR_STREAM("param velocity missing");
    deleteParam(&nh);
    return 1;
  }
  else if (!nh.hasParam("/gotoTP_client/acceleration"))
  {
    ROS_ERROR_STREAM("param acceleration missing");
    deleteParam(&nh);
    return 1;
  }

  // assign goalPose
  std::string inputGoalPose;
  nh.getParam("/gotoTP_client/goalPose", inputGoalPose);
  std::vector<double> goalPose = getVertexIndices(
    inputGoalPose); // detour because getParam() with vector doesn't seem to work although defined
  if (goalPose.size() == 6)
  {
    for (int i = 0; i < 6; i++)
    {
      srv.request.goalPose.poseVector.push_back(goalPose.at(i));
    }
  }
  else if (inputGoalPose == "HOME" || inputGoalPose == "CLAMPDEVICE" ||
           inputGoalPose == "TOOLCHANGER")
  {
    srv.request.definedLocation = inputGoalPose;
  }
  else
  {
    ROS_ERROR_STREAM("goal pose invalid: length must be 6 but is " << goalPose.size());
    ROS_ERROR_STREAM("other options: HOME/CLAMPDEVICE/TOOLCHANGER");
    deleteParam(&nh);
    return 1;
  }

  // assign useMoveit
  bool inputUseMoveit;
  nh.getParam("/gotoTP_client/useMoveit", inputUseMoveit);
  if (inputUseMoveit == 0 || inputUseMoveit == 1)
  {
    srv.request.useMoveit = inputUseMoveit;
  }
  else
  {
    ROS_ERROR_STREAM("useMoveit value invalid: must be true or false but is " << inputUseMoveit);
    deleteParam(&nh);
    return 1;
  }

  // assign velocity
  double inputVelocity;
  nh.getParam("/gotoTP_client/velocity", inputVelocity);
  if (inputVelocity >= 0)
  {
    srv.request.velocity = inputVelocity;
  }
  else
  {
    ROS_ERROR_STREAM("velocity value invalid: must be greater than 0 but is " << inputVelocity);
    deleteParam(&nh);
    return 1;
  }

  // assign acceleration
  double inputAcceleration;
  nh.getParam("/gotoTP_client/acceleration", inputAcceleration);
  if (inputAcceleration >= 0)
  {
    srv.request.acceleration = inputAcceleration;
  }
  else
  {
    ROS_ERROR_STREAM("acceleration value invalid: must be greater than 0 but is "
                     << inputAcceleration);
    deleteParam(&nh);
    return 1;
  }

  // call service and wait for response
  if (client.call(srv))
  {
    ROS_INFO_STREAM("Service gotoTP called: " << srv.response.answer);
  }
  else
  {
    ROS_ERROR("Failed to call sevice gotoTP");
    deleteParam(&nh);
    return 1;
  }

  deleteParam(&nh);
  return 0;
}