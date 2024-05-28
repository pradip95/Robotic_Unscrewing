#include "control_lib/DrillControl.h"
#include "control_lib/RobotControl.h"
#include "control_lib/TaskInterface.h"


int Spiralsuche::execute(double distanceTrajectory, TaskInterface* taskInterface)
{
  taskInterface->Robot_Task->rtde_control->zeroFtSensor();

  auto startPlanningTime = std::chrono::system_clock::now();
  distance_trajectory_   = distanceTrajectory;
  if (plan(taskInterface) == 1)
    return 1;
  auto endPlanningTime = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsedSecondsPlan;
  elapsedSecondsPlan = endPlanningTime - startPlanningTime;
  taskInterface->planningTime_.push_back((double)elapsedSecondsPlan.count());

  taskInterface->Drill_Task->triggerProg(FASTEN);
  taskInterface->Drill_Task->triggerStart();
  auto startScrewingTime = std::chrono::system_clock::now();


  std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
  taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);
  std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

  for (int h = 0; h < path_.size(); h++)
  {
    // ROS_INFO_STREAM((double)h/path_.size() << " %");

    std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();

    std::vector<double> TCPForce = refTCPForce;
    bool targetNewWp             = false;
    do
    {
      double angle, velocity, acceleration;
      double deceleration = 5;
      acceleration        = taskInterface->acceleration_;
      if (TCPForce.at(1) <= taskInterface->strength_max_ &&
          TCPForce.at(1) >= taskInterface->strength_min_)
      {
        angle    = 0;
        velocity = taskInterface->velocity_;
      }
      else
      {
        refTCPForce.at(1) = TCPForce.at(1);

        double strengthRange = 4;
        double maxAngle      = 45;
        if (TCPForce.at(1) > taskInterface->strength_max_)
        {
          double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
          angle             = percentage * maxAngle;
          if (angle > maxAngle)
          {
            angle        = 89.999;
            velocity     = 0.004;
            acceleration = 0.002;
          }
        }
        else if (TCPForce.at(1) < taskInterface->strength_min_)
        {
          double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
          angle             = -percentage * maxAngle;
          if (angle < maxAngle)
          {
            angle        = -89.999;
            velocity     = 0.004;
            acceleration = 0.002;
          }
        }

        velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
        if (velocity > 0.004)
          velocity = 0.004;
      }

      std::vector<double> pathVecCurrentPose = taskInterface->Robot_Task->TransformBaseToToolFrame(
        {path_[h][0], path_[h][1], path_[h][2]});
      double pathVecCurrentPoseMag =
        sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
      double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

      std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
      pathVecCurrentPose.at(1)        = wpOffset;
      std::vector<double> newWp =
        taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
      taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);

      bool adjustAngle = false;
      int progress;
      do
      {
        bool isWrenchSafetylimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 15;
        double currentDepth      = taskInterface->Robot_Task->calcDepth(contactPose);
        bool isMaxDepthReached   = abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
        if (taskInterface->Drill_Task->isDrillReady())
        {
          taskInterface->Robot_Task->rtde_control->stopL(50);

          auto endScrewingTime = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsedSecondsScrewing;
          elapsedSecondsScrewing = endScrewingTime - startScrewingTime;
          if ((double)elapsedSecondsScrewing.count() > 2)
          {
            taskInterface->Drill_Task->triggerProg(FASTEN);
            taskInterface->Drill_Task->triggerStart();
            startScrewingTime = std::chrono::system_clock::now();
            break;
          }
          else
          {
            // taskInterface->unscrew();

            ROS_INFO_STREAM("SUCCESS: srcew found");
            return 0;
          }
        }
        else if (isMaxDepthReached || isWrenchSafetylimit)
        {
          taskInterface->Robot_Task->rtde_control->stopL();
          if (isMaxDepthReached)
          {
            ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
            return 2;
          }
          else if (isWrenchSafetylimit)
          {
            ROS_ERROR_STREAM("ERROR: Wrench to high, aborting ...!");
            return 3;
          }
        }


        progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();

        TCPForce = taskInterface->Robot_Task->getToolFrameForce();

        double strengthMid = taskInterface->strength_min_ +
                             (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;
        if (angle > 0)
        {
          adjustAngle = TCPForce.at(1) < strengthMid;
          if (TCPForce.at(1) > refTCPForce.at(1))
          {
            adjustAngle       = true;
            refTCPForce.at(1) = TCPForce.at(1);
          }
        }
        else if (angle < 0)
        {
          adjustAngle = TCPForce.at(1) > strengthMid;
          if (TCPForce.at(1) < refTCPForce.at(1))
          {
            adjustAngle       = true;
            refTCPForce.at(1) = TCPForce.at(1);
          }
        }
        else
          adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ ||
                        TCPForce.at(1) < taskInterface->strength_min_;
      } while (progress >= 0 && !adjustAngle);
      taskInterface->Robot_Task->rtde_control->stopL(deceleration);

      if (progress < 0)
        targetNewWp = true;
    } while (!targetNewWp);
  }
  taskInterface->Robot_Task->rtde_control->stopL();

  ROS_ERROR_STREAM("ERROR: screw not found!");
  return 1;
}