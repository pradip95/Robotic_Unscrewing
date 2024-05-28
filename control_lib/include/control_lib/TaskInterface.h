#pragma once

#include <functional>
#include <stdio.h>
#include <time.h>
#include <random>
#include <cmath>
#include <vector>
#include <fstream> // For file handling
#include <string>  // For working with strings
#include <ctime>   // For working with timestamps


#include "DrillControl.h"
#include "RobotControl.h"
#include "UsefullFunctions.h"
#include "spiral_interpolation.h"

#define PI 3.14159265


class TaskInterface;


class Spiralsuche
{
private:
  std::vector<std::vector<double>> path_;
  double distance_trajectory_;
  double offset_;

  double distance_steps_          = 0.001;
  double approximation_increment_ = 0.000035;
  
public:
  int plan(TaskInterface* taskInterface);
  int execute(double distanceTrajectory, TaskInterface* taskInterface);
  int executeQuadrantSearch(double distanceTrajectory, double offset, TaskInterface* taskInterface);
  int executeWithLateralControl(double distanceTrajectory, TaskInterface* taskInterface);
  int executeSpiralSearch(double distanceTrajectory, TaskInterface* taskInterface);
  int testScrewAlignment_hex(double distanceTrajectory, TaskInterface* taskInterface);
  int testScrewAlignment_torx(double distanceTrajectory, TaskInterface* taskInterface);
  int testScrewAlignment_hex_withSensor(double distanceTrajectory, TaskInterface* taskInterface);
  int testScrewAlignment_torx20_withSensor(double distanceTrajectory, TaskInterface* taskInterface);
  int testScrewAlignment_torx25_withSensor(double distanceTrajectory, TaskInterface* taskInterface);
  
};


class SukzessiveApproximationBrian
{
private:
  std::vector<std::vector<double> > path_;
  double distance_steps_;
  int number_tries_;

  double contact_time_ = 4;

public:
  int plan(TaskInterface* taskInterface);
  int execute(double distanceSteps, int numberTries, TaskInterface* taskInterface);
};

class SukzessiveApproximationNave
{
private:
  std::vector<std::vector<double> > path_;
  double distance_steps_;
  int number_tries_;

  double contact_time_ = 4;

public:
  int plan(TaskInterface* taskInterface);
  int execute(double distanceSteps, int numberTries, TaskInterface* taskInterface);
};


class LinearschwingungUmMittelpunkt
{
private:
  std::vector<std::vector<double> > path_;
  double loopwidth_;

public:
  int plan(TaskInterface* taskInterface);
  int execute(double loopwidth, TaskInterface* taskInterface);
};

class LissajousscheFiguren
{
private:
  std::vector<std::vector<double> > path_;
  double distance_trajectory_;

public:
  int plan(TaskInterface* taskInterface);
  int execute(double distanceTrajectory, TaskInterface* taskInterface);
};


class TaskInterface
{
private:
public:
  RobotControl* Robot_Task;
  DrillControl* Drill_Task;
  SpiralInterpolation* Spiral;
  TaskInterface()
  {
    Robot_Task = new RobotControl();
    Drill_Task = new DrillControl();
    Spiral = new SpiralInterpolation();
  }
  ~TaskInterface()
  {
    delete Drill_Task;
    delete Robot_Task;
    delete Spiral;
    Robot_Task = nullptr;
    Drill_Task = nullptr;
    Spiral = nullptr;
  }

  Spiralsuche sp_search_;
  SukzessiveApproximationBrian saBrian_search_;
  SukzessiveApproximationNave saNave_search_;
  LinearschwingungUmMittelpunkt lo_search_; // lo: linear oszillation
  LissajousscheFiguren lf_search_;
  
  double radius_searchfield_;
  double strength_max_;
  double strength_min_;
  double velocity_;
  double abort_at_depth_;

  // double acceleration = 0.015; //OwnSearch all
  // double acceleration = 0.5; //QuadSearch T25, T20
  double acceleration_ = 0.1; // SpiSearch SK, T25
  double deceleration_ = 0.1;

  std::vector<double> planningTime_;

  void unscrew();
  bool checkLateralForce(std::vector<double> moveDirection);

  void setGeneralVariables(double radiusSearchfield,
                           double strengthMax,
                           double strengthMin,
                           double velocity,
                           double abortAtDepth)
  {
    if (radiusSearchfield > 0 && abortAtDepth > 0 && velocity > 0)
    {
      radius_searchfield_ = radiusSearchfield;
      strength_max_       = strengthMax;
      strength_min_       = strengthMin;
      velocity_           = velocity;
      abort_at_depth_     = abortAtDepth;
    }
    else
    {
      ROS_ERROR_STREAM("ERROR: no negativ parameters allowed, exept for strengthMin/strenthMax");
    }
  }
};
