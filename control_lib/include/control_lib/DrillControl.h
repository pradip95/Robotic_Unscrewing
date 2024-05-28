#pragma once

#include <iostream>

// ROS
#include "std_msgs/Bool.h"
#include <ros/ros.h>


enum DrillMode
{
  FASTEN,
  LOSEN,
  RESET
};

class DrillControl
{
private:
  ros::NodeHandle nh_DrillControl;
  ros::Subscriber sub_Ready_;
  ros::Subscriber sub_DrillSuccess_;
  ros::AsyncSpinner* spinner_Drillcontrol;

  bool isNotReady_ = false; // indicates if Drill is ready
 
  // updates isNotReady_ variable
  void setIsNotReady(const std_msgs::Bool::ConstPtr& msg);
  void momentDetected(const std_msgs::Bool::ConstPtr& msg);


public:
  DrillControl();
  ~DrillControl();
  bool momentSuccessChanged = false;
  bool momentSuccess_  = false; // indicates if drilltool operation is successful
  /**
   * @brief Selects a programm on the drill controller.
   * @param mode FASTEN for programm 1 and LOSEN for programm 2.
   */
  void triggerProg(DrillMode mode);

  /**
   * @brief Start the currently selected programm.
   */
  void triggerStart();

  bool getMomentSuccess() const;

  /**
   * @brief Checks the current status of the drill.
   * @returns a bool indicating if drill is ready or not.
   */
  bool isDrillReady();

};
