#include "control_lib/DrillControl.h"

/*
Describtion on whats going on:
DrillControl uses ROS to publish messages and listens to topics
*/

// simple ROS callback funktion
void DrillControl::setIsNotReady(const std_msgs::Bool::ConstPtr& msg)
{
  // assign value to isNotReady_
  isNotReady_ = msg->data;
}

// simple ROS callback funktion for fetching data of drill control
void DrillControl::momentDetected(const std_msgs::Bool::ConstPtr& msg)
{ 
  // Check if there is a change in momentSuccess_
  if (msg->data != momentSuccess_) {
    momentSuccessChanged = false;
  }else{
    momentSuccessChanged = true;
  }

  momentSuccess_ = momentSuccessChanged;
  
  ROS_INFO("Received digital input 01 data: %d", momentSuccess_);
}

// Constructor of DrillControl
DrillControl::DrillControl()
{
  ROS_INFO_STREAM("Starting DrillControl");

  // initiate ROS and create subscribers and an Async Spinner to update massages
  int null = 0;
  ros::init(null, NULL, "drill_control_server_node");
  sub_Ready_ = nh_DrillControl.subscribe("digital_input03", 3, &DrillControl::setIsNotReady, this, ros::TransportHints().tcpNoDelay());
  sub_DrillSuccess_ = nh_DrillControl.subscribe("digital_input01", 10, &DrillControl::momentDetected, this, ros::TransportHints().tcpNoDelay());
  spinner_Drillcontrol = new ros::AsyncSpinner(0);
  spinner_Drillcontrol->start();
}

// Destructor of DrillControl
DrillControl::~DrillControl()
{
  // delete pointes and set storage free
  delete spinner_Drillcontrol;
  spinner_Drillcontrol = nullptr;
}

// method to publish massages on different topics
void DrillControl::triggerProg(DrillMode mode)
{
  // create publisher
  ros::Publisher pubPG0 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output00", 1000);
  ros::Publisher pubPG1 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output01", 1000);
  ros::Publisher pubPG2 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output02", 1000);
  ros::Publisher pubPG3 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output03", 1000);
  ros::Publisher pubPG4 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output04", 1000);
  ros::Publisher pubPG5 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output05", 1000);
  ros::Publisher pubPG6 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output06", 1000);
  ros::Publisher pubPG7 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output07", 1000);

  // create massages
  std_msgs::Bool outpPG0;
  std_msgs::Bool outpPG1;
  std_msgs::Bool outpPG2;
  std_msgs::Bool outpPG3;
  std_msgs::Bool outpPG4;
  std_msgs::Bool outpPG5;
  std_msgs::Bool outpPG6;
  std_msgs::Bool outpPG7;

  // give massages values
  if (mode == FASTEN)
  {
    outpPG0.data = false;
    outpPG1.data = false;
    outpPG2.data = true;
    outpPG3.data = false;
    outpPG4.data = true;
    outpPG5.data = false;
    outpPG6.data = false; //true for program 3,   output 2,4,6
    outpPG7.data = false;
    ROS_INFO("Fastening operation on drill tool");
  }
  if (mode == LOSEN)
  {
    outpPG0.data = false;
    outpPG1.data = false;
    outpPG2.data = false;
    outpPG3.data = false;
    outpPG4.data = true;
    outpPG5.data = false;
    outpPG6.data = true;
    outpPG7.data = false;
    ROS_INFO("Loosening operation on drill tool");
  }
   if (mode == RESET)
  {
    outpPG0.data = false;
    outpPG1.data = false;
    outpPG2.data = false;
    outpPG3.data = false;
    outpPG4.data = false;
    outpPG5.data = false;
    outpPG6.data = false;
    outpPG7.data = false;
    ROS_INFO("Reseting drill tool");
  }
  ros::Rate loopRate(10);
  
  
  int countStart = 0;
  while (countStart < 10)
  {
    pubPG0.publish(outpPG0);
    pubPG1.publish(outpPG1);
    pubPG2.publish(outpPG2);
    pubPG3.publish(outpPG3);
    pubPG4.publish(outpPG4);
    pubPG5.publish(outpPG5);
    pubPG6.publish(outpPG6);
    pubPG7.publish(outpPG7);
    //ROS_INFO("Command sent to turn on the drill tool");
    isNotReady_ = false;
    ros::spinOnce();

    loopRate.sleep();
    countStart++;
  }
  // publish massages with 10Hz 5 times for each value to make sure they are getting received
}

// another method to publish a massage see method, triggerProg() for details
void DrillControl::triggerStart()
{
  ros::Publisher pubStart = nh_DrillControl.advertise<std_msgs::Bool>("digital_output04", 10);
  ros::Publisher pubPG0 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output02", 1000);
  ros::Publisher pubPG1 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output06", 1000);
  std_msgs::Bool outpStart;
  std_msgs::Bool outpPG0;
  std_msgs::Bool outpPG1;

  outpStart.data = true;
  outpPG0.data = true;
  outpPG1.data = true;
  ros::Rate loopRate(10);

  int countStart = 0;
  while (countStart < 2)
  {
    pubStart.publish(outpStart);
    pubPG0.publish(outpPG0);
    //pubPG1.publish(outpPG1);
    ros::spinOnce();

    loopRate.sleep();
    countStart++;
    //std::cout << "data: " << outpStart.data;
  }
  //std::cout << "ready: " << isNotReady_;
}


bool DrillControl::getMomentSuccess() const
{
  return momentSuccess_;
}
  
// because drill ready equals false and drill not ready equals true we have to revert the received
// status massage
bool DrillControl::isDrillReady()
{
  if (isNotReady_ == true)
    return false;
  else
    return true;
}

