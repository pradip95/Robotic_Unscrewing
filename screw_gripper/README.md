# screw_gripper package

## Description
This package is the control system for a screw gripper system at a disassembly station of "Autonom lernende Roboter (Team B)" of the AgiProbot project. It implements a control service management to this system.<br>
The project architecture was tested on the UR10e robot platform.<br>
Contributors: Heng Wai; Simon Mangold (simon.mangold@kit.edu);

## System Architecture
<img src="./gripper_system_control/System_Architecture.png" width="400" height="602">

## Project Structure
The package **screw_gripper** contains the implemented control service management which enables the controlling and managing of each gripper components of the screw gripper system. It contains the roslauch file "/**screw_gripper/launch/gripper.launch**" which starts the execution of the whole screw gripper system automatically. This package utilizes the **ros_opcua_communication** package.<br>
The package **ros_opcua_communication** builds a communication connection between ROS and an OPC UA-Server like a PLC. This project uses "freeopcua", an implementation method of [OPC UA communication technology](https://github.com/iirob/ros_opcua_communication). 

## Project Installation
Clone the required packages or rather this folder into a catkin workspace. Additionally, the Arduino libraries have to be included in the package **screw_gripper**. For that, you can extract the zip folder **screw_gripper/include/files/Arduino/libraries/Arduino_Gripper_Library** in **screw_gripper/include/libraries** or in the computer's Arduino folder (usually in **Documents**) with a shortcut file to it located in **screw_gripper/include/libraries**. Then execute following commands:
```
$ catkin build
$ source
```
## Project Use
To start this project, execute following command to start the execution file "screw_gripper/launch/gripper.launch":
```
$ roslaunch screw_gripper gripper.launch endpoint:=<IP address of PLC>  port:=<Port of Arduino>
```
The PLC IP address in this project is usually "<ins>opc.tcp://172.22.132.13:4840</ins>" and the Arduino port "<ins>/dev/ttyACM0</ins>".<br>
After that, you will see in the terminal one of the following messages:
```
ROS Gripper System: /gripper_node is ready!                                # roslaunch was successful
ROS Gripper System: /gripper_node can't be started! Launching is aborted!  # roslaunch wasn't successful
```
If it was not successful, check the connection to all controller and your terminal input.<br>
If it was successful, ROS service of the control service management of the screw gripper system are available. To use them and to get details, use following commands:
```
$ rosservice call  # call the service with the provided args
$ rosservice list  # list active services
$ rosservice info  # print information about service
```
At listing service, only the services that begin with "**/gripper_node**" are the ROS service of the control service management. All other service will be used internally and should only be used directly from user for debugging.
