/**
 * This file launches the gripper node program which controls the gripper system.
 *
 * @author uxpdp
 */

#include "gripper_handle.h"

/**
 * Start function to launch the gripper node with the gripper system.
 * This node is able to handle and communicate with the Arduino node and the OPCUA node.
 * This node will be kept alive until the user stops this program with Ctrl-C
 * or it will shut down if its initialization was not successful.
 *
 * @param argc number of command line arguments (should be 0)
 * @param argv arguments of command line (should be empty)
 * @return 0 if program execution was successfull, otherwise <> 0
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gripper_system");
	GripperHandle gripperHandle;
	if (!ros::isShuttingDown())
	{
		ros::spin();
		gripperHandle.callOffState();
	}
	return 0;
}