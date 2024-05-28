/**
 * This file contains an object class which implements the user interface
 * among the gripper node (this), the Arduino node and the OPCUA node which form the gripper system.
 *
 * @author uxpdp
 */

#pragma once

#include "arduino_gripper.h"
#include "opcua_gripper.h"
#include "screw_gripper/End.h"
#include "screw_gripper/Execute.h"
#include "screw_gripper/Initiate.h"
#include "screw_gripper/ManualControl.h"
#include "screw_gripper/ReadInfo.h"
#include "screw_gripper/Release.h"
#include "screw_gripper/Setup.h"
#include "screw_gripper/Stop.h"
#include "screw_gripper/Unrelease.h"

/**
 * An object class which handles with the communication architecture of the gripper system.
 */
class GripperHandle : public ArduinoGripper, public OPCUAGripper
{
public:
	/**
	 * Constructor which initiate the communication to whole gripper system.
	 */
	GripperHandle()
	{
		// Check connection to all other nodes
		std::string opcuaEndpoint;
		systemNode.getParam(OPCUA_ENDPOINT_ACCESS_PATH, opcuaEndpoint);
		connectClient = systemNode.serviceClient<ros_opcua_srvs::Connect>(OPCUA_CONNECT_SERVICE_PATH);
		if (arduinoHandle.connected() && opcuaHandle.callConnectService(connectClient, opcuaEndpoint))
		{
			// Setup gripper system services
			endService = systemNode.advertiseService(SYSTEM_END_SERVICE_PATH, &GripperHandle::end, this);
			executeService = systemNode.advertiseService(SYSTEM_EXECUTE_SERVICE_PATH, &GripperHandle::execute, this);
			initiateService = systemNode.advertiseService(SYSTEM_INITIATE_SERVICE_PATH, &GripperHandle::initiate, this);
			manualControlService = systemNode.advertiseService(SYSTEM_MANUAL_SERVICE_PATH, &GripperHandle::manualControl, this);
			readInfoService = systemNode.advertiseService(SYSTEM_READ_SERVICE_PATH, &GripperHandle::readInfo, this);
			releaseService = systemNode.advertiseService(SYSTEM_RELEASE_SERVICE_PATH, &GripperHandle::release, this);
			setupService = systemNode.advertiseService(SYSTEM_SETUP_SERVICE_PATH, &GripperHandle::setup, this);
			stopService = systemNode.advertiseService(SYSTEM_STOP_SERVICE_PATH, &GripperHandle::stop, this);
			unreleaseService = systemNode.advertiseService(SYSTEM_UNRELEASE_SERVICE_PATH, &GripperHandle::unrelease, this);

			// Setup Arduino communication
			pubInitiating = systemNode.advertise<std_msgs::Bool>(ARDUINO_TOPIC_INITIATING, ARDUINO_MESSAGE_QUEUE_BUFFER);
			subInitiating = systemNode.subscribe<std_msgs::Bool>(ARDUINO_TOPIC_INITIATING, ARDUINO_MESSAGE_QUEUE_BUFFER, &ArduinoGripper::handleInitiateData, (ArduinoGripper *)this);
			pubReleasing = systemNode.advertise<std_msgs::Bool>(ARDUINO_TOPIC_RELEASING, ARDUINO_MESSAGE_QUEUE_BUFFER);
			subReleasing = systemNode.subscribe<std_msgs::Bool>(ARDUINO_TOPIC_RELEASING, ARDUINO_MESSAGE_QUEUE_BUFFER, &ArduinoGripper::handleReleaseData, (ArduinoGripper *)this);
			pubPositioning = systemNode.advertise<std_msgs::Int16>(ARDUINO_TOPIC_POSITIONING, ARDUINO_MESSAGE_QUEUE_BUFFER);
			subPositioning = systemNode.subscribe<std_msgs::Int16>(ARDUINO_TOPIC_POSITIONING, ARDUINO_MESSAGE_QUEUE_BUFFER, &ArduinoGripper::handlePositionData, (ArduinoGripper *)this);

			// Setup OPCUA communication
			readClient = systemNode.serviceClient<ros_opcua_srvs::Read>(OPCUA_READ_SERVICE_PATH);
			writeClient = systemNode.serviceClient<ros_opcua_srvs::Write>(OPCUA_WRITE_SERVICE_PATH);

			// Initiate gripper system
			/*endClient = systemNode.serviceClient<screw_gripper::End>(SYSTEM_END_SERVICE_PATH);
			sleep(SETUP_INITIALIZATION_DELAY_S);
			if (callOffState())
			{
				system(LOG_DISABLING_COMMAND);

				// Start service run time*/
				ROS_INFO(READY_MESSAGE, SYSTEM_NAME, SYSTEM_NODE_NAME);
				return;
			//}
		}

		// Shutdown gripper system
		ROS_ERROR(ABORT_MESSAGE, SYSTEM_NAME, SYSTEM_NODE_NAME);
		sleep(SHUTDOWN_ERROR_DISPLAY_DELAY_S);
		system(SHUTDOWN_COMMAND);
	}

	/**
	 * System function to switch off the gripper system.
	 *
	 * @return {@code true} if calling was successful, otherwise {@code false}
	 */
	bool callOffState()
	{
		int timeOutCounter = 0;
		while (timeOutCounter < ARDUINO_TIME_OUT_S * 1E3 / (double)ARDUINO_WAITING_TIME_MS)
		{
			if (endClient.call(ending) && ending.response.success)
			{
				return true;
			}
			usleep(ARDUINO_WAITING_TIME_MS * 1E3);
			timeOutCounter++;
		}
		return false;
	}

private:
	const char *LOG_DISABLING_COMMAND = "rosservice call /arduino_node/set_logger_level rosout FATAL";
	const char *SHUTDOWN_COMMAND = "killall rosmaster";

	const int SETUP_INITIALIZATION_DELAY_S = 3;
	const int SHUTDOWN_ERROR_DISPLAY_DELAY_S = 3;

	const char *READY_MESSAGE = "%s: %s is ready!";
	const char *ABORT_MESSAGE = "%s: %s can't be started! Launching is aborted!";
	const char *SUCCESS_MESSAGE = "Execution successful!";
	const char *FAILED_MESSAGE = "Execution failed!";

	const char *SYSTEM_NAME = "ROS Gripper System";
	const char *SYSTEM_NODE_NAME = "/gripper_node";
	const char *SYSTEM_END_SERVICE_PATH = "/gripper_node/end";
	const char *SYSTEM_EXECUTE_SERVICE_PATH = "/gripper_node/execute";
	const char *SYSTEM_INITIATE_SERVICE_PATH = "/gripper_node/initiate";
	const char *SYSTEM_MANUAL_SERVICE_PATH = "/gripper_node/manual_control";
	const char *SYSTEM_READ_SERVICE_PATH = "/gripper_node/read_info";
	const char *SYSTEM_RELEASE_SERVICE_PATH = "/gripper_node/release";
	const char *SYSTEM_SETUP_SERVICE_PATH = "/gripper_node/setup";
	const char *SYSTEM_STOP_SERVICE_PATH = "/gripper_node/stop";
	const char *SYSTEM_UNRELEASE_SERVICE_PATH = "/gripper_node/unrelease";

	ros::NodeHandle systemNode;
	ros::ServiceServer endService;
	ros::ServiceServer executeService;
	ros::ServiceServer initiateService;
	ros::ServiceServer manualControlService;
	ros::ServiceServer readInfoService;
	ros::ServiceServer releaseService;
	ros::ServiceServer setupService;
	ros::ServiceServer stopService;
	ros::ServiceServer unreleaseService;
	ros::Publisher pubInitiating;
	ros::Publisher pubReleasing;
	ros::Publisher pubPositioning;
	ros::Subscriber subInitiating;
	ros::Subscriber subReleasing;
	ros::Subscriber subPositioning;
	ros::ServiceClient connectClient;
	ros::ServiceClient readClient;
	ros::ServiceClient writeClient;
	ros::ServiceClient endClient;
	screw_gripper::End ending;

	bool isGripping = false;

	/*
	 * System service function to put the gripper system in a closed state.
	 */
	bool end(screw_gripper::End::Request &request, screw_gripper::End::Response &response)
	{
		ArduinoGripper::initiated = false;
		arduinoOperate<std_msgs::Bool>(response, pubInitiating, ArduinoGripper::initiated) && opcuaOperate(response, OPCUA_RELEASE_NODE_ID, false);
		return true;
	}

	/*
	 * System service function to send the execution signal for the grip procedure.
	 */
	bool execute(screw_gripper::Execute::Request &request, screw_gripper::Execute::Response &response)
	{
		if (!isGripping)
		{
			ArduinoGripper::motorStrokeRaw = ARDUINO_GRIP_STROKE_LENGTH_RAW;
			arduinoOperate<std_msgs::Int16>(response, pubPositioning, ArduinoGripper::motorStrokeRaw) && opcuaOperate(response, std::list<uint8_t>{OPCUA_GRIP_COMMAND_INDEX});
		}
		else
		{
			ArduinoGripper::motorStrokeRaw = ARDUINO_START_STROKE_LENGTH_RAW;
			opcuaOperate(response, std::list<uint8_t>{OPCUA_UNHAND_COMMAND_INDEX}) && arduinoOperate<std_msgs::Int16>(response, pubPositioning, ArduinoGripper::motorStrokeRaw);
		}
		if (isGripping && response.success)
		{
			isGripping = false;
		}
		else
		{
			isGripping = true;
		}
		return true;
	}

	/*
	 * System service function to put the gripper system in its initial state.
	 */
	bool initiate(screw_gripper::Initiate::Request &request, screw_gripper::Initiate::Response &response)
	{
		ArduinoGripper::initiated = true;
		arduinoOperate<std_msgs::Bool>(response, pubInitiating, ArduinoGripper::initiated) && opcuaOperate(response, OPCUA_RELEASE_NODE_ID, true) && opcuaOperate(response, std::list<uint8_t>{OPCUA_ACKNOWLEDGE_COMMAND_INDEX, OPCUA_STROKE_MEASURE_COMMAND_INDEX, OPCUA_REFERENCE_COMMAND_INDEX});
		return true;
	}

	/*
	 * System service function to control the gripper system manually.
	 */
	bool manualControl(screw_gripper::ManualControl::Request &request, screw_gripper::ManualControl::Response &response)
	{
		isGripping = true;

		// Process Arduino motor stroke data
		int positionValue = ArduinoShare::positionValueMmToRaw(request.motorStroke);
		if (request.relativeDrive)
		{
			ArduinoGripper::motorStrokeRaw += positionValue;
		}
		else
		{
			ArduinoGripper::motorStrokeRaw = positionValue;
		}
		if (ArduinoGripper::motorStrokeRaw > ARDUINO_MAX_STROKE_LENGTH_RAW)
		{
			ArduinoGripper::motorStrokeRaw = ARDUINO_MAX_STROKE_LENGTH_RAW;
		}
		else if (ArduinoGripper::motorStrokeRaw < 0)
		{
			ArduinoGripper::motorStrokeRaw = 0;
		}

		arduinoOperate<std_msgs::Int16>(response, pubPositioning, ArduinoGripper::motorStrokeRaw) && opcuaOperate(response, OPCUA_GOAL_POSITION_NODE_ID, (float)request.gripperStroke) && opcuaOperate(response, std::list<uint8_t>{(uint8_t)(OPCUA_POSITION_DRIVE_COMMAND_INDEX + request.relativeDrive)});
		return true;
	}

	/*
	 * System service function to read current information about the gripper system.
	 */
	bool readInfo(screw_gripper::ReadInfo::Request &request, screw_gripper::ReadInfo::Response &response)
	{
		// Get Arduino info
		response.arduino.initiate = ArduinoGripper::initiated;
		response.arduino.release = ArduinoGripper::released;
		response.arduino.currentStrokeMM = ArduinoGripper::positionValueRawToMm(ArduinoGripper::motorStrokeRaw);

		// Get OPCUA info
		uint8_t forceMode;
		bool forceModeRead = opcuaHandle.callReadService(readClient, OPCUA_FORCE_MODE_NODE_ID, forceMode);
		response.success = forceModeRead && opcuaHandle.callReadService(readClient, OPCUA_RELEASE_NODE_ID, response.opcua.release) && opcuaHandle.callReadService(readClient, OPCUA_OUTSIDE_INSIDE_NODE_ID, response.opcua.gripInside) && opcuaHandle.callReadService(readClient, OPCUA_CURRENT_STROKE_NODE_ID, response.opcua.currentStrokeMM);
		if (forceModeRead)
		{
			response.opcua.softMode = forceMode >= OPCUA_GRIPPER_MAX_FORCE_LEVEL;
			response.opcua.forceLevel = OPCUA_GRIPPER_MAX_FORCE_LEVEL * (1 + (int)response.opcua.softMode) - forceMode;
		}
		return true;
	}

	/*
	 * System service function to make the gripper system controllable.
	 */
	bool release(screw_gripper::Release::Request &request, screw_gripper::Release::Response &response)
	{
		ArduinoGripper::released = true;
		arduinoOperate<std_msgs::Bool>(response, pubReleasing, ArduinoGripper::released) && opcuaOperate(response, std::list<uint8_t>{OPCUA_ACKNOWLEDGE_COMMAND_INDEX});
		return true;
	}

	/*
	 * System service function to adjust the settings of the gripper system.
	 */
	bool setup(screw_gripper::Setup::Request &request, screw_gripper::Setup::Response &response)
	{
		uint8_t forceMode = OPCUA_GRIPPER_MAX_FORCE_LEVEL * (1 + (int)request.softMode) - request.forceLevel;
		response.forceModeChanged = request.forceLevel <= OPCUA_GRIPPER_MAX_FORCE_LEVEL && opcuaHandle.callWriteService(writeClient, OPCUA_FORCE_MODE_NODE_ID, forceMode);
		response.gripDirectionChanged = opcuaHandle.callWriteService(writeClient, OPCUA_OUTSIDE_INSIDE_NODE_ID, request.gripInside);
		return true;
	}

	/*
	 * System service function to stop the execution of the grip procedure.
	 */
	bool stop(screw_gripper::Stop::Request &request, screw_gripper::Stop::Response &response)
	{
		ArduinoGripper::stopped = true;
		arduinoOperate<std_msgs::Int16>(response, pubPositioning, ARDUINO_STOP_VALUE) && opcuaOperate(response, std::list<uint8_t>{OPCUA_STOP_COMMAND_INDEX});
		return true;
	}

	/*
	 * System service function to make the gripper system uncontrollable.
	 */
	bool unrelease(screw_gripper::Unrelease::Request &request, screw_gripper::Unrelease::Response &response)
	{
		ArduinoGripper::released = false;
		arduinoOperate<std_msgs::Bool>(response, pubReleasing, ArduinoGripper::released) && opcuaOperate(response, std::list<uint8_t>{OPCUA_FAST_STOP_COMMAND_INDEX});
		return true;
	}

	/*
	 * System function to execute data exchange on Arduino node.
	 */
	template <class M, class R, typename T>
	bool arduinoOperate(R &response, const ros::Publisher &publisher, const T &data)
	{
		std::pair<const bool, const std::string> arduinoFeedbacks = ArduinoGripper::commandExecute<M>(publisher, data);
		if (!arduinoFeedbacks.first)
		{
			response.execution.arduinoFeedback = SUCCESS_MESSAGE;
		}
		else
		{
			response.execution.arduinoFeedback = FAILED_MESSAGE;
			response.execution.errorMessage = arduinoFeedbacks.second.c_str();
		}
		response.success = !arduinoFeedbacks.first;
		return response.success;
	}

	/*
	 * System function to change single value on OPCUA node.
	 */
	template <class R, typename T>
	bool opcuaOperate(R &response, const std::string &nodeId, const T &data)
	{
		response.success = opcuaHandle.callWriteService(writeClient, nodeId, data);
		if (response.success)
		{
			response.execution.opcuaFeedback = SUCCESS_MESSAGE;
		}
		else
		{
			response.execution.opcuaFeedback = FAILED_MESSAGE;
			std::string errorMessage;
			opcuaHandle.callReadService(readClient, OPCUA_ERROR_MESSAGE_NODE_ID, errorMessage);
			response.execution.errorMessage = errorMessage.c_str();
		}
		return response.success;
	}

	/*
	 * System function to execute commands on OPCUA node.
	 */
	template <class R>
	bool opcuaOperate(R &response, const std::list<uint8_t> &commandIndexList)
	{
		std::pair<const bool, const std::string> opcuaFeedbacks = OPCUAGripper::commandExecute(readClient, writeClient, commandIndexList);
		if (!opcuaFeedbacks.first)
		{
			response.execution.opcuaFeedback = SUCCESS_MESSAGE;
		}
		else
		{
			response.execution.opcuaFeedback = FAILED_MESSAGE;
			response.execution.errorMessage = opcuaFeedbacks.second.c_str();
		}
		response.success = !opcuaFeedbacks.first;
		return response.success;
	}
};