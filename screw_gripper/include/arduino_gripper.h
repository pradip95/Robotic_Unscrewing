/**
 * This file contains an object class which implements the communication management of Arduino's gripper system part.
 *
 * @author uxpdp
 */

#include "arduino_handle.h"
#include "libraries/arduino_gripper_library/arduino_share.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#pragma once

/**
 * An object class which handles with the gripper system part of Arduino.
 */
class ArduinoGripper : public ArduinoShare
{
public:
	/**
	 * Constructor which initiates the handling with Arduino.
	 */
	ArduinoGripper() : arduinoHandle(std::list<std::string>{ARDUINO_TOPIC_INITIATING, ARDUINO_TOPIC_RELEASING, ARDUINO_TOPIC_POSITIONING}) {}

	/**
	 * Arduino Rosserial function to perform writing the given data
	 * to the associated Arduino topic with optional handshake principle.
	 *
	 * @param <M> type of ROS message
	 * @param <T> type of data
	 * @param publisher publisher which publishs given data
	 * @param data data to publish
	 * @param withHandshake optional parameter to set handshake option (default: {@code true}})
	 * @return contain pair for execution handshake feedback in form of an error signal and error message
	 */
	template <class M, typename T>
	const std::pair<const bool, const std::string> commandExecute(const ros::Publisher &publisher, const T &data, const bool withHandshake = true)
	{
		// Execute communication
		arduinoHandle.publishData<M>(publisher, data);
		if (!withHandshake)
		{
			return std::make_pair(false, "");
		}

		// Activate communication state
		rosTransferring = true;
		received = false;

		// Wait callback
		int timeOutCounter = 0;
		while (!received && timeOutCounter < ARDUINO_TIME_OUT_S * 1E3 / (double)ARDUINO_WAITING_TIME_MS)
		{
			usleep(ARDUINO_WAITING_TIME_MS * 1E3);
			ros::spinOnce();
			timeOutCounter++;
		}

		// Error check
		std::string errorMessage;
		if (!received)
		{
			error = true;
			if ((publisher.getTopic() == ARDUINO_TOPIC_RELEASING && initiated) || (publisher.getTopic() == ARDUINO_TOPIC_POSITIONING && released))
			{
				errorMessage = ARDUINO_TIME_OUT_MESSAGE;
			}
			else
			{
				errorMessage = ARDUINO_ACCESS_DENIED_MESSAGE;
			}
		}
		else if (error)
		{
			errorMessage = ARDUINO_UNEXPECTED_RESULT_MESSAGE;
		}
		return std::make_pair(error, errorMessage);
	}

	/**
	 * Arduino Rosserial function to subscribe Arduino's topic {@code ARDUINO_TOPIC_INITIATING} to handle with its messages.
	 *
	 * @param initiateMessage ROS message object for initiating
	 */
	void handleInitiateData(const std_msgs::Bool initiateMessage)
	{
		if (!rosTransferring)
		{
			received = true;
			error = initiated != initiateMessage.data;
			if (initiated && initiateMessage.data)
			{
				motorStrokeRaw = 0;
				released = true;
			}
			else if (!initiateMessage.data)
			{
				released = false;
			}
			if (error)
			{
				initiated = initiateMessage.data;
			}
		}
		else
		{
			rosTransferring = false;
		}
	}

	/**
	 * Arduino Rosserial function to subscribe Arduino's topic {@code ARDUINO_TOPIC_RELEASING} to handle with its messages.
	 *
	 * @param releaseMessage ROS message object for releasing
	 */
	void handleReleaseData(const std_msgs::Bool releaseMessage)
	{
		if (!rosTransferring)
		{
			received = true;
			error = released != releaseMessage.data;
			if (error)
			{
				released = releaseMessage.data;
			}
		}
		else
		{
			rosTransferring = false;
		}
	}

	/**
	 * Arduino Rosserial function to subscribe Arduino's topic {@code ARDUINO_TOPIC_POSITIONING} to handle with its messages.
	 *
	 * @param positionMessage ROS message object for positioning
	 */
	void handlePositionData(const std_msgs::Int16 positionMessage)
	{
		if (!rosTransferring)
		{
			received = true;
			error = positionMessage.data < motorStrokeRaw - ARDUINO_STROKE_TOLERANCE_RAW || positionMessage.data > motorStrokeRaw + ARDUINO_STROKE_TOLERANCE_RAW;
			motorStrokeRaw = positionMessage.data;
			if (stopped)
			{
				error = !error;
				stopped = false;
			}
		}
		else
		{
			rosTransferring = false;
		}
	}

protected:
	const int ARDUINO_MESSAGE_QUEUE_BUFFER = 10;

	ArduinoHandle arduinoHandle;
	bool stopped = false;
	bool initiated = false;
	bool released = false;
	int motorStrokeRaw = 0;

private:
	const char *ARDUINO_ACCESS_DENIED_MESSAGE = "No_callback_because_current_state_denied_access!";
	const char *ARDUINO_TIME_OUT_MESSAGE = "Too_long_waiting_for_callback!";
	const char *ARDUINO_UNEXPECTED_RESULT_MESSAGE = "Unexpected_result_value!";

	bool rosTransferring = false;
	bool received = false;
	bool error = false;
};