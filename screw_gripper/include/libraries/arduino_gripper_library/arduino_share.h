/**
 * This file contains a container class with all Arduino constants und functions which are used in different packages.
 * This class can also be inherited by other classes to get a direct access to the constants.
 *
 * @author uxpdp
 */

#pragma once

#include "user_settings.h"

/**
 * A container class with Arduino constants and functions.
 */
class ArduinoShare : private UserSettings
{
public:
	/**
	 * Arduino share function to convert a servo actuator's motor position value from millimeter to raw format.
	 *
	 * @param positionValueMm position value in millimeter
	 * @return position value in raw
	 */
	int positionValueMmToRaw(const double &positionValueMm)
	{
		return round(positionValueMm * ARDUINO_MAX_STROKE_LENGTH_RAW / ARDUINO_MAX_STROKE_LENGTH_MM);
	}

	/**
	 * Arduino share function to convert a servo actuator's motor position value from raw format to millimeter.
	 *
	 * @param positionValueRaw position value in raw
	 * @return position value in millimeter
	 */
	double positionValueRawToMm(const int &positionValueRaw)
	{
		return positionValueRaw * ARDUINO_MAX_STROKE_LENGTH_MM / ARDUINO_MAX_STROKE_LENGTH_RAW;
	}

	/**
	 * Arduino share function to get the Arduino's node name.
	 *
	 * @return Arduino node name
	 */
	const char *getNodeName()
	{
		return this->ARDUINO_NODE_NAME;
	}

	/**
	 * Arduino share function to get the Arduno topic to change the servo actuator's initiate state.
	 *
	 * @return initiate topic name
	 */
	const char *getInitiateTopic()
	{
		return this->ARDUINO_TOPIC_INITIATING;
	}

	/**
	 * Arduino share function to get the Arduino topic to change the servo actuator's release state.
	 *
	 * @return release topic name
	 */
	const char *getReleaseTopic()
	{
		return this->ARDUINO_TOPIC_RELEASING;
	}

	/**
	 * Arduino share function to get the Arduino topic to change the servo actuator's motor stroke length.
	 *
	 * @return position topic name
	 */
	const char *getPositionTopic()
	{
		return this->ARDUINO_TOPIC_POSITIONING;
	}

	/**
	 * Arduino share function to get the value which stops the servo actuator's positioning progress.
	 *
	 * @return stop value
	 */
	const int getStopValue()
	{
		return this->ARDUINO_STOP_VALUE;
	}

	/**
	 * Arduino share function to get the servo actuator's maximum stroke length value in raw format.
	 *
	 * @return maximum stroke length in raw
	 */
	const int getRawMaxStroke()
	{
		return this->ARDUINO_MAX_STROKE_LENGTH_RAW;
	}

	/**
	 * Arduino share function to get the servo actuator's maximum stroke length value in millimeter.
	 *
	 * @return maximum stroke length in millimeter
	 */
	const int getMmMaxStroke()
	{
		return this->ARDUINO_MAX_STROKE_LENGTH_MM;
	}

	/**
	 * Arduino share funtion to get the set data rate value for the servo actuator.
	 *
	 * @return data rate
	 */
	const int getDatarate()
	{
		return this->ARDUINO_DATARATE;
	}

	/**
	 * Arduino share function to get the set waiting time value in milliseconds.
	 *
	 * @return waiting time in milliseconds
	 */
	const int getWaitingTime()
	{
		return this->ARDUINO_WAITING_TIME_MS;
	}

	/**
	 * Arduino share function to get the set time out value in seconds.
	 *
	 * @return time out in seconds
	 */
	const int getTimeOut()
	{
		return this->ARDUINO_TIME_OUT_S;
	}

	/**
	 * Arduino share function to get the set stroke lenght tolerance value in raw format.
	 *
	 * @return stroke tolerance in raw
	 */
	const int getRawStrokeTolerance()
	{
		return this->ARDUINO_STROKE_TOLERANCE_RAW;
	}
	/**
	 * Arduino share function to get the servo actuator's set stroke length value at the start position in raw format.
	 *
	 * @return start stroke length in raw
	 */
	const int getRawStartStroke()
	{
		return this->ARDUINO_START_STROKE_LENGTH_RAW;
	}

	/**
	 * Arduino share function to get the servo actuator's set stroke length value at the grip position in raw format.
	 *
	 * @return grip stroke length in raw
	 */
	const int getRawGripStroke()
	{
		return this->ARDUINO_GRIP_STROKE_LENGTH_RAW;
	}

protected:
	// Settings for ROS communication
	const char *ARDUINO_NODE_NAME = "/arduino_node";
	const char *ARDUINO_TOPIC_INITIATING = "/arduino_node/Initiate";
	const char *ARDUINO_TOPIC_RELEASING = "/arduino_node/Release";
	const char *ARDUINO_TOPIC_POSITIONING = "/arduino_node/Position";
	const int ARDUINO_STOP_VALUE = -1;

	// Settings for Arduino's MightyZap servo actuator
	const int ARDUINO_MAX_STROKE_LENGTH_RAW = 4095;
	const double ARDUINO_MAX_STROKE_LENGTH_MM = 30.0;

	// User settings
	const int ARDUINO_DATARATE = SET_ARDUINO_DATARATE;
	const int ARDUINO_WAITING_TIME_MS = SET_ARDUINO_WAITING_TIME_IN_MILLISECONDS;
	const int ARDUINO_TIME_OUT_S = SET_ARDUINO_CALLBACK_TIME_OUT_IN_SECONDS;
	const int ARDUINO_STROKE_TOLERANCE_RAW = SET_ARDUINO_GOAL_STROKE_LENGTH_TOLERANCE_IN_RAW_FORMAT;
	const int ARDUINO_START_STROKE_LENGTH_RAW = SET_ARDUINO_START_POSITION_STROKE_LENGTH_IN_RAW_FORMAT;
	const int ARDUINO_GRIP_STROKE_LENGTH_RAW = SET_ARDUINO_GRIP_POSITION_STROKE_LENGTH_IN_RAW_FORMAT;
};