/**
 * This file contains an object class which implements the interface to an Arduino microcontroller using Rosserial.
 *
 * @author uxpdp
 */

#pragma once

#include <ros/ros.h>

/**
 * An object class which handles with the Arduino Rosserial interface and communication.
 */
class ArduinoHandle
{
public:
	/**
	 * Constructor which initiate the communication to Arduino node.
	 *
	 * @param arduinoTopicNames list of all available Arduino topic names
	 */
	ArduinoHandle(const std::list<std::string> &arduinoTopicNames)
	{
		this->arduinoTopicNames = arduinoTopicNames;
	}

	/**
	 * Arduino Rosserial function to check the connection to Arduino
	 * by checking the availability of all initiated Arduino's topics.
	 * !THIS FUNCTION SHOULD BE CALLED BEFORE SUBSCRIBING AND ADVERTISING TOPICS!
	 *
	 * @return {@code true} if connection to Arduino with all its topis is established, otherwise {@code false}
	 */
	bool connected()
	{
		// Wait until connection to Arduino node is initiated
		sleep(INITIALIZATION_DELAY_S);

		// Get all initiated ROS topic names of ROS master
		ros::master::V_TopicInfo rosTopics;
		ros::master::getTopics(rosTopics);
		std::list<std::string> rosTopicNames;
		for (ros::master::TopicInfo rosTopic : rosTopics)
		{
			rosTopicNames.push_back(rosTopic.name);
		}

		// Check whether all Arduino topics are in list of ROS topics
		for (std::string topicName : arduinoTopicNames)
		{
			if (std::find(rosTopicNames.begin(), rosTopicNames.end(), topicName) == rosTopicNames.end())
			{
				ROS_ERROR("%s", ARDUINO_CONNECTION_FAILED);
				return false;
			}
		}
		ROS_INFO("%s", ARDUINO_CONNECTION_ESTABLISHED);
		return true;
	}

	/**
	 * Arduino Rosserial function to publish data on Arduino topics.
	 *
	 * @param <M> type of ROS message
	 * @param <T> type of data
	 * @param publisher publisher which publishs given data
	 * @param data data to publish
	 */
	template <class M, typename T>
	void publishData(const ros::Publisher &publisher, const T &data)
	{
		M pubMessage;
		pubMessage.data = data;
		publisher.publish(pubMessage);
	}

private:
	const char *ARDUINO_CONNECTION_ESTABLISHED = "Connection_to_Arduino_established!";
	const char *ARDUINO_CONNECTION_FAILED = "Connection_to_Arduino_failed!";
	const int INITIALIZATION_DELAY_S = 3;

	std::list<std::string> arduinoTopicNames;
};