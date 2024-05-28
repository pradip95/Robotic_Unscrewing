/**
 * This file implements the control application for the MightyZap Servo Actuator
 * and the communication & data exchange process management between ROS and MightyZap.
 *
 * @author uxpdp
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <arduino_share.h>
#include <MightyZap.h>
#include "mighty_zap_lib.h"

const int TRANSFER_STATE = LOW;
const int RECEIVE_STATE = HIGH;
const int SERVO_ID_NUMBER = 0;
const int DIRECTION_CONTROL_PORT = 2;

ArduinoShare arduinoShare;
ros::NodeHandle arduinoNode;
std_msgs::Bool pubInitiateState;
std_msgs::Bool pubReleaseState;
std_msgs::Int16 pubPositionValue;
ros::Publisher pubInitiating(arduinoShare.getInitiateTopic(), &pubInitiateState);
ros::Publisher pubReleasing(arduinoShare.getReleaseTopic(), &pubReleaseState);
ros::Publisher pubPositioning(arduinoShare.getPositionTopic(), &pubPositionValue);

bool rosTransferring = false;
bool isInitiated = false;
bool isReleased = false;
bool isMoving = false;

/*
 * MightyZap function to initiate the connection to MightyZap Servo Actuator
 * with activating RS-485 communication it returns as an object.
 * !DON'T FORGET TO SET IN RECEIVE_STATE AGAIN AFTER FINISHING RS-485 COMMUNICATION!
 */
MightyZap mightyZapInit() {
  MightyZap m_zap(&Serial, DIRECTION_CONTROL_PORT);
  m_zap.begin(arduinoShare.getDatarate());
  digitalWrite(DIRECTION_CONTROL_PORT, TRANSFER_STATE);
  return m_zap;
}

/**
 * Rosserial function to publish data on ROS with deactivating RS-485 communication.
 *
 * @param <M> type of message
 * @param <T> type of data
 * @param publisher publisher which publishs given data
 * @param pubMessage message container used by {@code publisher}
 * @param data data to publish
 */
template<class M, typename T>
void publishData(const ros::Publisher& publisher, M& pubMessage, const T& data) {
  digitalWrite(DIRECTION_CONTROL_PORT, RECEIVE_STATE);
  pubMessage.data = data;
  rosTransferring = true;
  publisher.publish(&pubMessage);
}

/**
 * ROS topic function to change the actuator's initiation status.
 *
 * @param initiateMessage message container for this topic
 */
void motorInitiating(const std_msgs::Bool& initiateMessage) {
  if (!rosTransferring) {

    // Initiation procedure
    MightyZap m_zap = mightyZapInit();
    m_zap.forceEnable(SERVO_ID_NUMBER, initiateMessage.data);
    m_zap.ledOn(SERVO_ID_NUMBER, (int)initiateMessage.data * 2);

    // State update
    isInitiated = m_zap.readByte(SERVO_ID_NUMBER, FORCE_ON_OFF_RW_ADDRESS);
    if (isInitiated && initiateMessage.data) {
      m_zap.GoalPosition(SERVO_ID_NUMBER, arduinoShare.getRawStartStroke());
      isReleased = true;
    } else if (!isInitiated) {
      isReleased = false;
    }
    publishData(pubInitiating, pubInitiateState, isInitiated);

  } else {
    rosTransferring = false;
  }
}

/**
 * ROS topic function to change the actuator's release status.
 *
 * @param releaseMessage message container for this topic
 */
void motorReleasing(const std_msgs::Bool& releaseMessage) {
  if (!rosTransferring && isInitiated) {

    // Release procedure
    MightyZap m_zap = mightyZapInit();
    m_zap.forceEnable(SERVO_ID_NUMBER, releaseMessage.data);
    m_zap.ledOn(SERVO_ID_NUMBER, (int)releaseMessage.data + 1);

    // State update
    isReleased = m_zap.readByte(SERVO_ID_NUMBER, FORCE_ON_OFF_RW_ADDRESS);
    publishData(pubReleasing, pubReleaseState, isReleased);

  } else {
    rosTransferring = false;
  }
}

/**
 * ROS topic function to control the actuator's stroke.
 *
 * @param positionMessage message container for this topic
 */
void motorPositioning(const std_msgs::Int16& positionMessage) {
  if (!rosTransferring && isReleased) {

    // Check message data
    int rawPositionValue = positionMessage.data;
    MightyZap m_zap = mightyZapInit();
    if (rawPositionValue > arduinoShare.getRawMaxStroke()) {
      rawPositionValue = arduinoShare.getRawMaxStroke();
    } else if (rawPositionValue == arduinoShare.getStopValue()) {
      rawPositionValue = m_zap.presentPosition(SERVO_ID_NUMBER);
    } else if (rawPositionValue < 0) {
      rawPositionValue = 0;
    }

    // Position control access
    m_zap.GoalPosition(SERVO_ID_NUMBER, rawPositionValue);
    digitalWrite(DIRECTION_CONTROL_PORT, RECEIVE_STATE);
    isMoving = true;

  } else {
    rosTransferring = false;
  }
}

/**
 * Arduino main function to run code once.
 */
void setup() {

  // Setup for manual RS-485 communication control
  pinMode(DIRECTION_CONTROL_PORT, OUTPUT);
  digitalWrite(DIRECTION_CONTROL_PORT, RECEIVE_STATE);

  // Setup for ROS node
  ros::Subscriber<std_msgs::Bool> subInitiating(arduinoShare.getInitiateTopic(), &motorInitiating);
  ros::Subscriber<std_msgs::Bool> subReleasing(arduinoShare.getReleaseTopic(), &motorReleasing);
  ros::Subscriber<std_msgs::Int16> subPositioning(arduinoShare.getPositionTopic(), &motorPositioning);
  arduinoNode.initNode();
  arduinoNode.advertise(pubInitiating);
  arduinoNode.subscribe(subInitiating);
  arduinoNode.advertise(pubReleasing);
  arduinoNode.subscribe(subReleasing);
  arduinoNode.advertise(pubPositioning);
  arduinoNode.subscribe(subPositioning);
  delay(arduinoShare.getWaitingTime());
}

/**
 * Arduino main function to run code in a loop.
 */
void loop() {

  // Query of ROS messages
  arduinoNode.spinOnce();
  delay(arduinoShare.getWaitingTime());

  // After actuator started moving and stopped after that, stop moving procedure will be executed.
  if (isMoving) {
    MightyZap m_zap = mightyZapInit();
    if (!m_zap.Moving(SERVO_ID_NUMBER)) {
      isMoving = false;
      publishData(pubPositioning, pubPositionValue, m_zap.presentPosition(SERVO_ID_NUMBER));
    } else {
      digitalWrite(DIRECTION_CONTROL_PORT, RECEIVE_STATE);
    }
  }
}