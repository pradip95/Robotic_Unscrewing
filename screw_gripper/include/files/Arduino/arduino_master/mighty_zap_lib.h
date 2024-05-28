/**
 * This file contains all information about data communication with the MightyZap servo actuator
 * and explanation of all commands extracted from official MightyZap API library.
 * (source: https://drive.google.com/drive/folders/10Gyg43ip5ONgJQ-MeVkDVBnFzeVg54I6)
 *
 * @author uxpdp
 */

#pragma once

/*
  MightyZap library command list:
   - Initalizations/ Settings:
    - MightyZAP m_zap(Serial &Serial, int Pin)                  // Communication port setting for Servo Actuator (when using IR-STS01 MightyZAP Arduino Servo Sheild)
     - Serial:          When using the Arduino Leonardo board, it communicates with the Servo actuator through the Serial port 1.
     - Pin:             Switch transmission/ reception (default : 2)
    - MightyZAP m_zap(Serial &Serial, int Pin, int Tx_Level)    // Communication port setting for Servo Actuator (when using Arduino Board only w/o shield)
     - Serial:          When using the Arduino Leonardo board, it communicates with the Servo actuator through the Serial port 1.
     - Pin:             Switch transmission/reception using the corresponding GPIO pin.
     - Tx_Level: RS485 Control level setting
    - m_zap.begin(int Datarate)                                 // Communication speed setting
     - Datarate:        16 ^= 115200 Baudrate, 32 ^= 57600 Baudrate, 64 ^= 19200 Baudrate, 128 ^= 9600 Baudrate
   - Universal commands (More details of Data map in User Maunal (source: https://mightyzap.com/en/digitalarchive4/?category1=User+Manual&mod=document&pageid=1&uid=298)):
    - m_zap.readByte(int Servo_ID, int Address)                 // Read data directly from the address to read 1 byte data of Memory/ Parameter in Data map
    - m_zap.readint(int Servo_ID, int Address)                  // Read data directly from the address to read 2 byte data of Memory/ Parameter in Data map
    - m_zap.writeByte(int Servo_ID, int Address, int Data)      // Input data directly to the address to input 1 byte data of Memory/ Parameter in Data map
    - m_zap.writeint(int Servo_ID, int Address, int Data)       // Input data directly to the address to input 2 byte data of Memory/ Parameter in Data map
   - Parameter Readings:
    - m_zap.getModelNumber(int Servo_ID)                        // Model number check of connected Servo actuator
    - m_zap.Version(int Servo_ID)                               // Servo actuator firmware version check
    - m_zap.PresentTemperature(int Servo_ID)                    // Present temperature check of Servo actuator (just for reference. Not exact info)
    - m_zap.presentPosition(int Servo_ID)                       // Read present position of Servo actuator
    - m_zap.Moving(int Servo_ID)                                // Read moving status of Servo actuator
   - Parameter/ Volatile Writings:
    - m_zap.ledOn(int Servo_ID, int Servo_LED_Color)            // Servo actuator LED setting
     - Servo_LED_Color: 0 = Off, 1 = Red, 2 = Green
    - m_zap.GoalPosition(int Servo_ID, int Position)            // Servo actuator Goal Position setting
     - Position:        0 - 4095 (default: 0 - 4095)
    - m_zap.GoalSpeed(int Servo_ID, int Speed)                  // Moving speed value setting of Servo actuator
     - Speed:           1 (Min speed) - 1023 (Max speed), 0 ^= 1023
    - m_zap.GoalCurrent(int Servo_ID, int Current)              // Max current limit setting of Servo actuator in milliampere
     - Current:         1 - 1600; the smaller value, the smaller force & speed
    - m_zap.forceEnable(int Servo_ID, int Value)                // Force On/ Off of Servo actuator
   - Memory/ Non-Volatile Writings:
    - m_zap.ServoID(int Servo_current_ID, int Servo_new_ID)     // Servo actuator ID setting
    - m_zap.ShortStrokeLimit(int Servo_ID, int Position)        // Set short stroke limit (retracting direction) of Servo actuator
     - Position:        0 - 4095
    - m_zap.LongStrokeLimit(int Servo_ID, int Position)         // Set long stroke limit (extending direction) of Servo actuator
     - Position:        0 - 4095
    - m_zap.Acceleration(int Servo_ID, int Acceleration)        // Set acceleration rate of Servo actuator
     - Acceleration:    0 - 255; the smaller value, the smoother start
    - m_zap.Deceleration(int Servo_ID, int Deceleration)        // Set deceleration rate of Servo actuator
     - Deceleration:    0 - 255; the smaller value, the smoother stop
*/

/*
 * IR Protocol Data map addresses for "Universal commands"
 * More details in User Maunal (source: https://mightyzap.com/en/digitalarchive4/?category1=User+Manual&mod=document&pageid=1&uid=298)
 * R = Read, W = Write, L = Low byte, H = High byte
 */
// Memory addresses
const int MODEL_NUMBER_L_R_ADDRESS = 0;
const int MODEL_NUMBER_H_R_ADDRESS = 1;
const int FIRMWARE_VERSION_R_ADDRESS = 2;
const int ID_RW_ADDRESS = 3;
const int BAUD_RATE_RW_ADDRESS = 4;
const int SHORT_STROKE_LIMIT_L_RW_ADDRESS = 6;
const int SHORT_STROKE_LIMIT_H_RW_ADDRESS = 7;
const int LONG_STROKE_LIMIT_L_RW_ADDRESS = 8;
const int LONG_STROKE_LIMIT_H_RW_ADDRESS = 9;
const int PROTOCOL_TYPE_RW_ADDRESS = 10;
const int LOWEST_LIMIT_VOLTAGE_R_ADDRESS = 12;
const int HIGHEST_LIMIT_VOLTAGE_RW_ADDRESS = 13;
const int MOTOR_OPERATING_RATE_L_RW_ADDRESS = 14;
const int MOTOR_OPERATING_RATE_H_RW_ADDRESS = 15;
const int FEEDBACK_RETURN_MODE_RW_ADDRESS = 16;
const int ALARM_LED_RW_ADDRESS = 17;
const int ALARM_SHUTDOWN_RW_ADDRESS = 18;
const int START_COMPLIANCE_MARGIN_RW_ADDRESS = 19;
const int END_COMPLIANCE_MARGIN_RW_ADDRESS = 20;
const int SPEED_LIMIT_L_RW_ADDRESS = 21;
const int SPEED_LIMIT_H_RW_ADDRESS = 22;
const int CALIBRATION_SHORT_STROKE_L_R_ADDRESS = 24;
const int CALIBRATION_SHORT_STROKE_H_R_ADDRESS = 25;
const int CALIBRATION_LONG_STROKE_L_R_ADDRESS = 26;
const int CALIBRATION_LONG_STROKE_H_R_ADDRESS = 27;
const int ACCELERATION_RATION_RW_ADDRESS = 33;
const int DECELERATION_RATION_RW_ADDRESS = 34;
const int CURRENT_I_GAIN_RW_ADDRESS = 35;
const int CURRENT_P_GAIN_RW_ADDRESS = 36;
const int SPEED_D_GAIN_RW_ADDRESS = 37;
const int SPEED_I_GAIN_RW_ADDRESS = 38;
const int SPEED_P_GAIN_RW_ADDRESS = 39;
const int MIN_POSITION_CALIBRATION_RW_ADDRESS = 46;
const int MAX_POSITION_CALIBRATION_RW_ADDRESS = 47;
const int CURRENT_LIMIT_L_RW_ADDRESS = 52;
const int CURRENT_LIMIT_H_RW_ADDRESS = 53;
// Parameter addresses
const int FORCE_ON_OFF_RW_ADDRESS = 128;
const int LED_RW_ADDRESS = 129;
const int GOAL_POSITION_L_RW_ADDRESS = 134;
const int GOAL_POSITION_H_RW_ADDRESS = 135;
const int GOAL_SPEED_L_RW_ADDRESS = 136;
const int GOAL_SPEED_H_RW_ADDRESS = 137;
const int GOAL_CURRENT_L_RW_ADDRESS = 138;
const int GOAL_CURRENT_H_RW_ADDRESS = 139;
const int PRESENT_POSITION_L_R_ADDRESS = 140;
const int PRESENT_POSITION_H_R_ADDRESS = 141;
const int PRESENT_CURRENT_L_R_ADDRESS = 142;
const int PRESENT_CURRENT_H_R_ADDRESS = 143;
const int PRESENT_MOTOR_OPERATING_RATE_L_R_ADDRESS = 144;
const int PRESENT_MOTOR_OPERATING_RATE_H_R_ADDRESS = 145;
const int PRESENT_VOLTAGE_R_ADDRESS = 146;
const int MOVING_R_ADDRESS = 150;