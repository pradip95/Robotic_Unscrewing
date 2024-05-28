/**
 * This file contains a container class with all setting constants the user can adjust.
 * Such constants are in the protected declaration.
 * In case of continuous settings, the user can set a value about which information is in the comments.
 * In case of discrete settings, the user can find further information in the private declaration.
 * !THIS CLASS IS ONLY INTENDED TO BE INHERITED BY OTHER CLASSES TO RESTRICT!
 * !THE ACCESS TO ITS ATTRIBUTES BY ANY OTHER CLASS WHICH DOESN'T EXTEND IT!
 *
 * @author uxpdp
 */

#pragma once

/**
 * A container class with all for the user adjustable setting constants.
 */
class UserSettings
{
private:
	const int ADJUSTMENT_PANEL_ARDUINO_DATARATE[4] = {16, 32, 64, 128};		// more information about the values in "mightyzap_lib.h"

protected:
	const int SET_ARDUINO_DATARATE = ADJUSTMENT_PANEL_ARDUINO_DATARATE[1];	// 0 - 3 
	const int SET_ARDUINO_WAITING_TIME_IN_MILLISECONDS = 100;				// any
	const int SET_ARDUINO_CALLBACK_TIME_OUT_IN_SECONDS = 5;					// any
	const int SET_ARDUINO_GOAL_STROKE_LENGTH_TOLERANCE_IN_RAW_FORMAT = 10;	// any
	const int SET_ARDUINO_START_POSITION_STROKE_LENGTH_IN_RAW_FORMAT = 0;	// at least in motor set min stroke length (default: 0)
	const int SET_ARDUINO_GRIP_POSITION_STROKE_LENGTH_IN_RAW_FORMAT = 2900;//3500; //3072;	// at maximum in motor set max stroke length (-> ARDUINO_MAX_STROKE_LENGTH_RAW)
};