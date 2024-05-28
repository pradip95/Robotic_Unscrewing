/**
 * This file contains an object class which implements the communication management of OPCUA's gripper system part.
 *
 * @author uxpdp
 */

#include "opcua_handle.h"

#pragma once

/**
 * An object class which handles with the gripper system part of OPCUA.
 */
class OPCUAGripper
{
public:
	/**
	 * Constructor which initiates the handling with OPCUA.
	 */
	OPCUAGripper() {}

	/**
	 * OPCUA function to perform writing the given data to the associated OPCUA node ids with handshake principle.
	 *
	 * @param readClient OPCUA service client for OPCUA reading
	 * @param writeClient OPCUA service client for OPCUA writing
	 * @param commandIndexList list of command indices
	 * @return contain pair for execution feedback in form of an error signal and error message
	 */
	const std::pair<const bool, const std::string> commandExecute(ros::ServiceClient &readClient, ros::ServiceClient &writeClient, const std::list<uint8_t> &commandIndexList)
	{
		for (uint8_t commandIndex : commandIndexList)
		{
			// Perform data writing on OPCUA node
			if (!opcuaHandle.callWriteService(writeClient, OPCUA_COMMAND_INDEX_NODE_ID, commandIndex) || !opcuaHandle.callWriteService(writeClient, OPCUA_EXECUTE_NODE_ID, true))
			{
				return std::make_pair(false, OPCUA_CALLING_FAILED_MESSAGE);
			}

			// Wait until executing is done
			bool executing;
			do
			{
				if (!opcuaHandle.callReadService(readClient, OPCUA_EXECUTE_NODE_ID, executing))
				{
					return std::make_pair(false, OPCUA_CALLING_FAILED_MESSAGE);
				}
			} while (executing);
			usleep(OPCUA_CALL_DELAY_US);
		}

		// Error check
		bool error;
		std::string errorMessage;
		if (!opcuaHandle.callReadService(readClient, OPCUA_ERROR_NODE_ID, error) || error)
		{
			opcuaHandle.callReadService(readClient, OPCUA_ERROR_MESSAGE_NODE_ID, errorMessage);
		}
		return std::make_pair(error, errorMessage);
	}

protected:
	const char *OPCUA_NODE_NAME = "/opcua_node";
	const char *OPCUA_ENDPOINT_ACCESS_PATH = "/opcua_node/endpoint";
	const char *OPCUA_CONNECT_SERVICE_PATH = "/opcua_node/connect";
	const char *OPCUA_READ_SERVICE_PATH = "/opcua_node/read";
	const char *OPCUA_WRITE_SERVICE_PATH = "/opcua_node/write";
	const char *OPCUA_RELEASE_NODE_ID = "ns=3;s=\"OPC_UA_Ansteuerungsschnittstelle\".\"Freigeben\"";
	const char *OPCUA_COMMAND_INDEX_NODE_ID = "ns=3;s=\"OPC_UA_Ansteuerungsschnittstelle\".\"Befehlsindex\"";
	const char *OPCUA_FORCE_MODE_NODE_ID = "ns=3;s=\"OPC_UA_Ansteuerungsschnittstelle\".\"Kraftmodus\"";
	const char *OPCUA_OUTSIDE_INSIDE_NODE_ID = "ns=3;s=\"OPC_UA_Ansteuerungsschnittstelle\".\"Außen_Innen\"";
	const char *OPCUA_GOAL_POSITION_NODE_ID = "ns=3;s=\"OPC_UA_Ansteuerungsschnittstelle\".\"Zielposition\"";
	const char *OPCUA_EXECUTE_NODE_ID = "ns=3;s=\"OPC_UA_Ansteuerungsschnittstelle\".\"Ausführen\"";
	const char *OPCUA_ERROR_NODE_ID = "ns=3;s=\"OPC_UA_Ansteuerungsschnittstelle\".\"Fehler\"";
	const char *OPCUA_ERROR_MESSAGE_NODE_ID = "ns=3;s=\"OPC_UA_Ansteuerungsschnittstelle\".\"Fehlermeldung\"";
	const char *OPCUA_CURRENT_STROKE_NODE_ID = "ns=3;s=\"OPC_UA_Ansteuerungsschnittstelle\".\"Hub_aktuell\"";
	const uint8_t OPCUA_FAST_STOP_COMMAND_INDEX = 0;
	const uint8_t OPCUA_STOP_COMMAND_INDEX = 1;
	const uint8_t OPCUA_ACKNOWLEDGE_COMMAND_INDEX = 2;
	const uint8_t OPCUA_REFERENCE_COMMAND_INDEX = 3;
	const uint8_t OPCUA_STROKE_MEASURE_COMMAND_INDEX = 4;
	const uint8_t OPCUA_GRIP_COMMAND_INDEX = 5;
	const uint8_t OPCUA_UNHAND_COMMAND_INDEX = 6;
	const uint8_t OPCUA_POSITION_DRIVE_COMMAND_INDEX = 7;
	const uint8_t OPCUA_RELATIVE_DRIVE_COMMAND_INDEX = 8;
	const int OPCUA_GRIPPER_MAX_FORCE_LEVEL = 4;
	const int OPCUA_FORCE_MODE_NUMBER = 2;

	OPCUAHandle opcuaHandle;

private:
	const char *OPCUA_CALLING_FAILED_MESSAGE = "Server_calling_not_successfull!";
	const int OPCUA_CALL_DELAY_US = 200 * 1E3;
};