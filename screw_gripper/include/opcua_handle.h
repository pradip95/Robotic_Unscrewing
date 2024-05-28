/**
 * This file contains an object class which implements the interface to an OPCUA server using Freeopcua implementation.
 *
 * @author uxpdp
 */

#pragma once

#include <ros/ros.h>
#include <ros_opcua_msgs/TypeValue.h>
#include <ros_opcua_srvs/Connect.h>
#include <ros_opcua_srvs/Read.h>
#include <ros_opcua_srvs/Write.h>

/**
 * An object class which handles with the OPCUA system interface and communication.
 */
class OPCUAHandle
{
public:
    /**
     * Constructor which initiate the communication to OPCUA node.
     */
    OPCUAHandle() {}

    /**
     * OPCUA service function which call the OPCUA's connect service to connect to OPCUA server.
     *
     * @param connectClient OPCUA service client for OPCUA connecting
     * @param endpoint endpoint address of OPCUA server
     * @return {@code true} if service calling was successful, otherwise {@code false}
     */
    bool callConnectService(ros::ServiceClient &connectClient, const std::string &endpoint)
    {
        connecting.request.endpoint = endpoint;
        if (!connectClient.call(connecting) || !connecting.response.success)
        {
            ROS_ERROR("%s", OPCUA_CONNECTION_FAILED);
        }
        return connecting.response.success;
    }

    /**
     * OPCUA service function which call the OPCUA's read service to read data from a child node of OPCUA server.
     * This function modifies the value of {@code storage} actively.
     * The result of this function will be stored in it.
     *
     * @param <T> type of data to store
     * @param readClient OPCUA service client for OPCUA reading
     * @param nodeId name id of node to read
     * @param storage data storage
     * @return {@code true} if service calling was successful, otherwise {@code false}
     */
    template <typename T>
    bool callReadService(ros::ServiceClient &readClient, const std::string &nodeId, T &storage)
    {
        reading.request.node.nodeId = nodeId;
        return readClient.call(reading) && reading.response.success && readDataMapping(reading.response.data, storage);
    }

    /**
     * OPCUA service function which call the OPCUA's write service to write data to a child node of OPCUA server.
     *
     * @param <T> type of data
     * @param writeClient OPCUA service client for OPCUA writing
     * @param nodeId id name of node to write
     * @param data data to write
     * @return {@code true} if service calling was successful, otherwise {@code false}
     */
    template <typename T>
    bool callWriteService(ros::ServiceClient &writeClient, const std::string &nodeId, const T &data)
    {
        writing.request.node.nodeId = nodeId;
        return writeDataMapping(writing.request.data, data) && writeClient.call(writing) && writing.response.success;
    }

private:
    const char *OPCUA_CONNECTION_FAILED = "Connection_to_OPCUA_server_failed!";

    ros_opcua_srvs::Connect connecting;
    ros_opcua_srvs::Read reading;
    ros_opcua_srvs::Write writing;

    /*
     * OPCUA mapping function to determine the type of read OPCUA node's data
     * which will be transferred to argument space corresponding with TypeValue space.
     */
    template <typename T>
    bool readDataMapping(const ros_opcua_msgs::TypeValue &srvData, T &storage)
    {
        const std::type_info &argType = typeid(storage);
        if (argType == typeid(bool))
        {
            storage = srvData.bool_d;
        }
        else if (argType == typeid(int8_t))
        {
            storage = srvData.int8_d;
        }
        else if (argType == typeid(uint8_t))
        {
            storage = srvData.uint8_d;
        }
        else if (argType == typeid(int16_t))
        {
            storage = srvData.int16_d;
        }
        else if (argType == typeid(uint16_t))
        {
            storage = srvData.uint16_d;
        }
        else if (argType == typeid(int32_t))
        {
            storage = srvData.int32_d;
        }
        else if (argType == typeid(uint32_t))
        {
            storage = srvData.uint32_d;
        }
        else if (argType == typeid(int64_t))
        {
            storage = srvData.int64_d;
        }
        else if (argType == typeid(uint64_t))
        {
            storage = srvData.uint64_d;
        }
        else if (argType == typeid(float))
        {
            storage = srvData.float_d;
        }
        else if (argType == typeid(double))
        {
            storage = srvData.double_d;
        }
        else if (argType == typeid(std::string))
        {
            std::stringstream(srvData.string_d) >> storage;
        }
        else
        {
            return false;
        }
        return true;
    }

    /*
     * OPCUA mapping function to assign the type and value of argument to the corresponding TypeValue space.
     */
    template <typename T>
    bool writeDataMapping(ros_opcua_msgs::TypeValue &srvData, const T &argData)
    {
        const std::type_info &argType = typeid(argData);
        if (argType == typeid(bool))
        {
            srvData.type = "bool";
            srvData.bool_d = argData;
        }
        else if (argType == typeid(int8_t))
        {
            srvData.type = "int8";
            srvData.int8_d = argData;
        }
        else if (argType == typeid(uint8_t))
        {
            srvData.type = "uint8";
            srvData.uint8_d = argData;
        }
        else if (argType == typeid(int16_t))
        {
            srvData.type = "int16";
            srvData.int16_d = argData;
        }
        else if (argType == typeid(uint16_t))
        {
            srvData.type = "uint16";
            srvData.uint16_d = argData;
        }
        else if (argType == typeid(int32_t))
        {
            srvData.type = "int32";
            srvData.int32_d = argData;
        }
        else if (argType == typeid(uint32_t))
        {
            srvData.type = "uint32";
            srvData.uint32_d = argData;
        }
        else if (argType == typeid(int64_t))
        {
            srvData.type = "int64";
            srvData.int64_d = argData;
        }
        else if (argType == typeid(uint64_t))
        {
            srvData.type = "uint64";
            srvData.uint64_d = argData;
        }
        else if (argType == typeid(float))
        {
            srvData.type = "float";
            srvData.float_d = argData;
        }
        else if (argType == typeid(double))
        {
            srvData.type = "double";
            srvData.double_d = argData;
        }
        else if (argType == typeid(std::string))
        {
            srvData.type = "string";
            srvData.string_d = argData;
        }
        else
        {
            return false;
        }
        return true;
    }
};