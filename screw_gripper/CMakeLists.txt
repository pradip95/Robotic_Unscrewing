cmake_minimum_required(VERSION 3.0.2)
project(screw_gripper)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

find_package(catkin REQUIRED
  roscpp
  ros_opcua_msgs
  ros_opcua_srvs
  message_generation
  std_msgs
)

add_message_files(FILES
  ArduinoInfo.msg
  Feedback.msg
  OPCUAInfo.msg
)

add_service_files(FILES
  End.srv
  Execute.srv
  Initiate.srv
  ManualControl.srv
  ReadInfo.srv
  Release.srv
  Setup.srv
  Stop.srv
  Unrelease.srv
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp ros_opcua_msgs ros_opcua_srvs message_runtime std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gripper_system
  src/start.cpp
)

add_dependencies(gripper_system
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(gripper_system
  ${catkin_LIBRARIES}
  ${FreeOpcUa_LIBRARIES}
)

install(TARGETS gripper_system
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)