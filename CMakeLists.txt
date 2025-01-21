cmake_minimum_required(VERSION 3.0.2)
project(Mission_Management)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

## Collect all .msg files in the msg directory
FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

## Add message files only if any are found
if(messages_to_build)
  add_message_files(
    FILES
    ${messages_to_build}
  )
else()
  message(WARNING "No .msg files found in the msg directory!")
endif()

## Generate message headers and dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare this catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

## Include catkin headers
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install the msg directory
install(DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
