cmake_minimum_required(VERSION 3.8)
project(ls2n_px4_ros_com)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fastcdr REQUIRED)

# export the configuration file to PX4 source
if(NOT DEFINED ENV{PX4_SOURCE_DIR})
  MESSAGE(FATAL_ERROR "Please define PX4_SOURCE_DIR with your PX4 folder path to compile this package.")
endif()
configure_file(templates/uorb_rtps_message_ids.yaml $ENV{PX4_SOURCE_DIR}/msg/tools/uorb_rtps_message_ids.yaml)

# generate the yaml file for the RTPS agent
add_custom_command(
        OUTPUT ${CMAKE_BINARY_DIR}/urtps_bridge_topics.yaml
        COMMAND ${PYTHON_EXECUTABLE} $ENV{PX4_SOURCE_DIR}/msg/tools/uorb_to_ros_rtps_ids.py
        -i templates/uorb_rtps_message_ids.yaml
        -o ${CMAKE_BINARY_DIR}/urtps_bridge_topics.yaml
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS templates/uorb_rtps_message_ids.yaml
)

# generate micro RTPS agent code
get_filename_component(PX4_MSGS_DIR "../" ABSOLUTE BASE_DIR ${px4_msgs_DIR})
set(MSGS_DIR "${PX4_MSGS_DIR}/msg" CACHE INTERNAL "MSGS_DIR")
add_custom_command(
        OUTPUT deserialize_wrapper.h deserialize_wrapper.cpp microRTPS_agent.cpp microRTPS_agent.h
        COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_microRTPS_agent.py -t ${MSGS_DIR}
        -o ${CMAKE_BINARY_DIR} -c ${CMAKE_BINARY_DIR}/urtps_bridge_topics.yaml
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS ${CMAKE_BINARY_DIR}/urtps_bridge_topics.yaml templates/microRTPS_agent.cpp.em templates/microRTPS_agent.h.em
        templates/deserialize_wrapper.cpp.em templates/deserialize_wrapper.h.em
)

# Compile the micro RTPS agent
include_directories(src)
add_executable(micrortps_agent microRTPS_agent.cpp deserialize_wrapper.cpp
               src/microRTPS_transport.cpp src/microRTPS_timesync.cpp)
target_link_libraries(micrortps_agent fastcdr)
ament_target_dependencies(micrortps_agent rclcpp px4_msgs std_msgs)

install(TARGETS
        micrortps_agent
        DESTINATION lib/${PROJECT_NAME})

ament_package()
