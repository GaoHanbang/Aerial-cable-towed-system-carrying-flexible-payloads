cmake_minimum_required(VERSION 3.5)
project(ls2n_fpr_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ls2n_interfaces REQUIRED)
find_package(ls2n_fpr_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)

# headers include
include_directories(
        include/fpr_lib
        include/nodes
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
        src/fpr_lib/common_fpr.cpp
        src/fpr_lib/model_fpr.cpp
        src/fpr_lib/param_fpr.cpp
        src/fpr_lib/controller_fpr.cpp
        src/fpr_lib/observer_fpr.cpp
        )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  # Add a gtest named ${PROJECT_NAME}_my_test
  ament_add_gtest(${PROJECT_NAME}_test test/test_fpr_model.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_lib)
endif()

## autopilot executable
add_executable(autopilot_fpr src/nodes/autopilot_fpr.cpp)
target_link_libraries(autopilot_fpr ${PROJECT_NAME}_lib ${Boost_FILESYSTEM_LIBRARY})
ament_target_dependencies(autopilot_fpr rclcpp std_msgs std_srvs ament_index_cpp nav_msgs ls2n_interfaces ls2n_fpr_interfaces tf2_ros tf2_eigen)

install(DIRECTORY
        launch config worlds models
        DESTINATION share/${PROJECT_NAME}/
        )
install(TARGETS
        autopilot_fpr
        DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
        scripts/trajectory_publisher_node.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
        scripts/fpr_teleop_node.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
        scripts/interaction_detector.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
        scripts/sensor_node.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
        scripts/plot_fpr_flightdata.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
        scripts/plot_sensor_data.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
