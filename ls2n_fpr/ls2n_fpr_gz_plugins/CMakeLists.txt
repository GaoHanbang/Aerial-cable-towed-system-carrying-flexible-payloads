cmake_minimum_required(VERSION 3.5)
project(ls2n_fpr_gz_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} include)
link_directories(${gazebo_dev_LIBRARY_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(fpr_init SHARED src/fpr_init.cc)
target_link_libraries(fpr_init ${GAZEBO_LIBRARIES})
add_library(fpr_contact_sensor SHARED src/fpr_contact_sensor.cc)
target_link_libraries(fpr_contact_sensor ContactPlugin)
ament_target_dependencies(fpr_contact_sensor
        "gazebo_ros"
        "gazebo_msgs"
        "std_msgs"
        )

install(TARGETS fpr_init DESTINATION share/${PROJECT_NAME}/gz_plugins)
install(TARGETS fpr_contact_sensor DESTINATION share/${PROJECT_NAME}/gz_plugins)

ament_package()
