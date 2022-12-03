cmake_minimum_required(VERSION 3.5)
project(ls2n_drones_cable_gz_plugins)

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

include_directories(${GAZEBO_INCLUDE_DIRS} include)
link_directories(${gazebo_dev_LIBRARY_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(cable_init SHARED src/cable_init.cc)
target_link_libraries(cable_init ${GAZEBO_LIBRARIES})

install(TARGETS cable_init DESTINATION share/${PROJECT_NAME}/gz_plugins)

ament_package()
