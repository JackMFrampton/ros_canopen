cmake_minimum_required(VERSION 3.5)
project(canopen_chain_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(canopen_master REQUIRED)
find_package(diagnostic_updater REQUIRED)
#find_package(message_generation REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(roslib REQUIRED)
find_package(socketcan_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    filesystem
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetObject.srv"
  "srv/SetObject.srv"
)

include_directories(include)
include_directories(include ${Boost_INCLUDE_DIRS})
include_directories(${socketcan_interface_INCLUDE_DIRS})

# canopen_ros_chain
add_library(canopen_ros_chain
  src/ros_chain.cpp
  src/rosconsole_bridge.cpp
)
target_link_libraries(canopen_ros_chain
  ${Boost_LIBRARIES}
)
ament_target_dependencies(canopen_ros_chain
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# canopen_chain_node
add_executable(chain_node
  src/chain_node.cpp
)
target_link_libraries(chain_node
  canopen_ros_chain
  ${Boost_LIBRARIES}
)

# canopen_sync_node
add_executable(canopen_sync_node
  src/rosconsole_bridge.cpp
  src/sync_node.cpp
)
target_link_libraries(canopen_sync_node
  ${Boost_LIBRARIES}
)

install(
  TARGETS
    chain_node
    canopen_ros_chain
    canopen_sync_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake
                          Boost
                          canopen_master
                          diagnostic_updater
                          message_runtime
                          pluginlib
                          roscpp
                          socketcan_interface
                          std_srvs)

ament_export_include_directories(include)

ament_export_libraries(canopen_ros_chain)

ament_package()