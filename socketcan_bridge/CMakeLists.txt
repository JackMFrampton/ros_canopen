cmake_minimum_required(VERSION 3.5)
project(socketcan_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# find_package(rosidl_typesupport_c REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(socketcan_interface REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    chrono
    system
    thread
)
find_package(nlohmann_json REQUIRED)

include_directories(include)
include_directories(include ${Boost_INCLUDE_DIRS})
include_directories(${socketcan_interface_INCLUDE_DIRS})

# socketcan_signal
add_library(socketcan_signal
  src/socketcan_signal.cpp
)

# socketcan_decoder_encoder
add_library(socketcan_decoder_encoder
  src/socketcan_decoder_encoder.cpp
)
ament_target_dependencies(socketcan_decoder_encoder
  can_msgs)

# socketcan_converter
add_library(socketcan_converter
  src/socketcan_converter.cpp
  src/socketcan_signal.cpp
  src/socketcan_decoder_encoder.cpp
)
ament_target_dependencies(socketcan_converter
  can_msgs
  socketcan_interface)

# socketcan_to_topic
add_library(socketcan_to_topic
  src/socketcan_to_topic.cpp
)
ament_target_dependencies(socketcan_to_topic
  rclcpp
  can_msgs
  socketcan_interface)
target_link_libraries(socketcan_to_topic
  socketcan_decoder_encoder
  socketcan_signal
  socketcan_converter)

# topic_to_socketcan
add_library(topic_to_socketcan
  src/topic_to_socketcan.cpp
)
ament_target_dependencies(topic_to_socketcan
  rclcpp
  can_msgs
  socketcan_interface)
target_link_libraries(topic_to_socketcan
  socketcan_decoder_encoder
  socketcan_signal
  socketcan_converter)

# # socketcan_to_topic_node
add_executable(socketcan_to_topic_node
  src/socketcan_to_topic_node.cpp
)
ament_target_dependencies(socketcan_to_topic_node
  rclcpp
  can_msgs
  socketcan_interface)
target_link_libraries(socketcan_to_topic_node
  socketcan_to_topic
  topic_to_socketcan
  ${Boost_LIBRARIES}
  nlohmann_json::nlohmann_json)

# # topic_to_socketcan_node
add_executable(topic_to_socketcan_node
  src/topic_to_socketcan_node.cpp
)
ament_target_dependencies(topic_to_socketcan_node
  rclcpp
  can_msgs
  socketcan_interface)
target_link_libraries(topic_to_socketcan_node
  socketcan_to_topic
  topic_to_socketcan
  ${Boost_LIBRARIES}
)

# socketcan_bridge_node
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  can_msgs
  socketcan_interface)
target_link_libraries(${PROJECT_NAME}_node
  topic_to_socketcan
  socketcan_to_topic
  ${Boost_LIBRARIES}
  nlohmann_json::nlohmann_json
)

install(
  TARGETS
    ${PROJECT_NAME}_node
    socketcan_to_topic_node
    topic_to_socketcan_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()

  # socketcan to topic
  ament_add_gtest(test_to_topic
    test/to_topic.test
    test/to_topic_test.cpp
  )
  ament_target_dependencies(test_to_topic
    rclcpp
    can_msgs
    socketcan_interface
  )
  target_link_libraries(test_to_topic
    socketcan_to_topic
    topic_to_socketcan
    ${Boost_LIBRARIES}
    nlohmann_json::nlohmann_json
  )

  # topic to socketcan
  ament_add_gtest(test_to_socketcan
    test/to_socketcan.test
    test/to_socketcan_test.cpp
  )
  ament_target_dependencies(test_to_socketcan
    rclcpp
    can_msgs
    socketcan_interface
  )
  target_link_libraries(test_to_socketcan
    socketcan_to_topic
    topic_to_socketcan
    ${Boost_LIBRARIES}
    nlohmann_json::nlohmann_json
  )

  # socketcan converter
  ament_add_gtest(test_conversion
    test/test_conversion.cpp
  )
  ament_target_dependencies(test_conversion
    can_msgs
    socketcan_interface
  )
  target_link_libraries(test_conversion
    topic_to_socketcan
    socketcan_to_topic
    ${Boost_LIBRARIES}
    nlohmann_json::nlohmann_json
  )
endif()

#if(BUILD_TESTING)
#  find_package(rostest REQUIRED)
#  find_package(roslint REQUIRED)
#
#  roslint_cpp()
#  roslint_add_test()
#
#  # unit test for the can_msgs::Frame and can::Frame types.
#  ament_add_gtest(test_conversion
#    test/test_conversion.cpp
#  )
#  target_link_libraries(test_conversion
#    topic_to_socketcan
#    socketcan_to_topic
#  )


#  add_rostest_gtest(test_to_socketcan
#    test/to_socketcan.test
#    test/to_socketcan_test.cpp
#  )
#  target_link_libraries(test_to_socketcan
#    topic_to_socketcan
#  )

#  add_rostest_gtest(test_to_topic
#    test/to_topic.test
#    test/to_topic_test.cpp
#  )
#  target_link_libraries(test_to_topic
#    socketcan_to_topic
#    topic_to_socketcan
#  )

#endif()

ament_export_dependencies(ament_cmake
                          Boost
                          can_msgs
                          roscpp
                          socketcan_interface)

ament_export_include_directories(include)

ament_export_libraries(socketcan_to_topic topic_to_socketcan socketcan_converter socketcan_signal socketcan_decoder_encoder)

ament_package()