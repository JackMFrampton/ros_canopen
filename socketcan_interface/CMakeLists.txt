cmake_minimum_required(VERSION 3.5)
project(socketcan_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(console_bridge REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    chrono
    system
    thread
)

include_directories(include)
include_directories(include ${Boost_INCLUDE_DIRS})
include_directories(include ${console_bridge_INCLUDE_DIRS}) 

# ${PROJECT_NAME}_string
add_library(${PROJECT_NAME}_string
    src/string.cpp
)

# socketcan_dump
add_executable(socketcan_dump
  src/candump.cpp
)

target_link_libraries(socketcan_dump
   ${PROJECT_NAME}_string
   ${console_bridge_LIBRARIES}
   ${Boost_LIBRARIES}
)

ament_target_dependencies(socketcan_dump class_loader)

# socketcan_bcm
add_executable(socketcan_bcm
  src/canbcm.cpp
)

target_link_libraries(socketcan_bcm
   ${PROJECT_NAME}_string
   ${console_bridge_LIBRARIES}
   ${Boost_LIBRARIES}
)

# ${PROJECT_NAME}_plugin
add_library(${PROJECT_NAME}_plugin
  src/${PROJECT_NAME}_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
  ${console_bridge_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS
    socketcan_bcm
    socketcan_dump
    ${PROJECT_NAME}_string
    ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(
  FILES
    ${PROJECT_NAME}_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

#if(BUILD_TESTING)
#  find_package(xmlrpcpp REQUIRED)
#
#  ament_add_gtest(${PROJECT_NAME}-test_dummy_interface
#    test/test_dummy_interface.cpp
#  )
#  target_link_libraries(${PROJECT_NAME}-test_dummy_interface
#    ${PROJECT_NAME}_string
#    ${console_bridge_LIBRARIES}
#    ${Boost_LIBRARIES}
#  )

#  ament_add_gtest(${PROJECT_NAME}-test_delegates
#    test/test_delegates.cpp
#  )
#  target_link_libraries(${PROJECT_NAME}-test_delegates
#    ${PROJECT_NAME}_string
#    ${Boost_LIBRARIES}
#  )

#  ament_add_gtest(${PROJECT_NAME}-test_settings
#    test/test_settings.cpp
#  )
#  target_include_directories(${PROJECT_NAME}-test_settings PRIVATE
#    ${xmlrpcpp_INCLUDE_DIRS}
#  )
#  target_link_libraries(${PROJECT_NAME}-test_settings
#    ${PROJECT_NAME}_string
#    ${xmlrpcpp_LIBRARIES}
#  )

#  ament_add_gtest(${PROJECT_NAME}-test_string
#    test/test_string.cpp
#  )
#  target_link_libraries(${PROJECT_NAME}-test_string
#    ${PROJECT_NAME}_string
#    ${console_bridge_LIBRARIES}
#    ${catkin_LIBRARIES}
#  )

#  ament_add_gtest(${PROJECT_NAME}-test_filter
#    test/test_filter.cpp
#  )
#  target_link_libraries(${PROJECT_NAME}-test_filter
#    ${PROJECT_NAME}_string
#    ${console_bridge_LIBRARIES}
#    ${Boost_LIBRARIES}
#  )

#  ament_add_gtest(${PROJECT_NAME}-test_dispatcher
#    test/test_dispatcher.cpp
#  )
#  target_link_libraries(${PROJECT_NAME}-test_dispatcher
#    ${PROJECT_NAME}_string
#    ${catkin_LIBRARIES}
#    ${Boost_LIBRARIES}
#  )
#  target_compile_options(${PROJECT_NAME}-test_dispatcher PRIVATE -Wno-deprecated-declarations)

#endif()

ament_export_libraries(${PROJECT_NAME}_string ${PROJECT_NAME}_plugin)
ament_export_dependencies(
                        ament_cmake
                        Boost 
                        console_bridge 
                        class_loader)
ament_export_include_directories(include)
ament_package()