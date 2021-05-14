cmake_minimum_required(VERSION 3.5)
project(canopen_master)

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
find_package(socketcan_interface REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    atomic
    chrono
    thread
)

include_directories(${socketcan_interface_INCLUDE_DIRS})
include_directories(include ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/emcy.cpp
  src/node.cpp
  src/objdict.cpp
  src/pdo.cpp
  src/sdo.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}_plugin
  src/master_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
)

# canopen_bcm_sync
add_executable(canopen_bcm_sync
  src/bcm_sync.cpp
)
ament_target_dependencies(canopen_bcm_sync
  socketcan_interface)
target_link_libraries(canopen_bcm_sync
  ${Boost_LIBRARIES}
)

install(
  TARGETS
    canopen_bcm_sync
    ${PROJECT_NAME}
   ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

install(
  FILES
    master_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

#if(BUILD_TESTING)
#  ament_add_gtest(${PROJECT_NAME}-test_parser
#    test/test_parser.cpp
#  )
#  target_link_libraries(${PROJECT_NAME}-test_parser
#    ${PROJECT_NAME}
#  )
#
#  ament_add_gtest(${PROJECT_NAME}-test_node
#    test/test_node.cpp
#  )
#  target_link_libraries(${PROJECT_NAME}-test_node
#    ${PROJECT_NAME}
#  )
#endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(socketcan_interface Boost class_loader)
ament_export_include_directories(include)

ament_package()