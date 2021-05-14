cmake_minimum_required(VERSION 3.5)
project(canopen_402)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(canopen_master REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    chrono
    system
    thread
)

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})

# canopen_402
add_library(${PROJECT_NAME}
  src/motor.cpp
)
ament_target_dependencies(${PROJECT_NAME} canopen_master class_loader)
target_link_libraries(${PROJECT_NAME}
   ${Boost_LIBRARIES}
)

# canopen_402_plugin
add_library(${PROJECT_NAME}_plugin
  src/plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
   ${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(
  FILES
    ${PROJECT_NAME}_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# if(CATKIN_ENABLE_TESTING)
#   catkin_add_gtest(${PROJECT_NAME}-test_clamping
#     test/clamping.cpp
#   )
#   target_link_libraries(${PROJECT_NAME}-test_clamping
#     ${catkin_LIBRARIES}
#   )
# endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
                        ament_cmake
                        Boost
                        class_loader)
ament_export_include_directories(include)
ament_package()