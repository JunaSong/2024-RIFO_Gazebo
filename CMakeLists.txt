cmake_minimum_required(VERSION 3.5)
project(rifo_gazebo2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# include_directories(
#     include
#     ${ament_INCLUDE_DIRS}
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )

add_executable(arm_command src/arm_command.cpp)
ament_target_dependencies(arm_command rclcpp std_msgs)

add_executable(arm_main src/arm_main.cpp)
ament_target_dependencies(arm_main rclcpp std_msgs)

add_library(arm_plugin src/arm_plugin.cpp)
target_link_libraries(arm_plugin ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  arm_command
  arm_main
  arm_plugin
  DESTINATION lib/${PROJECT_NAME}
)

#install the launch directory
install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}/
 )

ament_package()