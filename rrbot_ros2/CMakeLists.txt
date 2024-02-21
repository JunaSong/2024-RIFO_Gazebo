cmake_minimum_required(VERSION 3.5)
project(rrbot_ros2)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(rrbot_ros2_plugin SHARED src/rrbot_ros2_plugin.cpp)
target_link_libraries(rrbot_ros2_plugin ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
ament_target_dependencies(rrbot_ros2_plugin rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(rrbot2_main src/rrbot2_main.cpp)
ament_target_dependencies(rrbot2_main rclcpp std_msgs)

add_executable(rrbot2_command src/rrbot2_command.cpp)
ament_target_dependencies(rrbot2_command rclcpp std_msgs)


install(TARGETS
  rrbot_ros2_plugin
  rrbot2_main
  rrbot2_command
  DESTINATION lib/${PROJECT_NAME}
)

#install the launch directory
install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}/
 )

ament_package()
