add_compile_options(-std=c++11)
cmake_minimum_required(VERSION 2.8.3)
project(rifo_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES rifo_gazebo
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}   
    ./include
    ./src
)

add_executable(joint_control src/joint_control.cpp)
add_dependencies(joint_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_control ${catkin_LIBRARIES})

add_executable(rrbot_command src/arm_command.cpp)
add_dependencies(rrbot_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rrbot_command ${catkin_LIBRARIES})

add_executable(arm_main src/arm_main.cpp)
add_dependencies(arm_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_main ${catkin_LIBRARIES})