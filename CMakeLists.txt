cmake_minimum_required(VERSION 3.5)
project(rifo_gazebo2)

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
# find_package(message_generation REQUIRED)

include_directories(
    include
    ${ament_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(rrbot_command src/arm_command.cpp)
ament_target_dependencies(rrbot_command rclcpp std_msgs)
target_link_libraries(rrbot_command ${ament_LIBRARIES})

add_executable(arm_main src/arm_main.cpp)
ament_target_dependencies(arm_main rclcpp std_msgs)
target_link_libraries(arm_main ${ament_LIBRARIES})

install(TARGETS
  rrbot_command
  arm_main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()