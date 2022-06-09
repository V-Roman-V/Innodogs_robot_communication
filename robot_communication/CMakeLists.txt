cmake_minimum_required(VERSION 3.0.2)
project(robot_communication)

# add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
)

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib)

set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)
set(CMAKE_CXX_FLAGS "-O3")


add_executable(robot_communication "src/robot_communication.cpp" "src/connection_to_robot.cpp")
target_link_libraries(robot_communication ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(robot_communication ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

