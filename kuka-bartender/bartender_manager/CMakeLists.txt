cmake_minimum_required(VERSION 2.8.3)
project(bartender_manager)

add_definitions(-std=gnu++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  kdl_parser
  dynamic_reconfigure
  bartender_control
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/bartender.cfg
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs dynamic_reconfigure bartender_control
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_executable(bartender_manager src/main.cpp src/bartender_manager.cpp)
target_link_libraries(bartender_manager ${catkin_LIBRARIES})
add_dependencies(bartender_manager bartender_manager_gencfg bartender_manager_gencpp)
