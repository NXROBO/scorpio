cmake_minimum_required(VERSION 2.8.3)
project(scorpio_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  scorpio_base
)


###################################
## catkin specific configuration ##
###################################=
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scorpio_test
#  CATKIN_DEPENDS roscpp rospy std_msgs scorpio_base
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(scorpio_turn_left_right_node src/scorpio_turn_left_right.cpp)
add_dependencies(scorpio_turn_left_right_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(scorpio_turn_left_right_node ${catkin_LIBRARIES})

add_executable(scorpio_test_motion_node src/scorpio_test_motion.cpp)
add_dependencies(scorpio_test_motion_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(scorpio_test_motion_node ${catkin_LIBRARIES})


add_executable(scorpio_turn_left_right_st_node src/scorpio_turn_left_right_st.cpp)
add_dependencies(scorpio_turn_left_right_st_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(scorpio_turn_left_right_st_node ${catkin_LIBRARIES})


