cmake_minimum_required(VERSION 2.8.3)
project(scorpio_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  CarPwm.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy std_msgs tf
 # DEPENDS system_lib
)

include_directories(
  
  ${catkin_INCLUDE_DIRS}
  ../cereal_port/include
)

set(scorpio_base_node_SRCS
  src/scorpio_base.cpp
)

###########
## Build ##
###########
## Declare a C++ executable

add_executable(scorpio_base_node ${scorpio_base_node_SRCS})
target_link_libraries(scorpio_base_node
  ${catkin_LIBRARIES}
  cereal_port
)

add_executable(cmd_vel_to_ackermann_cmd_node src/cmd_vel_to_ackermann_cmd.cpp)
target_link_libraries(cmd_vel_to_ackermann_cmd_node
  ${catkin_LIBRARIES}
)



add_dependencies(scorpio_base_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)
add_dependencies(cmd_vel_to_ackermann_cmd_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)


#############
## Install ##
#############

install(TARGETS scorpio_base_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS cmd_vel_to_ackermann_cmd_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
 
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
