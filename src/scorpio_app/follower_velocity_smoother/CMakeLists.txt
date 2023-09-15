cmake_minimum_required(VERSION 2.8.3)
project(follower_velocity_smoother)
find_package(catkin REQUIRED COMPONENTS roscpp pluginlib nodelet ecl_threads dynamic_reconfigure geometry_msgs nav_msgs)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/params.cfg)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}_nodelet
   CATKIN_DEPENDS roscpp pluginlib nodelet ecl_threads dynamic_reconfigure geometry_msgs nav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########
# Nodelet library
add_library(${PROJECT_NAME}_nodelet src/velocity_smoother_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencfg)


add_executable(velocity_smoother_node src/velocity_smoother_node.cpp)
target_link_libraries(velocity_smoother_node
  ${catkin_LIBRARIES}
)
add_dependencies(velocity_smoother_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)
#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

