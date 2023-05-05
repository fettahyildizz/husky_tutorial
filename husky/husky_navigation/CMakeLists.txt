cmake_minimum_required(VERSION 3.0.2)
project(husky_navigation)

find_package(catkin REQUIRED 
  COMPONENTS nodelet
  nodelet
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  )




include_directories(include ${catkin_INCLUDE_DIRS})
add_library(local_planner src/local_planner.cpp)
target_link_libraries(local_planner ${catkin_LIBRARIES})

add_library(global_planner_lib src/global_planner.cpp)
target_link_libraries(global_planner_lib ${catkin_LIBRARIES})


catkin_package(INCLUDE_DIRS include
  LIBRARIES local_planner global_planner_lib
  CATKIN_DEPENDS roscpp 
  rospy
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  
  )
#add_executable(local_planner )


if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(
  DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

