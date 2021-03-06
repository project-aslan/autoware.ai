cmake_minimum_required(VERSION 2.8.3)
project(astar_planner)

find_package(aslan_msgs REQUIRED)
find_package(aslan_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  tf
  waypoint_follower
  aslan_msgs 
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS nodes/velocity_set include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs tf waypoint_follower aslan_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} nodes/velocity_set/velocity_set_path.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(velocity_set nodes/velocity_set/velocity_set_main.cpp nodes/velocity_set/velocity_set.cpp nodes/velocity_set/velocity_set_path.cpp nodes/velocity_set/velocity_set_info.cpp nodes/velocity_set/libvelocity_set.cpp)
target_link_libraries(velocity_set ${catkin_LIBRARIES})
add_dependencies(velocity_set 
${catkin_EXPORTED_TARGETS})

add_executable(obstacle_avoid nodes/obstacle_avoid/obstacle_avoid_main.cpp nodes/obstacle_avoid/obstacle_avoid.cpp nodes/obstacle_avoid/astar_search.cpp nodes/obstacle_avoid/search_info_ros.cpp nodes/obstacle_avoid/astar_util.cpp)
target_link_libraries(obstacle_avoid ${catkin_LIBRARIES})
add_dependencies(obstacle_avoid ${catkin_EXPORTED_TARGETS})

add_executable(obstacle_sim nodes/obstacle_avoid/obstacle_sim/obstacle_sim.cpp nodes/obstacle_avoid/obstacle_sim/obstacle_sim_node.cpp)
target_link_libraries(obstacle_sim ${catkin_LIBRARIES})
add_dependencies(obstacle_sim ${catkin_EXPORTED_TARGETS})
