cmake_minimum_required(VERSION 3.14)
project(loco_planning_project)

add_compile_options(-std=c++17)


## DEPENDENCIES:
include(FetchContent)
FetchContent_Declare(
  Clipper2
  GIT_REPOSITORY https://github.com/AngusJohnson/Clipper2.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(Clipper2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  loco_planning     
  obstacles_msgs
  pluginlib
  roslib
)

# Find Eigen3 (required for Roadmap and Environment logic)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)


## BUILD SETTINGS:

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planner_lib
  CATKIN_DEPENDS roscpp geometry_msgs obstacles_msgs loco_planning
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${clipper2_SOURCE_DIR}/CPP/Clipper2Lib/include
)


## LIBRARY (The Planners):

# Helper for Clipper2 compilation
set(CLIPPER_SOURCES
  ${clipper2_SOURCE_DIR}/CPP/Clipper2Lib/src/clipper.engine.cpp
  ${clipper2_SOURCE_DIR}/CPP/Clipper2Lib/src/clipper.offset.cpp
  ${clipper2_SOURCE_DIR}/CPP/Clipper2Lib/src/clipper.rectclip.cpp
)

# ADD ALL NEW SOURCE FILES HERE
add_library(planner_lib
  # Environment logic
  src/environment/Obstacle.cpp
  src/environment/Roadmap.cpp
  src/environment/EnvironmentHandler.cpp
  
  # Orienteering logic
  src/orienteering/DistanceMatrix.cpp
  src/orienteering/Orienteering.cpp
  
  # Trajectory logic
  src/trajectory/Smoothing.cpp
  src/trajectory/DubinsTrajectory.cpp
  
  # Planner Logic
  src/planners/planner_base.cpp
  src/planners/planner_prm.cpp
  src/planners/planner_ecd.cpp
  src/planners/planner_acd.cpp

  ${CLIPPER_SOURCES}
)

target_link_libraries(planner_lib
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

add_dependencies(planner_lib 
  loco_planning_generate_messages_cpp 
  obstacles_msgs_generate_messages_cpp
)


## NODE (The Executable):

add_executable(planner_node src/planner_node.cpp)

target_link_libraries(planner_node
  planner_lib
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)


## INSTALL RULES:

install(TARGETS planner_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)