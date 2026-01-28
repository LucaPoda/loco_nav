cmake_minimum_required(VERSION 3.14)
project(loco_planning_project)

add_compile_options(-std=c++17)

## ------------------------------------------------ ##
##            DEPENDENCIES                          ##
## ------------------------------------------------ ##
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
)

find_package(ompl REQUIRED)

## ------------------------------------------------ ##
##                 BUILD SETTINGS                   ##
## ------------------------------------------------ ##

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planner_lib
  CATKIN_DEPENDS roscpp geometry_msgs obstacles_msgs loco_planning
)

# Add Includes (Fixes "No such file" errors)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${clipper2_SOURCE_DIR}/CPP/Clipper2Lib/include
)

## ------------------------------------------------ ##
##            LIBRARY (The Planners)                ##
## ------------------------------------------------ ##

# DIRECTLY COMPILE CLIPPER SOURCES
# This fixes "cannot find -lClipper2" by building it inside your lib
set(CLIPPER_SOURCES
  ${clipper2_SOURCE_DIR}/CPP/Clipper2Lib/src/clipper.engine.cpp
  ${clipper2_SOURCE_DIR}/CPP/Clipper2Lib/src/clipper.offset.cpp
  ${clipper2_SOURCE_DIR}/CPP/Clipper2Lib/src/clipper.rectclip.cpp
)

add_library(planner_lib
  src/planners/planner_base.cpp
  src/planners/planner_prm.cpp
  src/planners/planner_combinatorial.cpp
  ${CLIPPER_SOURCES}  # <--- Compiling Clipper here!
)

target_link_libraries(planner_lib
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  # Clipper2 link removed (built-in now)
)

add_dependencies(planner_lib 
  loco_planning_generate_messages_cpp 
  obstacles_msgs_generate_messages_cpp
)

## ------------------------------------------------ ##
##            NODE (The Executable)                 ##
## ------------------------------------------------ ##

add_executable(planner_node src/planner_node.cpp)

target_link_libraries(planner_node
  planner_lib
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

## ------------------------------------------------ ##
##            INSTALL RULES                         ##
## ------------------------------------------------ ##

# 1. Install the Library (This is what you are missing!)
install(TARGETS planner_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 2. Install the Executable (You likely already added this)
install(TARGETS planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 3. Install Directories (Launch/Config)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)