
cmake_minimum_required(VERSION 2.8.3)
project(ros_fiducial_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf
  fiducial_msgs
  visualization_msgs
  message_generation
)


add_service_files(
  FILES
  resetOrigin.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(INCLUDE_DIRS)

###########
## Build ##
###########

add_definitions(-std=c++17)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(fiducial_tracker src/fiducial_tracker.cpp)

add_dependencies(fiducial_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(fiducial_tracker ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS fiducial_tracker
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
