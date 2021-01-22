
cmake_minimum_required(VERSION 2.8.3)
project(ros_fiducial_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
  fiducial_msgs
  visualization_msgs
  message_generation
  image_transport
  cv_bridge
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/DetectorParams.cfg)

add_service_files(
  FILES
  resetOrigin.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(INCLUDE_DIRS DEPENDS OpenCV)

###########
## Build ##
###########

add_definitions(-std=c++17)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(aruco_detect src/aruco_detect.cpp)

add_dependencies(aruco_detect ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(aruco_detect ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(fiducial_tracker src/fiducial_tracker.cpp)

add_dependencies(fiducial_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(fiducial_tracker ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS fiducial_tracker
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS aruco_detect
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/create_markers.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
