cmake_minimum_required(VERSION 3.0.2)
project(unity_basketdrone)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  sensor_msgs 
  geometry_msgs
  vision_msgs
  message_filters  
  tf2
  tf2_ros
  std_msgs
  compress_image
  image_geometry
  rosparam_shortcuts
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    vision_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(unity_camera src/unity_camera.cpp)
target_link_libraries(unity_camera ${catkin_LIBRARIES})
add_dependencies(unity_camera ${catkin_EXPORTED_TARGETS})

install(PROGRAMS
  script/drone_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  script/drone_controller_new.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  script/centroid_calculator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)