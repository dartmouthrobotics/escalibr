cmake_minimum_required(VERSION 2.8.3)
project(escalibr)

add_compile_options(-std=c++11)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  sensor_msgs
  message_generation
  message_filters
  ping_nodelet
  cv_bridge
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
                 rospy
                 roslint
                 sensor_msgs
                 message_generation
                 message_filters
                 ping_nodelet
                 cv_bridge
                 opencv2
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(calibrate
  src/calibrate.cpp
  src/EchoSounderCameraHandler.cpp
)


add_library(${PROJECT_NAME}
  src/EchoSounderCameraHandler.cpp
)

target_link_libraries(calibrate
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
)


roslint_cpp(
  src/calibrate.cpp
  src/EchoSounderCameraHandler.cpp
  include/${PROJECT_NAME}/EchoSounderCameraHandler.h
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
