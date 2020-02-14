cmake_minimum_required(VERSION 2.8.3)
project(escalibr)

add_compile_options(-std=c++11)
add_compile_options(-pg)

set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system)
find_package(yaml-cpp REQUIRED)

#add_subdirectory(ORB_SLAM2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  sensor_msgs
  geometry_msgs
  message_generation
  message_filters
  ping_nodelet
  cv_bridge
  genmsg
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
                 rospy
                 roslint
                 sensor_msgs
                 geometry_msgs
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
  ${YAML_CPP_INCLUDE_DIR}
  /home/darobot/orb_slam/ORB_SLAM2/msg_gen/cpp/include
)


add_executable(calibrate
  src/calibrate.cpp
  src/echo_sounder_camera_handler.cpp
)

add_executable(extract
  src/extract.cpp
  src/echo_sounder_camera_extractor.cpp
)

add_library(${PROJECT_NAME}
  src/echo_sounder_camera_handler.cpp
  src/echo_sounder_camera_extractor.cpp
  src/calibration_math.cpp
  src/calibration_gui.cpp
  src/image_proc.cpp
)

target_link_libraries(calibrate
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(extract
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${YAML_CPP_LIBRARIES}
)

add_dependencies(extract escalibr_generate_messages_cpp)


roslint_cpp(
  src/calibrate.cpp
  src/echo_sounder_camera_handler.cpp
  include/${PROJECT_NAME}/echo_sounder_camera_handler.h
  src/calibration_math.cpp
  include/${PROJECT_NAME}/calibration_math.h
  src/calibration_gui.cpp
  include/${PROJECT_NAME}/calibration_gui.h
  src/image_proc.cpp
  include/${PROJECT_NAME}/image_proc.h
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
