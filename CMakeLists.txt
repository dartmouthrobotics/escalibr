cmake_minimum_required(VERSION 2.8.3)
project(escalibr)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
                 rospy
                 roslint
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)


add_executable(super_coarse_calibr
  src/Options/super_coarse.cpp
)


#add_library(${PROJECT_NAME}
#)

target_link_libraries(super_coarse_calibr
  #${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


roslint_cpp(
  src/Options/super_coarse.cpp
)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
