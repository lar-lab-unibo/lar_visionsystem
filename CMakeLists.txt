cmake_minimum_required(VERSION 2.8.3)
project(lar_visionsystem)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf
  message_generation
)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake/Modules/")
set(CMAKE_INCLUDE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/")

#OpenCV
find_package(OpenCV REQUIRED)
if (OPENCV_FOUND)
  set(HAVE_OPENCV ON)
  include_directories(SYSTEM ${OPENCV_INCLUDE_DIRS})
  message("OPENCV INCLUDE DIR:")
  message(${OpenCV_LIBS})
endif(OPENCV_FOUND)

#Aruco
set(ARUCO_LIBS /usr/local/lib/libaruco.so)

#VisionSystemLegacy
#set(VISION_SYSTEM_LIBS /usr/local/lib/libVisionSystemLegacy.so)


#PCL
#find_package(PCL 1.8 REQUIRED )
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})


#SOPHUS
#set(SOPHUS_INCLUDE_DIRS /usr/local/include/sophus)
#include_directories(${SOPHUS_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#    MarkerImageService.srv
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lar_visionsystem
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_INCLUDE_PATH}
)

MESSAGE(CIAO)
MESSAGE(${CMAKE_INCLUDE_PATH})

#add_executable(vision_test src/vision_test.cpp)
#
# target_link_libraries(vision_test
#  ${catkin_LIBRARIES}
#
#  ${OpenCV_LIBS}
#  ${PCL_LIBRARY_DIRS}
#  ${ARUCO_LIBS}
#  ${VISION_SYSTEM_LIBS}
#  /usr/local/lib/libpcl_visualization.so.1.8.0
#  /usr/local/lib/libpcl_common.so.1.8
#  /usr/lib/libvtkCommon.so.5.8
#  /usr/lib/libvtkFiltering.so.5.8
#  /usr/lib/libvtkRendering.so.5.8
#)
add_library(lar_visionsystem_commons
        src/utils/MathUtils.cpp
    )
target_link_libraries (lar_visionsystem_commons ${OpenCV_LIBS} ${ARUCO_LIBS})

add_executable(camera_viewer src/camera_viewer.cpp)
target_link_libraries(camera_viewer ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(marker_detector src/marker_detector.cpp)
target_link_libraries(marker_detector ${catkin_LIBRARIES} ${OpenCV_LIBS} ${ARUCO_LIBS} lar_visionsystem_commons)