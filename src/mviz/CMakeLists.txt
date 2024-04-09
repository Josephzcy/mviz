CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(mviz)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -DNDEBUG")
ENDIF ()

# 设置C++编译器选项
IF (CMAKE_BUILD_TYPE MATCHES "(Debug|debug|DEBUG)")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -fsanitize=address")
    set(CMAKE_DEBUG_POSTFIX "d")
ENDIF ()

MESSAGE(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")     # mviz2_ros/src
MESSAGE(STATUS "PROJECT_BINARY_DIR: ${PROJECT_BINARY_DIR}") # mviz2_ros/build/mviz
MESSAGE(STATUS "CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")     # mviz2_ros/build

SET(USE_PERSON_PATH TRUE)
SET(FOR_AIPLORER TRUE)
#==== Will be according Aiplorer to set env ===
IF (USE_PERSON_PATH)
  SET(AIPLORE_ROOT ${CMAKE_SOURCE_DIR}/../../aiplorer) 
  SET(BUILD_SCRIPT_ROOT ${AIPLORE_ROOT}/scripts/cmakebuild)
  FILE(COPY ${CMAKE_CURRENT_SOURCE_DIR}/aiplorer.deps/deps.cmake DESTINATION ${CMAKE_BINARY_DIR}/)
ENDIF()

MESSAGE(STATUS "AIPLORE_ROOT: ${AIPLORE_ROOT}")
INCLUDE(${BUILD_SCRIPT_ROOT}/cmake/common.cmake)

SET(MVIZ_HOME ${CMAKE_SOURCE_DIR}/mviz)
SET(MVIZ_WORKSPACE_HOME ${CMAKE_SOURCE_DIR}/../)
MESSAGE(STATUS "MVIZ_HOME: ${MVIZ_HOME}")
MESSAGE(STATUS "MVIZ_WORKSPACE_HOME: ${MVIZ_WORKSPACE_HOME}")



find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rosmsg
  rospy
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
SET(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

SET(PB_PATH "${AIPLORE_ROOT}/msg/precompiler")
SET(Third_Part_DIR ${CMAKE_SOURCE_DIR}/../3rdParty)
SET(Mviz_Msg_DIR ${CMAKE_SOURCE_DIR}/../msg/rosmsgcomplier)
SET(FFMPEG_LIBS_DIR /usr/local/ffmpeg/lib)
SET(FFMPEG_INCLUDE_DIRS /usr/local/ffmpeg/include)
SET(AIPLORE_Pb_Msg_DIR ${AIPLORE_ROOT}/msg/precompiler)


# cmake -DMVIZ_J3=ON ..
option(MVIZ_J3 "MVIZ_J3 Use Protobuf2" OFF)
option(MVIZ_TDA4 "MVIZ_TDA4 Use Protobuf3" ON)

# J3需要使用protobuf2版本(建议aiplorer切到对应分支)
IF(MVIZ_J3)
  add_definitions(-DMVIZ_J3)
  MESSAGE(STATUS "MVIZ_J3 Using Protobuf2")
  SET(PROTOBUF_INCLUDE_PATH ${AIPLORE_ROOT}/third_party/linux/protobuf_2.6.1/include)
  SET(PROTOBUF_LIB_PATH ${AIPLORE_ROOT}/third_party/linux/protobuf_2.6.1/lib)
# tda4使用protobuf3版本(建议aiplorer切到对应分支)
ELSEIF(MVIZ_TDA4)
  add_definitions(-DMVIZ_TDA4)
  message(STATUS "MVIZ_TDA4 Using Protobuf3")
  SET(PROTOBUF_INCLUDE_PATH ${AIPLORE_ROOT}/third_party/linux/protobuf_3.12.3/include)
  SET(PROTOBUF_LIB_PATH ${AIPLORE_ROOT}/third_party/linux/protobuf_3.12.3/lib)
ENDIF()


###########
## Build ##
###########
include_directories(
  ${MVIZ_HOME}/include
  ${MVIZ_HOME}/core/
  ${MVIZ_HOME}/utils
  ${MVIZ_HOME}/utils/bev_odom2pix
  ${MVIZ_HOME}/common/platform
  ${MVIZ_HOME}/tools
  ${MVIZ_WORKSPACE_HOME}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${FFMPEG_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Third_Part_DIR}/decode_video/include/
  ${Mviz_Msg_DIR}
  ${PROTOBUF_INCLUDE_PATH}
  ${AIPLORE_Pb_Msg_DIR}
)

link_directories(
  ${Third_Part_DIR}/decode_video/lib/
  ${FFMPEG_LIBS_DIR}
  ${PROTOBUF_LIB_PATH}
)


# unsed
SET(GLOBAL_LINK_LIBS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  pthread
  flow
  glog
  ${Boost_LIBRARIES}
  protobuf
  decode_video
  yaml-cpp
  avcodec avformat avutil swresample swscale swscale avfilter
  nanomsg
)


# TODO::rosMsg ，生成对应的.c和.h 文件


SET(GenerateRosMsg OFF)
IF(GenerateRosMsg)
  FILE(GLOB_RECURSE MSG_FILES  ${MVIZ_WORKSPACE_HOME}/msg/rosMsg/*.msg)
  foreach(MSG ${MSG_FILES})
    get_filename_component(FILE_NAME ${MSG} NAME)
    MESSAGE(STATUS "File name: ${FILE_NAME}")
    list(APPEND ALL_MSG ${FILE_NAME})
  endforeach()
  add_message_files(
    DIRECTORY ${MVIZ_WORKSPACE_HOME}/msg/rosMsg
    FILES
    ${ALL_MSG}
  )

  generate_messages(
    DEPENDENCIES
    geometry_msgs   sensor_msgs
  )
ENDIF()

# todo:Mviz_Msg is *.h,include implement 
aux_source_directory(${Mviz_Msg_DIR}/mviz_apa_show Mviz_Msg_SRC)
aux_source_directory(${AIPLORE_Pb_Msg_DIR} AIPLORE_PROTO_CPP)
list(REMOVE_ITEM AIPLORE_PROTO_CPP ${AIPLORE_Pb_Msg_DIR}/gnss.pb.cc)


catkin_package(
 CATKIN_DEPENDS cv_bridge image_transport roscpp rosmsg rospy sensor_msgs std_msgs tf #libqt5-dev qt_build
)

# ros_msg_src and pb_msg_src compile library
ADD_LIBRARY(pb_msg SHARED ${AIPLORE_PROTO_CPP} )

ADD_SUBDIRECTORY(core)
ADD_SUBDIRECTORY(main)


# if(GenerateRosMsg)
#   add_dependencies(apa_data_pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
# endif()

