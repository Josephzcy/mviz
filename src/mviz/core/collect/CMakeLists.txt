
SET(MODULE_NAME mviz_collect)
SET(COLLECT_HOME_PATH ${MVIZ_HOME}/core/collect)

SET(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

INCLUDE_DIRECTORIES(${COLLECT_HOME_PATH})
LINK_DIRECTORIES()


aux_source_directory(${COLLECT_HOME_PATH} SOURCE_LIST)
FILE(GLOB SOURCE_LIST "${COLLECT_HOME_PATH}/*.cpp" "${COLLECT_HOME_PATH}/*.hpp")


SET(BUILD_SHARED_LIBS ON)
IF (BUILD_SHARED_LIBS)
  ADD_LIBRARY(${MODULE_NAME} SHARED ${SOURCE_LIST} )
ELSE ()
  ADD_LIBRARY(${MODULE_NAME} STATIC ${SOURCE_LIST} )
ENDIF ()

target_link_libraries(${MODULE_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  pb_msg
  pthread
  flow
  glog
  gflags
  ${Boost_LIBRARIES}
  protobuf
  decode_video
  yaml-cpp
  avcodec avformat avutil swresample swscale swscale avfilter
  nanomsg
)

SET(ENABLE_TEST OFF)
IF(ENABLE_TEST)
  ADD_SUBDIRECTORY(test)
ENDIF()