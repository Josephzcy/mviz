add_executable(dt_test
  ${MVIZ_HOME}/core/replay/dt/dt_reader_manager.cpp 
  ${MVIZ_HOME}/core/replay/configuration_manager.cpp 
  ${MVIZ_HOME}/core/replay/global_value.cpp 
  ${MVIZ_HOME}/core/replay/l2data_reader.cpp 
  ${MVIZ_HOME}/core/replay/pystring.cpp 
  ${MVIZ_HOME}/core/replay/reader_manager.cpp 
  ${MVIZ_HOME}/core/replay/video_stream_decoder.cpp 
)
target_link_libraries(
  dt_test 
  ${OpenCV_LIBRARIES}
  ${QT_LIBRARIES} 
  pthread
  yaml-cpp
  protobuf
  avcodec avformat avutil swresample swscale swscale avfilter
  ${catkin_LIBRARIES})

add_executable(dt_pystring
  ${MVIZ_HOME}/core/replay/dt/dt_pystring.cpp 
  ${MVIZ_HOME}/core/replay/pystring.cpp 
)
target_link_libraries(
  dt_pystring 
  ${catkin_LIBRARIES})