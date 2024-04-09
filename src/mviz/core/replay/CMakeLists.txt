SET(MODULE_NAME mviz_replay)

SET(REPLAY_HOME_PATH  ${MVIZ_HOME}/core/replay)
SET(UTILS_HOME_PATH ${MVIZ_HOME}/utils)
INCLUDE_DIRECTORIES(
  ${REPLAY_HOME_PATH}
)
LINK_DIRECTORIES()

qt5_wrap_cpp(REPALY_MOC_FILES ${REPLAY_HOME_PATH}/replay_gui.h)
qt5_wrap_cpp(REPALY_MOC_FILES1 ${REPLAY_HOME_PATH}/replay_ui.h)
qt5_wrap_cpp(REPALY_MOC_FILES2 ${REPLAY_HOME_PATH}/config_editor.h)
qt5_wrap_cpp(REPALY_MOC_FILES3 ${REPLAY_HOME_PATH}/log_view_ui.h)
qt5_wrap_cpp(REPALY_MOC_FILES4 ${REPLAY_HOME_PATH}/mviz_tool_box.h)
SET(replay_ui_src 
  ${REPLAY_HOME_PATH}/replay_ui.cpp 
  ${REPLAY_HOME_PATH}/clickable_slider.cpp 
  ${REPLAY_HOME_PATH}/config_editor.cpp 
  ${REPLAY_HOME_PATH}/log_view_ui.cpp 
  ${REPLAY_HOME_PATH}/configuration_manager.cpp 
  ${REPLAY_HOME_PATH}/global_value.cpp 
  ${REPLAY_HOME_PATH}/frames_pub_manager.cpp 
  ${REPLAY_HOME_PATH}/l2data_reader.cpp 
  ${REPLAY_HOME_PATH}/pystring.cpp 
  ${REPLAY_HOME_PATH}/reader_manager.cpp 
  ${REPLAY_HOME_PATH}/video_stream_decoder.cpp 
  ${MVIZ_HOME}/core/display/manager_marker.cpp 
  ${MVIZ_HOME}/core/display/generate_car_model.cpp 
  ${UTILS_HOME_PATH}/draw_box/box3d_vertex.cpp
  ${REPALY_MOC_FILES1}
  ${REPALY_MOC_FILES2}
  ${REPALY_MOC_FILES3}
)

set(odom2pix_src
  ${UTILS_HOME_PATH}/bev_odom2pix/bev_pix2odom.cpp
  ${UTILS_HOME_PATH}/pose/pose.cpp
  ${UTILS_HOME_PATH}/rigid_transformer/rigid_transformer.cpp
)

add_library(mviz_tool_box 
  ${REPLAY_HOME_PATH}/mviz_tool_box.cpp 
  ${replay_ui_src}
  ${REPALY_MOC_FILES4}
  ${odom2pix_src}
)

target_link_libraries(
  mviz_tool_box 
  ${QT_LIBRARIES} 
  pthread
  yaml-cpp
  protobuf
  avcodec avformat avutil swresample swscale swscale avfilter
  ${catkin_LIBRARIES}
)
    
add_library(rviz_replay_gui
  ${REPLAY_HOME_PATH}/replay_gui.cpp 
  ${replay_ui_src}
  ${REPALY_MOC_FILES}
  ${odom2pix_src}
)
target_link_libraries(
  rviz_replay_gui 
  ${QT_LIBRARIES} 
  pb_msg
  pthread
  yaml-cpp
  protobuf
  avcodec avformat avutil swresample swscale swscale avfilter
  ${catkin_LIBRARIES}
)


SET(ENABLE_TEST OFF)
IF(ENABLE_TEST)
add_subdirectory(test)
ENDIF()
