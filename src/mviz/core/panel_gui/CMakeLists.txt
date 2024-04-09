SET(MODULE_NAME panel_gui)

INCLUDE_DIRECTORIES(
  ${MVIZ_HOME}/core/panel_gui
)
LINK_DIRECTORIES()

SET(MODULE_HOME_PATH ${MVIZ_HOME}/core/panel_gui)

file(GLOB SRC_FILES "${MODULE_HOME_PATH}/*.cpp")

foreach(src ${SRC_FILES})
  get_filename_component(FILENAME ${src} NAME_WE)
  message(STATUS "File name: ${FILENAME}")
  SET(SRC_HEADER "${FILENAME}.h" )
  qt5_wrap_cpp(GUI_MOC_${FILENAME} ${MODULE_HOME_PATH}/${SRC_HEADER})

  add_library(rviz_${FILENAME}  "${FILENAME}.cpp" ${GUI_MOC_${FILENAME}})
  target_link_libraries(rviz_${FILENAME}
    ${QT_LIBRARIES} 
    ${catkin_LIBRARIES}
  )

endforeach()


SET(ENABLE_TEST OFF)
IF(ENABLE_TEST)
add_subdirectory(test)
ENDIF()
