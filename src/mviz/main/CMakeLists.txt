SET(MODULE_NAME mviz)
SET(MODULE_NAME_PATH ${MVIZ_HOME}/main)
MESSAGE(STATUS "SRC: ${MVIZ_HOME}")

set(CMAKE_CXX_STANDARD 17)

ADD_EXECUTABLE(${MODULE_NAME}_node
  ${MODULE_NAME_PATH}/main.cpp
  ${MODULE_NAME_PATH}/${MODULE_NAME}_inf.cpp
  # ${MVIZ_HOME}/utils/performance_analyzer/performance_analyzer.cpp
)


target_link_libraries(${MODULE_NAME}_node
  mviz_collect
  mviz_display
)
SET(ENABLE_TEST OFF)
IF(ENABLE_TEST)
add_subdirectory(test)
ENDIF()
