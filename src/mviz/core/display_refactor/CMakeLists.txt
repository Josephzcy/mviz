
SET(MODULE_NAME display_refactor)

MESSAGE(STATUS "SRC: ${MVIZ_HOME}")
SET(MODULE_HOME_PATH ${MVIZ_HOME}/core/display_refactor)

INCLUDE_DIRECTORIES(
  ${MODULE_HOME_PATH}
)
aux_source_directory(${MODULE_HOME_PATH} SRC_LISTS)

# list(APPEND SRC_LISTS
#   display_srv.hpp
#   ${MVIZ_HOME}/utils/draw_box/box3d_vertex.cpp
# )

SET(target_name mviz_${MODULE_NAME})
ADD_LIBRARY(${target_name} SHARED ${SRC_LISTS})

SET(ENABLE_TEST OFF)
IF(ENABLE_TEST)
  add_subdirectory(${SOURCE_LIST})
ENDIF()
