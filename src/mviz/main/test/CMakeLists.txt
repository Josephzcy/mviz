SET(MODULE_NAME mviz_test)
SET(MODULE_NAME_PATH ${MVIZ_HOME}/main)

include_directories(${MODULE_NAME_PATH})
ADD_EXECUTABLE(${MODULE_NAME}_node
  ${MODULE_NAME_PATH}/test/main_test.cpp
  # ${MODULE_NAME_PATH}/mviz_inf.cpp
  # ${MVIZ_HOME}/utils/performance_analyzer/performance_analyzer.cpp
  ${MVIZ_HOME}/utils/bind_thread_to_cpu/bind_thread_to_cpu.cpp

)

target_link_libraries(${MODULE_NAME}_node

  # profiler 
  # tcmalloc
  # unwind
  pthread
)

