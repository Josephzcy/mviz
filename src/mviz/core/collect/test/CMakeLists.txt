
SET(MODULE_NAME collect_test)
SET(MODULE_HOME_PATH  ${MVIZ_HOME}/core/collect/test)

SET(GLOBAL_LINK_LIBS 
  nanomsg
  glog
)

# add_executable(collect_test ${MODULE_HOME_PATH}/record_test.cpp)

add_executable(nanomsg_send_test ${MODULE_HOME_PATH}/nanomsg_send.cpp)

target_link_libraries(nanomsg_send_test
  ${GLOBAL_LINK_LIBS}
)



