mviz_upstream_tools_path="/data/mviz_upstream_tools" 
if [ ! -n "$1" ];then
    echo "Parm Doesn't exist,please input run mode"
    exit 1
fi

run_mode=$1
if [ "${run_mode}" = "hil" ] ;then
  dds2flow_pid=`ps -aux | grep "dds2flow" |grep -v "grep"|awk '{print $2}'`
  if [ ! -n "$dds2flow_pid" ]; then 
    cd ${mviz_upstream_tools_path}/scripts/
    ./run_dds2flow.sh  >/dev/null 2>&1 &
    sleep 0.1
    echo "now ddsflow has been executed."
  else
    echo "[ddsflow] is running."
  fi

  SIM_TDA4_V3_pid=`ps -aux | grep "sim_tda4_V3" |grep -v "grep"|awk '{print $2}'`
  if [ ! -n "$SIM_TDA4_V3_pid" ]; then 
    cd ${mviz_upstream_tools_path}/scripts/
    ./run_sim.sh  >/dev/null 2>&1 &
    sleep 0.1
    echo "now sim_tda4_V3 has been executed."
   else
    echo "[sim_tda4_V3] is running."
  fi


  bev_pid=`ps -aux | grep "bev_libflow.out" |grep -v "grep"|awk '{print $2}'`
  if [ ! -n "$bev_pid" ]; then 
    cd ${mviz_upstream_tools_path}/scripts/
    ./run_bev2libflow.sh  >/dev/null 2>&1 &
    sleep 0.1
    echo "now bev_libflow has been executed."
  else
    echo "[bev_libflow] is running."
  fi
  
  sleep 0.1

elif [ "${run_mode}" = "oncar" ];then
  # source /app/env.sh
  # avm_test_pid=`ps -aux | grep "avm_test" |grep -v "grep"|awk '{print $2}'`
  # if [ -n "$avm_test_pid" ]; then 
  #   # kill -9 $avm_test_pid
  #   # echo "avm_test has been killed."
  #   echo "avm has been running"
  # else
  #   echo "Avm_Test_Pid IS NULL"
  # fi  
  # sleep 0.1

  # start app_camera_encode_libflow.sh
  camera_encode_pid=`ps -aux | grep "app_camera_encode_libflow.out" |grep -v "grep"|awk '{print $2}'`
  if [ ! -n "$camera_encode_pid" ]; then 
    source /app/env.sh
    cd /app/base/camera/
    ./bin/app_camera_encode_libflow.out camera30  >/dev/null 2>&1 &
    sleep 0.1
    echo "now camera_encode has been executed."
  else
    echo "[ camera_encode ] is running."
  fi
 

  # start bev_libflow
  bev_pid=`ps -aux | grep "bev_libflow.out" |grep -v "grep"|awk '{print $2}'`
  if [ ! -n "$bev_pid" ]; then 
    cd ${mviz_upstream_tools_path}/scripts/
    ./run_bev2libflow.sh  >/dev/null 2>&1 &
    sleep 0.1
    echo "now bev_libflow has been executed."
  else
    echo "[ bev_libflow ] is running."
  fi
  

  # start dds2flow.sh
  dds2flow_pid=`ps -aux | grep "dds2flow" |grep -v "grep"|awk '{print $2}'`
  if [ ! -n "$dds2flow_pid" ]; then
    cd ${mviz_upstream_tools_path}/scripts/ 
    ./run_dds2flow.sh  >/dev/null 2>&1 &
    sleep 0.1
    echo "now ddsflow has been executed."
  else
    echo "[ ddsflow ] is running."
  fi


  # start system_res_monitor
  system_res_monitor_pid=`ps -aux | grep "system_res_monitor" |grep -v "grep"|awk '{print $2}'`
  if [ ! -n "$system_res_monitor_pid" ]; then 
    cd /app/misc/system_res_monitor/ && ./system_res_monitor >/dev/null 2>&1 &
    sleep 0.1
    echo "now system_res_monitor has been executed."
  else
    echo "[ system_res_monitor ] is running."
  fi
  

  echo "start_mviz_upstream_components_board has been executed."
  exit 1

fi
