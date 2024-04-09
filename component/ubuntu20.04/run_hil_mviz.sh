#! /usr/bin/env bash 
export LD_LIBRARY_PATH=lib:$LD_LIBRARY_PATH
chmod +x ./bin/hil_for_mviz
./bin/hil_for_mviz --task_file=./config/hil_task_mviz.json --sim_ip=tcp://192.168.98.233:32000 --send_speed_scale 1 --enable_libflow=1 --g_order_key=tick --use_mviz_flow_msg true
