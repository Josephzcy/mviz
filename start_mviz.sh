#!/bin/bash
RED_COLOR='\E[1;31m'
GREEN_COLOR='\E[1;32m'
NC='\e[0m'

if [ $# -ne 2 ] ;then
  echo -e "${GREEN_COLOR}Usage: $0 <mode> <car_name>  eg: start_mviz oncar x50 ${NC}"
  echo -e "${RED_COLOR} Wrong params, please see the eg above ${NC}"
  exit 1
fi
echo -e "${GREEN_COLOR}Usage: mode:$1 car_type:$2  ${NC}"

run_mode=$1;body_type=$2
xhost + 
container_names="mviz_dev" 
container_id=$(docker ps -a |grep ${container_names} -w | awk '{print $1}')
echo ${container_id}
workspaceDir=$(pwd);echo "start_mviz.sh workspace_dir: ${workspaceDir}"

if [[ -n $(docker ps -q -f "name=${container_names}") ]];then
      echo "${container_names} has running"
else
    echo "${container_names} is down!"
    docker start ${container_id}
fi

docker_exec_cmd="docker exec -it ${container_id} /bin/bash -c"
mviz_start_cmd="cd /root/mviz2_ros/ && ./scripts/docker_script/start_mviz.sh"


# 定义函数

hil_board_func(){
  ssh -i debug.key root@192.168.98.233 /data/mviz_upstream_tools/start_mviz_upstream_components_board.sh hil
  echo -e "${GREEN_COLOR}Usage: ${docker_exec_cmd} "${mviz_start_cmd} $1 $2"  ${NC}"
  ${docker_exec_cmd} "${mviz_start_cmd} $1 $2"
}

hil_pc_func(){
  echo -e "${GREEN_COLOR}Usage: ${docker_exec_cmd} "${mviz_start_cmd} $1 $2"  ${NC}"
  ${docker_exec_cmd} "${mviz_start_cmd} $1 $2"
}

oncar_func(){
  #todo: start screen_recording 
  screen_recordings_dir="./screen_recordings/"
  if [ ! -d ${screen_recordings_dir}  ];then
    mkdir -p ${screen_recordings_dir} 
  fi

  ffmpeg -f x11grab -r 25 -s $(xdpyinfo | grep dimensions | awk '{print $2;}') -i :0.0 -vcodec libx264 -preset ultrafast "${screen_recordings_dir}/$(date +%Y%m%d%H%M%S).mp4" >/dev/null 2>&1 &
  REC_PID=$!

  # execute remoteOrders.py
  cd ${workspaceDir}/extensions/Ubuntu20.04/scripts/
  sudo python3 remoteOrders.py
  echo "remoteOrders.py has been executed."

  # start docker
  cd ${workspaceDir}
  echo -e "${GREEN_COLOR}Usage: ${docker_exec_cmd} "${mviz_start_cmd} $1 $2"  ${NC}"
  ${docker_exec_cmd} "${mviz_start_cmd} $1 $2"  echo "mviz has alreadr endded."

  kill -INT ${REC_PID}
  sleep 1

  # execute fileCopy.py
  cd ${workspaceDir}/extensions/Ubuntu20.04/scripts/
  sudo python3 fileCopy.py  
  echo "fileCopy.py has been executed."

}

replay_func(){
  echo "[fun_body_type] run_mode: $1 car_type: $2 "
  echo -e "${GREEN_COLOR}Usage: ${docker_exec_cmd} "${mviz_start_cmd} $1 $2"  ${NC}"
  ${docker_exec_cmd} "${mviz_start_cmd} $1 $2"
}


# 声明关联数组,函数名当做字符串处理
declare -A FuncArray

FuncArray["hil_board"]="hil_board_func"
FuncArray["hil_pc"]="hil_pc_func"
FuncArray["oncar"]="oncar_func"
FuncArray["replay"]="replay_func"


# 调用关联数组中的函数并打印参数
for func_key in "${!FuncArray[@]}"; do
  echo "Key: $func_key, Value: ${FuncArray[$func_key]}"
  if [[ "$func_key" == "$run_mode" ]]; then
    FuncName=${FuncArray[$func_key]}
    $FuncName ${run_mode} ${body_type}
    break
  fi
done


