#!/bin/bash
clear
RED='\033[0;31m'    # 设置红色的ANSI转义序列
NC='\033[0m'        # 重置ANSI转义序列
platform=""
if [ $# -eq 0 ]; then
  platform="tda4"
elif [ $# -eq 1 ]; then
  if [ $1 = "j3" ] || [ $1 = "tda4" ]; then
    platform=$1
  else
    echo -e "${RED}[error] don't support: $1${NC}"
    echo -e "${RED}[usage]: $0 j3 or $0 tda4;${NC}"
    exit 1
  fi
else
  echo -e "${RED}[error]too many parameters ${NC}"
  exit 1
fi

yellow='\e[0;33m' 
NC='\e[0m'
echo -e "${yellow} ====================== build mviz ....======================== ${NC}"

container_names="mviz_dev" 
container_id=$(docker ps -a |grep "${container_names}" | awk '{print $1}')
echo "container_id:${container_id}"

if [[ -n $(docker ps -q -f "name=${container_names}") ]];then
  echo "${container_names} is running"
else
  echo "${container_names} is down!"
  docker start ${container_id}
fi

docker exec -it ${container_id} /bin/bash -c "cd /root/mviz2_ros/ && ./scripts/docker_script/build.sh ${platform} "

