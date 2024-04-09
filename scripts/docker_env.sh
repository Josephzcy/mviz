#!/bin/bash
RED_COLOR='\E[1;31m'   #红
GREEN_COLOR='\E[1;32m' #绿
YELOW_COLOR='\E[1;33m' #黄
BLUE_COLOR='\E[1;34m'  #蓝
RES='\E[0m'

echo -e "${BLUE_COLOR}please run this shell under mviz like this: bash ./script/docker_env.sh${RES}"
cd `dirname $(dirname $0)`    # in mviz2_ros dir now
docker_image_dir=../mviz_melodic_1211.tar     #此处建议用绝对路径
volume_dir_path=$(pwd)

container_names="mviz_dev"

if [ -n "`docker ps -a|grep  ${container_names} -w`" ]; then
	echo -e "${GREEN_COLOR}docker ${container_names} is uping ${RES}"
  docker stop ${container_names}
  docker rm ${container_names}
fi

# find docker_image
if [[ "$(docker images -q mviz_image:v0 2> /dev/null)" = "" ]]; then
  if [[ -f  ${docker_image_dir} ]] ; then 
    echo -e  "${GREEN_COLOR}find image file $(basename ${docker_image_dir}) in $(dirname $docker_image_dir)/ ${RES}"
    echo -e "${GREEN_COLOR}loading image ... ${RES}" 
    docker load -i ${docker_image_dir}
  else
    echo -e  "${RED_COLOR}docker image file not found !!! ${RES}"
    exit 1
  fi
else
  echo -e "${BLUE_COLOR}docker image is existied on local ${RES}"
fi

# start container
echo -e "${GREEN_COLOR}... to run docker container ...${RES}"
docker run -v /tmp/.X11-unix:/tmp/.X11-unix -v /usr/share/zoneinfo:/usr/share/zoneinfo -e DISPLAY=unix$DISPLAY \
          -e GDK_SCALE -e GDK_DPI_SCALE --device=/dev/dri --group-add video \
          -v ${volume_dir_path}/../aiplorer:/root/aiplorer -v ${volume_dir_path}:/root/mviz2_ros  --privileged=true -it \
          --network host -d --ipc=host \
          --name ${container_names} mviz_image:v0


sleep 2
if [ -n "`docker ps |grep  ${container_names} -w`" ]; then
  echo -e "${GREEN_COLOR}docker ${container_names} is uping~~${RES}"
else
  echo -e "${RED_COLOR}run ${container_names} fail ${RES}"
fi







