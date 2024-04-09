ln -nsf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime
source /opt/ros/melodic/setup.bash
source /root/mviz2_ros/devel/setup.bash
rosclean check
rosclean purge -y
export LD_LIBRARY_PATH=/usr/local/ffmpeg/lib/:/opt/ros/melodic/lib:/opt/intel/mediasdk/lib/:/root/mviz2_ros/devel/lib:/root/mviz2_ros/lib:../aiplorer/third_party/linux/gflags_2.2.2/lib/
export ROS_PACKAGE_PATH=/root/mviz2_ros/src:/opt/ros/melodic/share:/usr/local/ffmpeg/lib/:/opt/ros/melodic/lib


RED_COLOR='\E[1;31m'
GREEN_COLOR='\E[1;32m'
NC='\e[0m'

if [ $# -ne 2 ] ;then
  echo -e "${RED_COLOR} docker start_mviz_collect.sh rong params, please see the eg above ${NC}"
  exit 1
fi

conf_name=mviz_config.json;car_conf=$2_body.yaml
mviz_mode=$1;bSave=true
rviz_name=mviz_collect.rviz

if [ "$1" = "oncar" ] ;then
  bSave=true
fi
# bSave=true
if [ "$1" == "replay" ] ;then
  rviz_name=mviz_replay.rviz
fi
echo -e "${GREEN_COLOR}Usage: conf_name:${conf_name} car_name:${car_conf} save_mviz_data:${bSave}  ${NC}"


if [ "$1" = "replay" ] ;then
  roslaunch mviz mviz_replay.launch \
    conf_file:=${conf_name} \
    car_conf:=${car_conf} \
    save_mviz_data:=${bSave}\
    mviz_mode:=${mviz_mode} \
    rviz_name:=${rviz_name}
else
  roslaunch mviz mviz_collect.launch \
    conf_file:=${conf_name} \
    car_conf:=${car_conf} \
    save_mviz_data:=${bSave}\
    mviz_mode:=${mviz_mode} \
    rviz_name:=${rviz_name}

fi

