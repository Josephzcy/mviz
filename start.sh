source /opt/ros/melodic/setup.bash
source ./devel/setup.bash
rosclean check
rosclean purge -y
export LD_LIBRARY_PATH=/opt/ros/melodic/lib:${PWD}/../aiplorer/third_party/linux/gflags_2.2.2/lib:/usr/local/ffmpeg/lib:${PWD}/../aiplorer/third_party/linux/glog_0.3.5/lib:${PWD}/../aiplorer/third_party/linux/libflow_1.0.0/lib:${PWD}/devel/lib:${PWD}/../aiplorer/third_party/linux/protobuf_3.12.3/lib:${PWD}/../aiplorer/third_party/linux/nanomsg_1.1.5/lib
export ROS_PACKAGE_PATH=${PWD}/src:/opt/ros/melodic/share:/opt/ros/melodic/lib

#roslaunch mviz mviz_x50_hil_pc.launch
roslaunch mviz mviz_replay.launch
