source /opt/ros/melodic/setup.bash
export LD_LIBRARY_PATH=/usr/local/ffmpeg/lib/:/opt/ros/melodic/lib:/opt/intel/mediasdk/lib/
export ROS_PACKAGE_PATH=/root/mviz2_ros/src:/opt/ros/melodic/share:/usr/local/ffmpeg/lib/:/opt/ros/melodic/lib
# catkin_make -DCATKIN_WHITELIST_PACKAGES="mviz" -j8
if [ $# -eq 0 ]; then
  echo "Executing command for tda4."
  catkin_make -j $(nproc) -DMVIZ_TDA4=ON -DMVIZ_J3=OFF 
fi
if [ $# -eq 1 ]; then
  if [ "$1" = "j3" ]; then
    echo "Executing command for j3"
    catkin_make -j $(nproc) -DMVIZ_J3=ON -DMVIZ_TDA4=OFF
  elif [ "$1" = "tda4" ]; then
    echo "Executing command for tda4"
    catkin_make -j $(nproc) -DMVIZ_TDA4=ON -DMVIZ_J3=OFF
  fi
fi
# catkin_make install 
