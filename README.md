# mviz2_ros

将mviz2 （https://git.minieye.tech/zhangjia/mviz2） 采用ros包的组织方式，移植mviz1 （https://git.minieye.tech/zhangchengyu/mviz_apa） 的可视化.

## Getting started
git clone git@git.minieye.tech:mviz_group/mviz2_ros.git

## 编译
当本地编译和docker容器编译切换时，先执行：
```bash
cd mviz2_ros
sudo rm build devel -rf
```

开启新的终端需要source 环境变量
```bash
#if bash
. /opt/ros/noetic/setup.bash
#if zsh
. /opt/ros/noetic/setup.zsh
```
或者写到~/.bashrc中永久设置。
### 本地环境的编译和运行

#### 编译
```bash
cd mviz2_ros
catkin_make -j$(nproc)
```

#### 运行
- hil
./run_hil_libflow.sh

- bash
. ./devel/setup.bash
- zsh
. ./devel/setup.zsh

- mviz
clear && roslaunch mviz mviz_apa_x50_hil_noetic.launch

- replay
 roslaunch mviz mviz_replay.launch

### docker容器环境的编译和运行 （推荐）
#### 容器生成
./scripts/docker_env.sh

#### 编译
./scripts/build_mviz.sh <tda4|j3>   

#### 运行
## 回灌模式
- hil
1. ubuntu18.04:./run_hil_libflow.sh
1. ubuntu20.04:./run_hil_mviz.sh

- mviz
2. hil_sim_mode:
  + ./start_mviz.sh hil_board x50 ("车型")
  + ./start_mviz.sh hil_pc x50 ("车型")

- replay_mode
  ./start_mviz.sh replay x50

## 实车模式
/start_mviz.sh oncar x50 ("车型")

## 其他
修复catkin: sudo apt-get install --reinstall ros-melodic-catkin
catkin_create_pkg mviz roscpp rospy rosmsg cv_bridge image_transport sensor_msgs tf std_msgs qt_build libqt5-dev   

## 版本   
```
截至当前20231213，分3个不同发布版本，mviz_apa_vm, mviz_apa_j3, mviz_havp_vm   
版本号命名方式：mviz_apa_vm_v1.2.3, mviz_apa_j3_v3.2.1,  mviz_havp_vm_v2.2.2   
而且当前mviz_havp_vm的功能cover了mviz_apa_vm的功能，dev 分支包含了 mviz_apa_vm, mviz_apa_j3的代码，   
mviz_havp_vm在havp_t1q3分支，后续会做合并，为了减少麻烦，暂时在合并前发mviz_apa_vm_v2.0.4版本 
```  


## 编译和打包:   
```
mviz_apa_vm：./scripts/build_mviz.sh tda4      (依赖aiplorer master_apa_x50分支的precompiler)   
mviz_apa_j3:  ./scripts/build_mviz.sh j3       (依赖aiplorer master_apa_j3分支的precompiler)   
mviz_havp_vm: ./scripts/build_mviz.sh tda4     (依赖aiplorer master_hapa_t1q3分支的precompiler)    
打包： ./scripts/pack.sh  ball_name  <tda4|j3>   
```


## havp_vm平台相关分支编译apa_vm 相关分支 
1. aiplorer切换到相关分支    
  1. havp_vm 对应master_havp_t1q3   
  2. apa_vm  对应master_apa_x50(byd)   
2. 打开 platform_defination.h文件中相关平台头文件包含   
3. 打开topic_global_object_register.hpp文件中相关平台头文件包含   

## havp_vm平台相关分支编译apa_vm 相关分支   
1. 增加pb 的修改方法
  1. ddsflow 和mviz_config.json 中增加相关的topic配置
  2. 平台相关文件中添加对应的头文件并且注册topic和要采集的数据类型即可