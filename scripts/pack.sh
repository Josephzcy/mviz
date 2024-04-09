#!/bin/bash  
RED_COLOR='\E[1;31m'
GREEN_COLOR='\E[1;32m'

NC='\e[0m'
echo -e "${GREEN_COLOR} ====================== [ pack mviz start ] ....======================== ${NC}"
echo -e "${GREEN_COLOR}Usage: $0 <ball name> <j3|tda4>   eg: $0 mviz_apa_vm_v2.0.4 tda4  ${NC}"

platform="tda4"
if [ $# -eq 2 ] && ([ $2 = "j3" ] || [ $2 = "tda4" ]); then
    platform=$2
else
  echo -e "${RED_COLOR} Wrong params, please see the eg above ${NC}"
  exit 1
fi

mviz_package_path=`realpath $1`
echo "mviz_package_path:${mviz_package_path}"

if [ ! -d ${mviz_package_path}  ];then
  mkdir -p ${mviz_package_path} 
else 
  rm -rf ${mviz_package_path}/*
fi

mviz_workspace_dir=${PWD}
echo "mviz_workspace_dir:${mviz_workspace_dir}"
cd ${mviz_workspace_dir}

mviz_package_name=mviz

cp --path devel/lib/${mviz_package_name}/mviz_node  ${mviz_package_path} 
cp --path devel/lib/*.so  ${mviz_package_path}      
cp --path devel/{setup.bash,setup.sh,_setup_util.py,.rosinstall,.catkin}  ${mviz_package_path}
cp --path scripts/docker_env_release.sh  ${mviz_package_path}
cp --path scripts/docker_script/*.sh   ${mviz_package_path}

cp --path src/${mviz_package_name}/package.xml  ${mviz_package_path}         
cp --path src/${mviz_package_name}/plugin_rviz/plugin_description.xml  ${mviz_package_path}   
cp --path src/${mviz_package_name}/launch/*  ${mviz_package_path}         
cp --path src/${mviz_package_name}/config/*  ${mviz_package_path}         

if [ ! -d lib  ];then
  mkdir -p ${mviz_package_path}/lib 
fi

mviz_third_party_path=${mviz_workspace_dir}/../aiplorer/third_party/linux

if [ "$platform" = "j3" ]; then
  cp -rf ${mviz_third_party_path}/protobuf_2.6.1/lib/libprotobuf*  ${mviz_package_path}/lib
elif [ "$platform" = "tda4" ]; then
  cp -rf ${mviz_third_party_path}/protobuf_3.12.3/lib/libprotobuf*  ${mviz_package_path}/lib
fi

cp -rf ${mviz_third_party_path}/libflow_1.0.0/lib/libflow.so  ${mviz_package_path}/lib
cp -rf ${mviz_third_party_path}/glog_0.3.5/lib/libglog.so*  ${mviz_package_path}/lib
cp -rf ${mviz_third_party_path}/gflags_2.2.2/lib/libgflags.so*  ${mviz_package_path}/lib
cp -rf ${mviz_third_party_path}/nanomsg_1.1.5/lib/libnanomsg.so*  ${mviz_package_path}/lib


# hil tools 
cp -r --path component ${mviz_package_path}
cp -r --path extensions ${mviz_package_path}
cp ./scripts/burn.sh  ${mviz_package_path}
cp -r  debug.key  ${mviz_package_path}
cp run_hil_mviz.sh ${mviz_package_path}
cp ./scripts/start_mviz.sh ${mviz_package_path}

cp CHANGELOG.MD ${mviz_package_path}

mviz_package_name=$(basename "${mviz_package_path}")

echo "mviz_package_name:${mviz_package_name}"

# todo:mviz apa打成包 ，解包tar zxvf
cd ${mviz_package_path} && rm -rf extensions/Ubuntu20.04/lib
strip devel/lib/*.so devel/lib/mviz/*  lib/*
tar -czf ${mviz_package_name}.tar.gz *
cp ${mviz_package_name}.tar.gz ${mviz_package_path}/../ && cd ../

rm -rf ${mviz_package_path}
echo -e "${GREEN_COLOR} ======================  [ pack mviz end ]  ....======================== ${NC}"
