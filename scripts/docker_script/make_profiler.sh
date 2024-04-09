#!/bin/bash
RED='\033[0;31m'    # 设置红色的ANSI转义序列
NC='\033[0m'        # 重置ANSI转义序列
workspace_dir="/root/mviz2_ros"
process_path=${workspace_dir}/devel/lib/mviz/mviz_node

src_data="${process_path} mviz.prof"
target_dir=${workspace_dir}/profiler

if [ ! -d ${target_dir}  ];then
  mkdir -p ${target_dir} 
fi

file_name="${target_dir}/mviz"
pprof --text ${src_data} > "${file_name}.txt"
pprof --svg ${src_data} > "${file_name}.svg"
pprof --collapsed ${src_data} >"${file_name}.cbt"

./tools/FlameGraph/flamegraph.pl ${file_name}.cbt > "${file_name}_flame.svg"
./tools/FlameGraph/flamegraph.pl --invert --color aqua ${file_name}.cbt > "${file_name}_invert.svg"














