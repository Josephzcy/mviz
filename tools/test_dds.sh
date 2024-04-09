
workspace_dir="./"
target_dir=${workspace_dir}/perf_data/
if [ ! -d ${target_dir}  ];then
  mkdir -p ${target_dir} 
fi

t1q_path="/data/mviz_upstream_tools/scripts"
scp -i debug.key root@192.168.98.233:${t1q_path}/out.perf  ${target_dir}
./tools/FlameGraph/stackcollapse-perf.pl "${target_dir}/out.perf" > "${target_dir}/out.folded"
./tools/FlameGraph/flamegraph.pl "${target_dir}/out.folded" > ${target_dir}/out.svg
 


# ssh root@192.168.98.233 -i debug.key && rm ${t1q_path}/*perf* 
# perf record -a -g -p $PID sleep $Time && perf script > out.perf 
# perf record -a -g -p 2357 sleep 300 && perf script > out.perf

