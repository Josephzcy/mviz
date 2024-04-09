# 获取并输出当前路径
current_path="$(pwd)"
workspaceDir=$current_path/../../../
echo "run_sysinfo.sh: $workspaceDir"
export LD_LIBRARY_PATH=$workspaceDir/lib:$current_path/../lib:${LD_LIBRARY_PATH}
./../bin/recv_sysinfo >/dev/null 2>&1 &
# ./../bin/recv_sysinfo >./log.txt 2>&1 &


