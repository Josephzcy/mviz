WorkSpaceDir="./" 
hil_script_dir=${WorkSpaceDir}/component/ubuntu20.04/
hil_for_mviz_pid=`ps -aux | grep "hil_for_mviz" |grep -v "grep"|awk '{print $2}'`
if [ -n "$hil_for_mviz_pid" ]; then 
  killall -9 hil_for_mviz
  sleep 0.5
  echo "all hil_for_mviz has been killed."
  echo "next start hil_for_mviz"
  cd ${hil_script_dir} && ./run_hil_mviz.sh
else
  echo "now start hil_for_mviz"
  cd ${hil_script_dir} && ./run_hil_mviz.sh
fi
