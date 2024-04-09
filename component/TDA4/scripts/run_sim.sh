export LD_LIBRARY_PATH=/usr/lib:/app/algo/apa/lib:/app/algo/vehicle/lib/:/app/algo/lane/libs/:/data/mviz_upstream_tools/lib
echo LD_LIBRARY_PATH=$LD_LIBRARY_PATH
./../bin/sim_tda4_V3 --sim_ip=tcp://192.168.98.233:32000 --camera_delay_ms=0 --enable_sim_waitlk=false 



