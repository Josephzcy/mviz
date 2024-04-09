container_names="mviz_dev" 
container_id=$(docker ps -a |grep "${container_names}" | awk '{print $1}')
echo ${container_id}
docker exec -it ${container_id} /bin/bash -c "cd /root/mviz2_ros/ && ./scripts/docker_script/make_profiler.sh "

