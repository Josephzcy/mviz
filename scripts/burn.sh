yellow='\e[0;33m' 
NC='\e[0m'
echo -e "${yellow} ====================== burn tools ....======================== ${NC}"

mviz_upstream_tools_path="/data/mviz_upstream_tools"
chmod 600 debug.key 
chmod 600 ./extensions/Ubuntu20.04/scripts/debug.key
if [ ! -d ${mviz_upstream_tools_path}  ];then
  ssh -i debug.key root@192.168.98.233  mkdir -p ${mviz_upstream_tools_path} 
fi
scp -i debug.key ./extensions/TDA4/scripts/start_mviz_upstream_components_board.sh root@192.168.98.233:${mviz_upstream_tools_path}/start_mviz_upstream_components_board.sh
scp -i debug.key -r ./component/TDA4/* root@192.168.98.233:${mviz_upstream_tools_path}
