### mviz_havp_vm_v2.0.9 主要更新:    
Mviz_Record   
● 新增端口支持avm_libflow 视频图像的采集   
● 修复imu topic index 文件中tick存为us的缺陷   
● 修复imu_rtk topic index 文件中tick为零的问题    
其他       
● 更新mviz_havp_vm_v2.0.9 软件使用手册

### mviz_havp_vm_v2.0.8 主要更新:    
Mviz_Display   
● 适配havp失败后mviz闪退的问题     
● 新增map_engine中轨迹的显示   
Mviz_Replay   
●  Hil-Sim-Mode  
    1. ddsflow 适配新增topic数据的转发，总共41个pb 的转发  
    2. Hil默认适配所有采集数据的转发，总共44个pb 的回灌   
Mviz_extension  
●  更新sysinfo_recv,适配新版底软的更改  
其他  
● mviz 启动时新增版本号的打印   
● 更新mviz_havp_vm_v2.0.8 软件使用手册  


### mviz_havp_vm_v2.0.7 主要更新:
Mviz_Display   
● 主面板新增ap_map_response状态的显示   
● 主面板新增定位精度的显示       
● 主面板新增定位失败原因的显示   
● 显示界面重新布局   
    1. bev 显示界面移动到四路鱼眼图区域显示（图像相关的要素放在一起）      
    2. 左侧状态显示面板移动到右侧，与视角面板、超声波探测距离值面板共用同一视窗     
● 更新mviz_havp_vm_v2.0.7 软件使用手册 

### mviz_havp_vm_v2.0.6.1 主要更新:
Mviz_Record   
● 新增ihu_to_avm_pb topic 的采集      
● 新增apa_hmi_signal 和apa_hmi_signal_res topic 的采集    
● 新增imu_rtk topic 的采集   
● 适配apa_state_control topic 的修改
Mviz_Replay         
● Hil-Sim-Mode   
    1 ddsflow 适配新增topic数据的转发   
    2 hil适配新增topic的转发     
其他：工程优化      
● 更新mviz_havp_vm_v2.0.6.1 软件使用手册   

### mviz_havp_vm_v2.0.6 主要更新:
Mviz_Record   
● 适配soc_to_hmi 修改为 soc_to_hmi_havp   
其他：工程优化   
● 编译加速：单独将pb 编译成库，给对应的模块使用，减少多次编译带来的时间开销   
● 增加cmakelist.txt 文件的扩展性   
    1. display_gui 模块采用循环构建，减少每次增加gui 需要更改cmakelist的问题   
    2. 顶层cmakelist.txt 采用循环构建，减少每次增加模块需要更改cmakelist的问题   
● 增加common 层，将与平台相关的代码解耦并放到顶层       

### mviz_havp_vm_v2.0.5 主要更新:
Mviz Record   
● 修复记录车辆泊入/泊出起停时间(hmi parking start & stop time)    
● 新增can版本不同，导致数据采集异常问题        
● replay模式下odom坐标系下车位转换到bev图像坐标系下显示 (replay操作详见 3.3.3 章节)   
● replay模式下通过"点选"进度条定位到对应帧      
● MOD模块上下游数据的显示(暂无数据可测试)    
● 新增plan bias相关数据的显示   
● gridmap新增按照数据视觉和超声波类别来显示   

### mviz_apa_vm_v2.0.4 主要更新:   
● replay模式下odom坐标系下车位转换到bev图像坐标系下显示 (replay操作详见 3.3.3 章节)   
● replay模式下通过"点选"进度条定位到对应帧   
● 记录车辆泊入/泊出起停时间(hmi parking start & stop time)   
● 新增CAN数据采集功能   
● 支持MOD模块上下游数据采集   
mviz_apa_vm_v2.0.4.1 更新:   
● MOD模块上下游数据的显示(暂无数据可测试)   
● 重新确认记录车辆泊入/泊出起停时间(hmi parking start & stop time)内容和格式   

### mviz_v2.0.2主要更新如下：
Mviz Record   
● 新增can数据采集功能   
Mviz Replay   
● hil_sim_mode   
  ○ 修复软件包中台架回灌配置丢失问题   
  ○ 修复软件包中Sim版本运行报错的问题   
● Mviz_FAQ 模块正式上线   

### mviz_v2.0.1主要更新如下：
  a. mviz_2.0.1 同步mviz_apa_v1.0.19 所有功能，并新增以下功能：
Mviz Record   
● pb数据的存储的名字统一按照实际接收topic来命名。目前总共record 32个pb数据https://alidocs.dingtalk.com/i/nodes/YndMj49yWjPlP03QC7696oXkJ3pmz5aA?corpId=ding607f16720f9278c6&utm_medium=im_card&iframeQuery=utm_medium%3Dim_card%26utm_source%3Dim&utm_scene=team_space&utm_source=im    
Mviz Replay   
● hil_sim_mode   
  ○ 修改原先hil_sim_mode 启动方式，./start_mviz.sh hil_board x50(byd)    
● 新增 mviz_replay 模式   
    即读取本地数据，播放数据+可视化+Pb明文显示   
  ○ 支持全界面操作：数据的自定义加载和配置，回放显示等；   
  ○ 支持所有数据按某个key_topic对齐，所有数据按帧播放，支持连续播放，前后单步移动播放；   
  ○ 支持pb数据明文同步显示；   
  ○ 支持随时在播放、暂停之间切换；   
  ○ 新增重播放按钮，方便重播放；   
  ○ 新增进度条功能，支持显示当前进度、拖动跳帧播放等；   
  ○ 支持实时fps显示；当前配置文件路径显示；当前播放数据路径显示；keytopic显示。   
others   
● 优化hil_for_mviz 存在时，再次启动hil报错的问题   
● 增加docker_env_release.sh    
● 优化docker_env.sh ,解决每次新版本必须创建容器的问题   
● pack 脚本新增 mviz_replay 功能的打包   


### mviz_apa_v1.0.19主要更新如下：
Mviz Record   
● 增加rtk真值数据的采集，包括parking_ins 、parking_gnss、odometry_3d_gt、odometry_3d_lz 数据的采集   
Mviz Replay    
● Hil-Sim 回灌   
  1. TDA4_HIL_V3 配置文件增加parking_ins 、parking_gnss、odometry_3d_gt、odometry_3d_lz 数据回灌   
  2. TDA4_Sim_V3 配置文件增加 parking_ins 、parking_gnss、odometry_3d_gt、odometry_3d_lz 数据回灌   
  3. dds_to_flow 配置文件增加 parking_ins 、parking_gnss、odometry_3d_gt、odometry_3d_lz  数据回灌   
● Hil-PC 回灌   
  1. Hil_PC回灌增加 parking_ins 、parking_gnss、odometry_3d_gt、odometry_3d_lz 数据   
其他   
● 使用文档补充FQA 板块   
● 更新Mviz 软件使用手册  V1.0.19 https://alidocs.dingtalk.com/i/nodes/P7QG4Yx2Jp7x75R6I2LwXeLjV9dEq3XD?doc_type=wiki_doc#   
● 更新hil 台架回灌和pc 回灌的配置方法   

### mviz_apa_v1.0.18主要更新如下：   
Mviz Display   
● mviz_dispaly 界面对图像面板窗口名字重新命名，方便查看不同视图   
Mviz Extension   
● mviz_extension fault repair   
  1. 修复 ssh 连接限制问题   
  2. 修复sys_info 文件概率丢失问题   
  3. 修复ssh连接后未执行释放的bug    
●  mviz_extension code refactor    
  1. 远程命令执行改为阻塞模式，确保程序执行完   
  2. shell执行脚本由os.system改为subprocoss 方式，并增加脚本返回值判断，确保脚本执行完   
  3. 优化标定数据的采集，由原来直接拷贝/data/calib 下的文件，改为拷贝所需要的19个文件   
  4. 新增fault_diagnosis 文件夹，将故障诊断文件(diaglogA.json) 和(diaglogB.json)拷贝到对应文件中   
  5. mviz_extension log 增加关键信息的打印并用颜色加以区分   
● others   
  6. mviz_extension 不再支持dlt相关的数据采集   
其他   
● 更新Mviz 软件使用手册  V1.0.18 https://alidocs.dingtalk.com/i/nodes/ZX6GRezwJl7z75R6Iv4oawgxVdqbropQ?doc_type=wiki_doc#   


### mviz_apa_v1.0.14主要更新如下：
Mviz Record   
1. 新增pb    
● 新增 cus_parkingspace.pb的采集   
2. 适配pb   
● 适配apa_state_control pb 的修改   
● 适配 ihu_to_soc pb 的修改   
● 适配 planning_to_hmi pb 的修改   
Mviz Replay   
● Hil-Sim 回灌   
    ○ TDA4_HIL_V3 配置文件增加cus_parkingspace 数据回灌配置   
    ○ TDA4_Sim_V3 配置文件增加cus_parkingspace数据回灌配置   
    ○ dds_to_flow 配置文件增加cus_parkingspace 数据转发   
● Hil-PC 回灌   
    ○ mviz_apa_1.0.14 增加Hil_PC回灌增加cus_parkingspace 数据转发   
其他   
● 更新Mviz 软件使用手册  V1.0.14 https://alidocs.dingtalk.com/i/nodes/QOG9lyrgJP3P3abBTGOL0eELVzN67Mw4?doc_type=wiki_doc#   
● mviz_apa_1.0.14 新增/calib/avm_calib 文件的拷贝   

### mviz_apa_v1.0.13主要更新如下：
Mviz Record   
1. 新增pb    
● 新增 avm_settings.pb的采集   
● 新增soc_to_ihu.pb的采集   
● 新增parkingspace_to_ui的采集   
● mviz_replay 适配parkingspace 降频后，apa_state_control 存储异常的问题   
2. 适配pb   
● 适配soc_to_ihu pb 的修改   
Mviz Display   
● 修复接收到多帧超声波空间车位时，历史车位显示异常的bug   
Mviz Replay   
● Hil-Sim 回灌   
    ○ TDA4_HIL_V3 配置文件增加avm_settings、soc_to_ihu、parkingspace_to_ui 数据回灌配置   
    ○ TDA4_Sim_V3 配置文件增加avm_settings、soc_to_ihu、parkingspace_to_ui数据回灌配置   
    ○ dds_to_flow 配置文件增加avm_settings、soc_to_ihu、parkingspace_to_ui 数据转发   
● Hil-PC 回灌   
    ○ mviz_apa_1.0.13 增加Hil_PC回灌增加avm_settings、soc_to_ihu、parkingspace_to_ui 数据转发   
其他   
● 更新Mviz 软件使用手册  V1.0.13 https://alidocs.dingtalk.com/i/nodes/AR4GpnMqJzMKMr7DfOapPZn1VKe0xjE3?doc_type=wiki_doc#   
● 更新验证数据   
