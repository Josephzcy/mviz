uint64 frame_id                                   # 帧ID 类型:uint64 字节数:8 取值范围:(0~2^64)
uint64 timestamp                                  # 对应的图像时戳 类型:uint64 字节数:8 取值范围:(0~2^64)
LanelineList laneline                             # 车道线 字节数:(0~39328)
FreeSpaces freespaces                             # 可通行区域 字节数:(12~6412)
Roadmarks roadmarkers                             # 路面标识 字节数:(0~3180)
RoadEdgeList road_edges                           # 路沿 字节数:(0~5080)
# LDWInfo ldw_info                                  # 告警信息 字节数:25
LanelineList calib_lines                          # 用于标定的车道线方程 字节数:(0~9824)
MotionEst motion                                 # 运动估计信息 字节数:12
PoseEst pose                                     # 姿态估计信息 字节数:4 (前视摄像头)
# HppInfo hpp_info                                 # 预测行驶线 字节数:(370-14770）
uint64 tick                                      # tick ms 字节数: 8
DataSource data_source                   # 描述数据源 字节数: 4
CurveCoeff slope_equation                        # 坡度方程 字节数 40
uint64 finish_time                            # 处理结束时间 字节数 8
JunctionList junc_list                          # 交叉点信息 字节数 256 (最多4个交叉点) 
# RoadSysProfiling profiling_items                # 统计项
uint64 recv_time                                # 图像接收时间戳ms
float32 fps                                       # 车道线算法fps
EgoMotion ego_motion                    # 基于imu和can_speed估计两帧之间的运动
float32 ego_lane_width                            # 自车道宽度（m）
CamID cam_id                            # 摄像头id
uint32 lane_change                                # 车辆变道，跟LDW有点区别，这个是指左右线切换了。0： 无 1:左  2： 右
PoleList pole_list                              # 路灯杆