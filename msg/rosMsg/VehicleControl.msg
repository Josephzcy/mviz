uint64  timestamp        # 时间戳(单位：微秒)
uint64 tick              # 数据面时间(单位：微秒)
uint32  control_state    # 控制状态: 0|off, 1|shakehand, 2|execute_unspecified, 3|execute_waiting, 4|execute_moving, 5|suspend, 6|finish, 7|safe_stop
float32 wheel_cmd        # 方向盘指令(单位：°):左正右负
float32 speed_cmd        # 速度指令(单位：m/s)
uint32  gear_mode        # 档位指令：0|无效/不请求，1|空挡，2|前进，3|后退，4|P挡， 其它：无效值
float32 dist_cmd        # 距离指令：(单位：m) (针对部分车型apa“速度-距离”的控制模式接口)
bool    is_reach_pathend     # 是否到达路径终点(在控制execute_moving状态下，若未到达终点位置，发false，否则发true) 调试用
float32 l_bias_err       # 横向偏差(单位：m) 调试用
float32 yaw_bias_err     # 航向偏差(单位：°) 调试用