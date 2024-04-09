float32 speed_mps           # m/s
bool is_imu_valid         # 是否去除bias
float32 dyaw                # delta yaw, unit: radian
bool is_dyaw_valid         # dyaw 标志位
float32 dpitch               # delta pitch, unit : radian
bool is_dpitch_valid       # dpitch 标志位
int32 dt_ms                # time lag between two images
uint64 curr_timestamp      # 当前时戳