float32 pitch_est                                              # 估计pitch(单位: radian) 类型:float32 字节数:4 取值范围:[-0.175, 0.175]
Point2f vanish_pt                         # 灭点 字节数：8
Point2f vanish_cov                        # 灭点协方差 字节数：8
bool is_valid 												 # 灭点有效标志 字节数：1
uint32 camera_id                                             # 摄像头id
float32 dpitch                                                # delta pitch (unit: radian)
float32 confidence                                            # 置信度