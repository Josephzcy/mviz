uint32 id                                                    # id 类型:uint32 字节数:4 取值范围:(0~2^32)
uint32 age				                                      # 检测帧数 类型:uint32 字节数:4 取值范围:(0~2^32)
CurveCoeff curve_vehicle_coord                               # 车身坐标系下曲线方程 字节数:40
CurveCoeff curve_image_coord                                 # 图像坐标系下的曲线方程 字节数:40
Point3DList points_vehicle_coord           # 车身坐标系下的采样点 字节数:24*n 取值范围(100>=x>=0, 20>=y>=-20) 最大容量:60
Point2fList points_image_coord             # 图像坐标系下的采样点 字节数:16*n 取值范围(w>=x>=0, h>=y>=0) 最大容量:60
Point2fList fit_points_image_coord         # 图像坐标系下的点集 (拟合抽样) 字节数:16*n 取值范围(w>=x>=0, h>=y>=0) 最大容量:60
Point3DList fit_points_vehicle_coord       # 车身坐标系下的点集 (拟合抽样) 字节数:24*n 取值范围(100>=x>=0, 20>=y>=-20) 最大容量:60
LineState state                                              # 线状态 类型:int 字节数:4 取值范围:(0~2)
int32 side                                                  # 路沿在左侧还是右侧 类型:int32 字节数:4 取值范围(0~1)
float32 confidence                                            # 置信度 类型:float 字节数:4 取值范围:(0~1)
uint32[] camera_id                         # 摄像头id 