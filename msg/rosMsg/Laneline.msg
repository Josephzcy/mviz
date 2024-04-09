LineType type                                                # 线型 类型:int 字节数:4 取值范围:(0~8)
LanelinePositionType pos_type                                # 位置 类型:int 字节数:4 取值范围:(0~8)
ColorType color_type                                         # 颜色 类型:int 字节数:4 取值范围:(0~4)
CurveCoeff curve_vehicle_coord                               # 车身坐标系下曲线方程 字节数:40
CurveCoeff curve_image_coord                                 # 图像坐标下曲线方程   字节数:40

Point2fList points_image_coord             # 图像坐标系下的点集 (检测)  字节数:16*n 取值范围(w>=x>=0, h>=y>=0) 最大容量:60
Point3DList points_vehicle_coord           # 车身坐标系下的点集 (检测) 字节数:24*n 取值范围(100>=x>=0, 20>=y>=-20) 最大容量:60
Point2fList fit_points_image_coord         # 图像坐标系下的点集 (拟合抽样) 字节数:16*n 取值范围(w>=x>=0, h>=y>=0) 最大容量:60
Point3DList fit_points_vehicle_coord       # 车身坐标系下的点集 (拟合抽样) 字节数:24*n 取值范围(100>=x>=0, 20>=y>=-20) 最大容量:60
uint64 id                                                   # id 类型:uint64 字节数:8 取值范围(0~2^64)
LineState line_state                                        # 线状态 类型:int 字节数:4 取值范围:(0~2)
float32 confidence                                            # 置信度 类型:float 字节数:4 取值范围:(0~1)
float32 width                                                 # 车道线宽度(单位:m) 类型:float 字节数:4 取值范围:(0, 0.6)
uint32 age                                                  # 声明周期 类型:uint32 字节数:4 取值范围:(0~2^32)
uint32[] point_cam_id                                # 采样点对应的camera id  字节数: 16*n 取值范围 最大容量:255   
Point2f centroid_pt                       # 质心点坐标
int32 lane_id                                               # 车道线位置id(当前车道左侧从0向远处递减，右侧从1向远处递增)