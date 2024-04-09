# 道路合流分流点, 总字节数64
# Type type = 1                    # 类型 字节数:4
Point2f pt_image_coord     # 图像坐标 字节数:8 
Point2f pt_vehicle_coord   # 车身坐标 字节数:8 
uint64[] laneline_id                   # 形成交叉点的车道线id  字节数:32 (最多4个)
int32 state                                   # 状态 字节数:4 
float32 confidence                              # 置信度 字节数:4
int32 pos                                       # 位置  字节数：4 1-> 左边 2-> 右边