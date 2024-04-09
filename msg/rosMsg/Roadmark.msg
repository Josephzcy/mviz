uint32 id                                                    # id 类型:uint32 字节数:4 取值范围:(0~2^32)
RoadmarkType type                                                    # 类型 类型:int 字节数:4 取值范围:(0~17)
Point2fList corner_pt_image_coord     # 外接矩形角点图像坐标 字节数:64 取值范围(w>=x>=0, h>=y>=0) 容量:4
Point3DList corner_pt_vehicle_coord   # 外接矩形角点世界坐标 字节数:96 取值范围(100>=x>=0, 20>=y>=-20) 容量:4
geometry_msgs/Point centroid_vehicle_coord             # 质心世界坐标 字节数:24 取值范围(100>=x>=0, 20>=y>=-20)
float32 confidence                                            # 置信度 类型:float32 字节数:4 取值范围:(0~1)
uint32 lane_idx                                              # 路面标识所在的lane id 类型:uint32 字节数:4 取值范围:(0~7)
# Form form                                             
# State state
Point2f centroid_image_coord
float32 width                                                # 线型标识世界坐标下的宽度
uint32 camera_id                                            # 摄像头id