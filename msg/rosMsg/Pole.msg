uint32 id                             # id 类型:uint32 字节数:4 取值范围:(0~2^32)
Point2fList corner_pt_image_coord     # 关键点图像坐标 字节数:64 取值范围(w>=x>=0, h>=y>=0) 容量:4
Point3DList corner_pt_vehicle_coord   # 关键点世界坐标 字节数:96 取值范围(100>=x>=0, 20>=y>=-20) 容量:4
geometry_msgs/Point centroid_vehicle_coord        # 质心世界坐标 字节数:24 取值范围(100>=x>=0, 20>=y>=-20)
float32 confidence                    # 置信度 类型:float 字节数:4 取值范围:(0~1)
Point2f centroid_image_coord
int32 pos                             # 位置 1->左边 2->右边
uint32 camera_id                      # 摄像头id