float32 focal_u                            # 归一化焦距
float32 focal_v                            # 归一化焦距
float32 cu                                 # 光心
float32 cv                                 # 光心
float32[] pos                       # 车身坐标系下坐标(vcs) (x, y, z)
float32 pitch                              # 俯仰角 (vcs) 
float32 yaw                                # 偏航角 (vcs)
float32 roll                               # 横滚角 (vcs)
float32 fov                                # 视场角
int32 image_format                       # 图像格式 [ImageFormat]
string isp_ver                           # isp version 
int32 install_direction                  # 安装方位 (CamDirect的组合) 
TransMatrix trans_mtx                    # 变换矩阵
ProjectionModel prj_model                # 投影模型 
uint32 image_width                       # 像素宽度
uint32 image_height                      # 像素高度
float64[] distort_coeffs           # 畸变系数 (k1, k2, k3, p1, p2) 
uint32 camera_id                         # camera idA