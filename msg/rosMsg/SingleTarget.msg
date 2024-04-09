#ObjectID
uint32 id                                      #->trackID # 目标ID

# /*
# "目标的融合维护状态：
#define TRACK_STATUS_NEW 2                         ->2(New)                                                   
#define TRACK_STATUS_NEW_COASTED 3                 ->3(new Coasted)
#define TRACK_STATUS_NEW_UPDATED_RADAR 4           ->4(new updated)
#define TRACK_STATUS_NEW_UPDATED_CAMERA 5          ->4(new updated)
#define TRACK_STATUS_COASTED 6                     ->6(Coasted)
#define TRACK_STATUS_UPDATED_RADAR 9               ->5(updated)
#define TRACK_STATUS_UPDATED_MUL_RADAR 10          ->5(updated)
#define TRACK_STATUS_UPDATED_CAMERA 11             ->5(updated)
#define TRACK_STATUS_UPDATED_BOTH 12               ->5(updated)
#define TRACK_STATUS_INVALID 0                     ->0(invalid)
#define TRACK_STATUS_MERGE 7                       ->1(Merged)
#define TRACK_STATUS_MERGE_BY_FRONT_VIEW 8         ->0(invalid)
# */
#MaintenanceStatus
uint32 status                                  # 目标更新状态

# /*
# "目标运动状态：
#define TRACK_DYNP_UNKNOWN 0                      ->0xf(notavailable)
#define TRACK_DYNP_STATIONARY 1                   ->0(stationary)
#define TRACK_DYNP_ONCOMING 2                     ->3(oncoming)
#define TRACK_DYNP_MOVING 3                       ->2(moving)
#define TRACK_DYNP_ONCOMING_FAST 4                ->3(oncoming)
#define TRACK_DYNP_MOVING_FAST 5                  ->2(moving)
# */
#MotionPattern
uint32 movement                                # 目标运动状态

# /*
# 障碍物类别:
# const int32_t kVehicleClassNegative = 0;          ->0(undetermined)
# const int32_t kVehicleClassCar = 1;               ->1(car)
# const int32_t kVehicleClassMiniBus = 2;           ->1(car)
# const int32_t kVehicleClassBus = 3;               ->3(bus)
# const int32_t kVehicleClassTruck = 4;             ->4(boxtruck)
# const int32_t kVehicleClassSpecial = 5;           ->5(special car)
# const int32_t kVehicleClassCnt = 6;               ->0(undetermined)
# const int32_t kCyclist = 7;                       ->7(bicycle)
# const int32_t kPedestrian = 8;                    ->6(Pedestrian)

# */
#Class
uint32 object_class                           # 目标分类

#AccelerationAbs.y
float32 accel_lat_obj                         #->accel_lat_obj   # 目标横向加速度

uint32 vis_track_id                         #->vis_track_id # 关联到视觉目标ID
uint32 radar_track_id                       #  # 关联的雷达目标ID

#RectInfo.SizeLWH.y
float32 width                                 #->Width  # 目标宽度

#ExistenceProbability
float32 confidence                            #->confidence  # 目标置信度

bool cipv                                   #->把cipv为true的track设置到0x400报文中  # 目标是否是关键目标        

#VelocityAbs.x
float32 v_long_obj                            #->v_long_obj   # 目标纵向速度

#VelocityAbs.y
float32 v_lat_obj                             #->v_lat_obj   # 目标横向速度

float32 l_long_rel                            #->l_long_rel   # 目标纵向距离(参考点)

float32 l_lat_rel                             #->l_lat_rel   # 目标横向距离（参考点）

# /*
# 在当前的融合Cycle内，探测到该目标的传感器：
# Bit0:Radar Front
# Bit1:Radar FrontLeft
# Bit2:Radar FrontRight
# Bit3:Radar RearLeft
# Bit4:Radar RearRight
# Bit9:Camera FrontWideAngle
# Bit10:Camera Rear
# Bit13:Camera LeftForwardLooking
# Bit14:Camera LeftBackwardLooking
# Bit17:Camera RightForwardLooking
# Bit18:Camera RightBackwardLooking
# */
# /*
# 以下只针对1v1r
# 只有bit0->1(single tracklet);
# 只有bit9->3(vision only);
# 同时有bit0和bit9->4(tracklet and vison)
# 其他情况则均为0        
# */
#DetectSensor_Current
uint32 detection_sensor                        # 目标由哪个传感器更新

#AccelerationAbs.x
float32 accel_long_obj                           #->accel_long_obj # 目标纵向加速度

#RectInfo.SizeLWH.x
float32 length                                   #->length # 目标长度

#RectInfo.Orientation
float32 heading_angle                              #->heading_angle # 目标航向角

# RadarFrame asso_radar
# CameraFrame asso_camera 
# /*
# "在目标的生命周期内，曾经探测到该目标的传感器。
# 位域的定义同detection_sensor。"
# */
#DetectSensor_History
uint32 DetectSensor_History                    

# /*
# "讨论结果：如果OBJ是Radar Only OBJ时,后续讨论类别置信度输出问题
# ==>直接赋值100，很危险，所以请Minieye使用更为合理的方式评估。"
# */
float32 ClassConfidence
# /*
# 障碍物被识别的时间戳,us
# */
uint64 TimeCreation
# /*
# 障碍物最近更新时间,us
# */
uint64 LastUpdatedTime
# /*
# "记载当该目标被某Sensor识别时，Sensor赋予它的ID号。
# 数组大小32，排序方法与上方DetectSensor使用的位域排序方法一致。预留的元素均填“0”"
# */
uint32[] SensorID
# /*
# "目标历史运动状态：
# */
uint32 MotionPatternHistory
# /*
# "刹车灯的状态：
# 0-Unknow
# 1-Off
# 2-On"
# */
uint32 BrakeLightSt
# /*
# "转向信号灯状态：
# 0-Unknow
# 1-Off
# 2-Left_Flash
# 3-Right_Flash
# 4-Left_and_rihgt_Flash"
# */
uint32 TurnLightSt
# /*
# "近边，即传感器探测的目标的面。
# 0-FRONT;
# 1-REAR;
# 2-RIGHTSIDE;
# 3-LEFTSIDE;"
# */
uint32 NearSide                                 #->Target_pos

FusionRect3D RectInfo

float32 OrientationStdDev

float32 ttc

uint32 lane_id

uint32 tsl_id

uint32 cut_state

float32 collip_prob