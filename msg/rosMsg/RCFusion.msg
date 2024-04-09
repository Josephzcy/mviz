    #header.gnssStamp
	uint64 timestamp                              # UTC时戳, 微秒
    #header.timestamp
	uint64 tick                                   # 异构设备上的时钟, 微秒
    uint32 frame_id                               # 融合对应的图像帧号
    #header.seq
    uint32 counter                                # 融合计数，判断融合是否更新
    #ObjAmount
	uint32 track_num                              # 融合目标数量
    #fusionOut
    SingleTarget[] fusion_list             # 融合目标信息
    bool isValid