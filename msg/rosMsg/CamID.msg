	# /* 0x1 -> front, 0x2 -> rear, 0x4 -> left, 0x8 -> right
	#  * 0x1 | 0x4 -> left and front
	#  * 0x2 | 0x4 -> left and rear
	#  * 0x1 | 0x8 -> right and front
	#  * 0x2 | 0x8 -> right and rear
	#  */
	int32 cam_direct # 安装位置
	int8 fov        # fov描述 
	int32 id        # 扩展字段(可以直接用此字段表示camera个体)