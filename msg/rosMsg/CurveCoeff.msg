# 曲线方程: y = c0 + c1*x + c2*x^2 + c3*x^3
float32 longitude_min  # view range start 类型:float 字节数:4 取值范围:(FLT_MIN~FLT_MAX)
float32 longitude_max  # view range end   类型:float 字节数:4 取值范围:(FLT_MIN~FLT_MAX)
float64 c0            # 类型:float64 字节数:8 取值范围:(DOUBLE_MIN~DOUBLE_MAX)
float64 c1            # 类型:float64 字节数:8 取值范围:(DOUBLE_MIN~DOUBLE_MAX)
float64 c2            # 类型:float64 字节数:8 取值范围:(DOUBLE_MIN~DOUBLE_MAX)
float64 c3            # 类型:float64 字节数:8 取值范围:(DOUBLE_MIN~DOUBLE_MAX)
float64 dev_c0        # c0的标准差，即，车道线横向位置的标准差
float64 dev_c1		  # c1的标准差，即，车道线航向角的标准差
float64 dev_c2        # c2的标准差，即，车道线原点处曲率的标准差
float64 dev_c3       # c3的标准差，即，车道线原点处曲率变化率的标注差