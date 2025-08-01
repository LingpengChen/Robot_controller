# ArduSub状态消息
Header header

# 姿态信息 (角度单位: 度)
float64 pitch        # 俯仰角
float64 roll         # 横滚角  
float64 yaw          # 偏航角

# 位置信息
float64 depth        # 深度 (米)

# 环境信息
float64 temperature  # 温度 (摄氏度)

# 系统状态
bool armed           # 电机解锁状态
string flight_mode   # 飞行模式

# 时间戳
float64 timestamp    # 数据时间戳
