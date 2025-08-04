# ArduSub状态消息
Header header

# 姿态信息 (degrees)
float64 roll
float64 pitch  
float64 yaw

# 位置信息
float64 depth  # 深度 (m)
float64 altitude  # 高度 (m)

# 速度信息
float64 ground_speed
float64 air_speed
float64 climb_rate

# 状态信息
bool armed
uint8 flight_mode
uint8 system_status

# 传感器数据
float64 temperature  # 温度 (°C)
float64 pressure  # 压力 (mbar)
