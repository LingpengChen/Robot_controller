# ArduSub控制命令消息
Header header

# 控制类型
string control_type   # "movement", "mode", "device", "system"

# 运动控制 (值范围: -1.0 到 1.0)
float64 forward       # 前进/后退
float64 strafe        # 左右平移
float64 vertical      # 上浮/下潜
float64 yaw_rate      # 偏航角速度
float64 pitch_rate    # 俯仰角速度
float64 roll_rate     # 横滚角速度

# 模式控制
string flight_mode    # "manual", "stabilize", "depth_hold"

# 设备控制
string device_name    # "gripper", "light_main", "light_circle", "camera"
string device_action  # "open", "close", "on", "off", "toggle", "tilt"
float64 device_value  # 设备控制值

# 系统控制
bool arm_request      # 解锁请求
bool heartbeat        # 心跳信号

# 时间戳
float64 timestamp     # 命令时间戳
