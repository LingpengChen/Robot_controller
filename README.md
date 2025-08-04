# ArduSub ROS Control Package

这个ROS包提供了与ArduSub飞控进行通信和控制的完整解决方案，基于MAVLink协议。

## 功能特性

- **实时数据通信**: 接收飞控的姿态、深度、温度等传感器数据
- **多种控制模式**: 支持手动控制、速度控制、深度控制、姿态控制
- **设备控制**: 支持灯光、机械手、相机云台等设备控制
- **ROS标准接口**: 使用标准ROS消息类型，便于集成其他ROS包
- **无需sudo权限**: 不依赖键盘输入，适合在服务器环境运行

## 系统要求

- Ubuntu 18.04/20.04 + ROS Melodic/Noetic
- Python 3.x
- PyMAVLink库
- 网络连接到ArduSub飞控 (UDP端口14551)

## 安装依赖

```bash
# 安装ROS依赖
sudo apt install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras

# 安装Python依赖
pip3 install pymavlink

# 如果需要MAVROS完整功能
sudo /opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh
```

## 文件结构

```
ardusub_control/
├── ardusub_ros_node.py         # 主ROS节点
├── ardusub_controller.py       # 控制客户端示例
├── control_ardusub.py          # MAVLink控制模块 (原有)
├── udp_connect.py             # UDP代理模块 (原有)
├── connect_main.py            # 原始主程序 (原有)
├── launch/
│   └── ardusub_control.launch # 启动文件
├── msg/
│   └── ArdusubStatus.msg      # 自定义消息类型
├── package.xml                # ROS包配置
├── CMakeLists.txt            # 构建配置
└── README.md                 # 本文件
```

## 编译和安装

```bash
# 将包放到ROS工作空间
cd ~/catkin_ws/src
git clone <your-repo> ardusub_control

# 编译
cd ~/catkin_ws
catkin_make

# 加载环境
source devel/setup.bash
```

## 使用方法

### 1. 启动主节点

```bash
# 方法1: 使用launch文件启动
roslaunch ardusub_control ardusub_control.launch

# 方法2: 直接启动节点
rosrun ardusub_control ardusub_ros_node.py
```

### 2. 使用控制客户端

```bash
# 启动交互式控制客户端
rosrun ardusub_control ardusub_controller.py
```

### 3. 使用ROS话题控制

```bash
# 武装载具
rostopic pub /ardusub/cmd_arm std_msgs/Bool "data: true"

# 发送速度指令 (前进1m/s)
rostopic pub /ardusub/cmd_vel geometry_msgs/Twist "
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# 设置目标深度
rostopic pub /ardusub/cmd_depth std_msgs/Float64 "data: 5.0"

# 切换到定深模式
rostopic pub /ardusub/cmd_mode std_msgs/Int32 "data: 2"

# 开启灯光
rostopic pub /ardusub/cmd_light std_msgs/Bool "data: true"
```

### 4. 查看状态数据

```bash
# 查看姿态数据
rostopic echo /ardusub/attitude

# 查看深度数据  
rostopic echo /ardusub/depth

# 查看温度数据
rostopic echo /ardusub/temperature

# 查看武装状态
rostopic echo /ardusub/armed
```

## ROS话题接口

### 发布的话题 (状态数据)

| 话题名称 | 消息类型 | 描述 |
|---------|---------|-----|
| `/ardusub/attitude` | `geometry_msgs/Vector3` | 姿态角度 (roll, pitch, yaw) |
| `/ardusub/depth` | `std_msgs/Float64` | 当前深度 |
| `/ardusub/temperature` | `sensor_msgs/Temperature` | 水温 |
| `/ardusub/armed` | `std_msgs/Bool` | 武装状态 |
| `/ardusub/mode` | `std_msgs/Int32` | 飞行模式 |

### 订阅的话题 (控制指令)

| 话题名称 | 消息类型 | 描述 |
|---------|---------|-----|
| `/ardusub/cmd_vel` | `geometry_msgs/Twist` | 速度控制指令 |
| `/ardusub/cmd_depth` | `std_msgs/Float64` | 深度控制指令 |
| `/ardusub/cmd_attitude` | `geometry_msgs/Vector3` | 姿态控制指令 |
| `/ardusub/cmd_arm` | `std_msgs/Bool` | 武装/解除武装 |
| `/ardusub/cmd_mode` | `std_msgs/Int32` | 模式切换 |
| `/ardusub/cmd_light` | `std_msgs/Bool` | 灯光控制 |
| `/ardusub/cmd_camera_tilt` | `std_msgs/Float64` | 相机云台控制 |
| `/ardusub/cmd_servo` | `geometry_msgs/Vector3` | 舵机控制 |

## 控制模式说明

### 速度控制模式
通过`/ardusub/cmd_vel`话题发送Twist消息：
- `linear.x`: 前进/后退速度 (正值=前进)
- `linear.y`: 左/右平移速度 (正值=右移)
- `linear.z`: 上浮/下潜速度 (正值=上浮)
- `angular.z`: 偏航角速度 (正值=逆时针)

### 飞行模式
- `0`: 手动模式 (Manual)
- `1`: 自稳模式 (Stabilize) 
- `2`: 定深模式 (Depth Hold)

## 与原有UDP系统的关系

这个ROS包保持了对原有UDP控制系统的兼容性：
- `control_ardusub.py` - 核心MAVLink通信模块保持不变
- `udp_connect.py` - UDP代理可以与ROS节点并行运行
- 新增的ROS接口提供了更标准化和模块化的控制方式

## 常见问题

### 1. 连接问题
- 确保ArduSub飞控在UDP端口14551上监听
- 检查网络连接和防火墙设置
- 确认MAVLink参数配置正确

### 2. 权限问题
- 本包不需要sudo权限
- 如果遇到串口权限问题，将用户添加到dialout组：
  ```bash
  sudo usermod -a -G dialout $USER
  ```

### 3. 依赖问题
- 确保所有ROS依赖包已安装
- 检查Python路径和包导入

## 开发和扩展

### 添加新的控制功能
1. 在`ardusub_ros_node.py`中添加新的订阅者
2. 在相应的回调函数中调用MAVLink命令
3. 在`ardusub_controller.py`中添加用户接口

### 添加新的状态数据
1. 在`mavlink_message_handler`中处理新的MAVLink消息
2. 创建对应的ROS发布者
3. 定义新的消息类型（如需要）

## 许可证

MIT License

## 贡献

欢迎提交Issue和Pull Request来改进这个项目。
