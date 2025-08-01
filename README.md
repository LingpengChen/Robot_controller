# Robot_controller （ArduSub控制系统）

这是一个基于ROS的ArduSub水下机器人控制系统的重构版本，采用模块化设计，支持多种控制方式，无需sudo权限，更加安全可靠。

## 🏗️ 项目结构

```
ardusub_control/                    # ROS包根目录
├── package.xml                     # ROS包配置文件
├── CMakeLists.txt                  # CMake构建文件
├── requirements.txt                # Python依赖
├── install.sh                      # 安装脚本
├── README.md                       # 项目说明
│
├── src/ardusub_control/           # 核心库模块
│   ├── __init__.py                # 包初始化
│   ├── control_ardusub.py         # ArduSub底层控制库
│   ├── udp_connect.py             # UDP通信核心类
│   └── config_manager.py          # 配置管理器
│
├── scripts/                       # ROS可执行脚本
│   ├── ardusub_ros_bridge.py      # ROS桥接节点 (主推荐)
│   ├── ardusub_keyboard_ros.py    # 键盘控制节点
│   ├── ardusub_gui_controller.py  # GUI控制界面
│   └── ardusub_demo.py            # 演示程序
│
├── tools/                         # 工具脚本
│   ├── keyboard_controller.py     # 原版键盘控制器
│   ├── status_receiver.py         # UDP状态接收器
│   └── connect_main.py            # UDP主程序入口
│
├── config/                        # 配置文件
│   ├── ardusub_config.yaml        # 系统配置
│   └── start_ardusub.sh           # 启动脚本 (重构版)
│
├── launch/                        # ROS启动文件
│   └── ardusub_control.launch     # 系统启动配置
│
└── msg/                           # ROS消息定义
    ├── ArduSubStatus.msg          # 状态消息
    └── ArduSubCommand.msg         # 控制命令消息
```

## 系统架构

```
ROS话题系统 ←→ ArduSub ROS桥接 ←→ MAVLink ←→ ArduSub飞控 ←→ 水下机器人
     ↑
GUI控制界面 / 键盘控制 / 其他ROS节点
```

## 文件说明

### 核心文件
- `connect_main.py` - 原始UDP主程序入口
- `udp_connect.py` - UDP通信核心类 (已修复拼写错误)
- `control_ardusub.py` - ArduSub底层控制库

### ROS相关文件
- `ardusub_ros_bridge.py` - **ROS桥接节点** (主推荐)
- `ardusub_keyboard_ros.py` - 基于pynput的键盘控制节点
- `ardusub_gui_controller.py` - **GUI控制界面** (无需sudo)
- `msg/ArduSubStatus.msg` - 状态消息定义
- `msg/ArduSubCommand.msg` - 控制命令定义

### 工具脚本
- `status_receiver.py` - UDP状态接收器
- `start_ardusub.sh` - 启动脚本
- `README.md` - 本说明文档

## 🚀 快速开始

### 方式1: 自动安装 (推荐)
```bash
# 运行安装脚本
./install.sh

# 启动系统
./config/start_ardusub.sh
```

### 方式2: 使用启动脚本
```bash
# 进入包目录
cd /path/to/catkin_ws/src/ardusub_control

# 运行启动脚本 (交互式菜单)
./config/start_ardusub.sh

# 或者直接指定启动模式
./config/start_ardusub.sh gui          # GUI控制器
./config/start_ardusub.sh ros          # 完整ROS系统
./config/start_ardusub.sh bridge       # ROS桥接节点
```

### 方式3: 直接启动GUI控制 (最简单)
```bash
cd scripts
python3 ardusub_gui_controller.py
```

### 方式4: ROS模式
```bash
# 终端1: 启动完整ROS系统
roslaunch ardusub_control ardusub_control.launch

# 或者分步启动:
# 终端1: 启动ROS桥接
cd scripts && python3 ardusub_ros_bridge.py

# 终端2: 启动键盘控制 (可选)
cd scripts && python3 ardusub_keyboard_ros.py
```

## 🎮 控制方式

### 1. GUI控制界面 (推荐)
- **优点**: 直观易用，无需sudo权限，支持鼠标和键盘
- **特点**: 实时状态显示，按钮控制，键盘快捷键
- **启动**: `python3 ardusub_gui_controller.py`

### 2. ROS话题控制
- **优点**: 标准ROS接口，易于集成其他ROS节点
- **话题列表**:
  - `/ardusub/cmd_vel` (geometry_msgs/Twist) - 运动控制
  - `/ardusub/set_mode` (std_msgs/String) - 模式切换
  - `/ardusub/arm` (std_msgs/Bool) - 解锁控制
  - `/ardusub/device_cmd` (std_msgs/String) - 设备控制

### 3. 键盘控制 (pynput)
- **优点**: 无需sudo权限，基于pynput库
- **启动**: `python3 ardusub_keyboard_ros.py`

## 🎯 控制说明

### 运动控制
- **W/S**: 前进/后退
- **A/D**: 左平移/右平移
- **Q/E**: 上浮/下潜
- **J/L**: 左转/右转
- **I/K**: 抬头/低头
- **U/O**: 左倾/右倾

### 模式切换
- **1**: 手动模式
- **2**: 自稳模式
- **3**: 定深模式

### 设备控制
- **F**: 机械手合
- **G**: 机械手张
- **T**: 主灯开关
- **Y**: 光圈灯开关

### 系统控制
- **Space**: 解锁/锁定电机
- **H**: 发送心跳包
- **ESC**: 退出程序

## 📊 状态反馈

系统实时反馈以下数据：
- **姿态信息**: 俯仰角、横滚角、偏航角
- **位置信息**: 深度
- **环境数据**: 水温
- **系统状态**: 电机解锁状态、飞行模式

## 🔧 ROS话题接口

### 发布的话题
```bash
/ardusub/imu           # sensor_msgs/Imu - IMU数据
/ardusub/depth         # std_msgs/Float64 - 深度信息
/ardusub/temperature   # std_msgs/Float64 - 温度数据
/ardusub/armed         # std_msgs/Bool - 解锁状态
/ardusub/flight_mode   # std_msgs/String - 飞行模式
```

### 订阅的话题
```bash
/ardusub/cmd_vel       # geometry_msgs/Twist - 运动控制
/ardusub/set_mode      # std_msgs/String - 模式设置
/ardusub/arm           # std_msgs/Bool - 解锁控制
/ardusub/device_cmd    # std_msgs/String - 设备控制
```

## 📦 依赖安装

### 自动安装 (推荐)
```bash
./install.sh
```

### 手动安装
```bash
# Python依赖
pip3 install -r requirements.txt
# 或者
pip3 install pymavlink pynput PyYAML numpy

# 系统依赖 (Ubuntu/Debian)
sudo apt-get install python3-tk

# ROS依赖 (如果使用ROS模式)
source /opt/ros/noetic/setup.bash  # 或对应ROS版本
catkin_make  # 在工作空间根目录运行
```

## 🛡️ 安全特性

1. **无需sudo权限**: 使用pynput和tkinter，避免安全风险
2. **标准ROS接口**: 符合机器人开发规范
3. **多重保护**: 心跳检测、状态监控、错误处理
4. **兼容性好**: 保持与原UDP协议的兼容性

## 🔍 故障排除

### GUI无法启动
- 检查是否安装了tkinter: `python3 -m tkinter`
- 确保有图形界面环境

### 键盘控制无响应
- 确保程序窗口获得焦点
- 检查pynput是否正确安装

### ROS连接问题
- 检查ROS环境变量: `echo $ROS_MASTER_URI`
- 确保roscore正在运行: `roscore`

### MAVLink连接失败
- 检查ArduSub飞控连接
- 确认串口权限和设备路径

## 🎯 推荐使用方式

1. **新用户**: 直接使用GUI控制界面 (`python3 ardusub_gui_controller.py`)
2. **ROS开发**: 使用ROS桥接模式进行集成开发
3. **兼容性**: 需要兼容原有系统时使用UDP模式

## 🤝 与原系统的兼容性

- ROS桥接节点同时支持ROS话题和UDP协议
- 可以与原有UDP客户端无缝配合
- 状态数据格式保持一致

键盘控制脚本 ──┐
              ├── UDP:9999 ──→ connect_main.py ──→ ArduSub飞控
状态接收脚本 ──┘         ←── UDP:8888 ──┘
