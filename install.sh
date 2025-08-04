#!/bin/bash

# ArduSub ROS Package 安装脚本

echo "=== ArduSub ROS Package 安装脚本 ==="

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: 未检测到ROS环境变量，请先source ROS setup.bash文件"
    exit 1
fi

echo "检测到ROS版本: $ROS_DISTRO"

# 安装ROS依赖
echo "正在安装ROS依赖包..."
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-mavros \
    ros-$ROS_DISTRO-mavros-extras \
    ros-$ROS_DISTRO-geographic-msgs

# 安装GeographicLib数据集
echo "正在安装GeographicLib数据集..."
sudo /opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh

# 安装Python依赖
echo "正在安装Python依赖..."
pip3 install pymavlink

# 检查catkin工作空间
if [ ! -d "$HOME/catkin_ws" ]; then
    echo "创建catkin工作空间..."
    mkdir -p $HOME/catkin_ws/src
    cd $HOME/catkin_ws
    catkin_make
fi

# 复制包到工作空间
PACKAGE_PATH="$HOME/catkin_ws/src/ardusub_control"
if [ ! -d "$PACKAGE_PATH" ]; then
    echo "复制包到catkin工作空间..."
    cp -r $(pwd) $PACKAGE_PATH
else
    echo "更新现有包..."
    cp -r $(pwd)/* $PACKAGE_PATH/
fi

# 编译包
echo "编译ROS包..."
cd $HOME/catkin_ws
catkin_make

# 设置环境变量
echo "设置环境变量..."
if ! grep -q "source $HOME/catkin_ws/devel/setup.bash" ~/.bashrc; then
    echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
fi

source $HOME/catkin_ws/devel/setup.bash

echo "=== 安装完成! ==="
echo ""
echo "使用方法:"
echo "1. 启动ROS核心: roscore"
echo "2. 启动ArduSub节点: roslaunch ardusub_control ardusub_control.launch"
echo "3. 启动控制客户端: rosrun ardusub_control ardusub_controller.py"
echo ""
echo "更多信息请查看README.md文件"
