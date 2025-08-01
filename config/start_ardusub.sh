#!/bin/bash

#!/bin/bash

# ArduSub控制系统启动脚本 (重构版)
# 支持多种启动模式和配置选项

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
SCRIPTS_DIR="$PACKAGE_DIR/scripts"
TOOLS_DIR="$PACKAGE_DIR/tools"

# 检查Python依赖
check_python_deps() {
    echo -e "${BLUE}检查Python依赖...${NC}"
    
    local missing_deps=()
    
    # 检查基础依赖
    python3 -c "import pymavlink" 2>/dev/null || missing_deps+=("pymavlink")
    python3 -c "import pynput" 2>/dev/null || missing_deps+=("pynput")
    python3 -c "import yaml" 2>/dev/null || missing_deps+=("pyyaml")
    python3 -c "import tkinter" 2>/dev/null || missing_deps+=("python3-tk")
    
    # 检查ROS依赖
    if [ "$1" = "ros" ]; then
        python3 -c "import rospy" 2>/dev/null || missing_deps+=("ROS环境")
    fi
    
    if [ ${#missing_deps[@]} -ne 0 ]; then
        echo -e "${RED}缺少以下依赖:${NC}"
        for dep in "${missing_deps[@]}"; do
            echo -e "  - $dep"
        done
        echo -e "
${YELLOW}安装命令:${NC}"
        echo "pip3 install pymavlink pynput pyyaml"
        echo "sudo apt-get install python3-tk"
        [ "$1" = "ros" ] && echo "source /opt/ros/\$ROS_DISTRO/setup.bash"
        return 1
    fi
    
    echo -e "${GREEN}依赖检查通过${NC}"
    return 0
}

# 检查ROS环境
check_ros_env() {
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${RED}ROS环境未设置${NC}"
        echo -e "${YELLOW}请先运行: source /opt/ros/noetic/setup.bash${NC}"
        return 1
    fi
    
    if ! command -v roscore &> /dev/null; then
        echo -e "${RED}ROS未正确安装${NC}"
        return 1
    fi
    
    echo -e "${GREEN}ROS环境检查通过 (ROS $ROS_DISTRO)${NC}"
    return 0
}

# 启动GUI控制器
start_gui() {
    echo -e "${GREEN}启动GUI控制器...${NC}"
    check_python_deps "gui" || return 1
    
    cd "$SCRIPTS_DIR"
    python3 ardusub_gui_controller.py
}

# 启动ROS桥接
start_ros_bridge() {
    echo -e "${GREEN}启动ROS桥接节点...${NC}"
    check_ros_env || return 1
    check_python_deps "ros" || return 1
    
    # 检查roscore是否运行
    if ! pgrep -f rosmaster > /dev/null; then
        echo -e "${YELLOW}roscore未运行，启动roscore...${NC}"
        roscore &
        sleep 3
    fi
    
    cd "$SCRIPTS_DIR"
    python3 ardusub_ros_bridge.py
}

# 启动键盘控制 (ROS版本)
start_keyboard_ros() {
    echo -e "${GREEN}启动键盘控制 (ROS版本)...${NC}"
    check_ros_env || return 1
    check_python_deps "ros" || return 1
    
    cd "$SCRIPTS_DIR"
    python3 ardusub_keyboard_ros.py
}

# 启动键盘控制 (UDP版本)
start_keyboard_udp() {
    echo -e "${GREEN}启动键盘控制 (UDP版本)...${NC}"
    check_python_deps "udp" || return 1
    
    cd "$TOOLS_DIR"
    python3 keyboard_controller.py
}

# 启动完整ROS系统
start_full_ros() {
    echo -e "${GREEN}启动完整ROS系统...${NC}"
    check_ros_env || return 1
    check_python_deps "ros" || return 1
    
    # 使用roslaunch启动
    roslaunch ardusub_control ardusub_control.launch
}

# 启动UDP主程序
start_udp_main() {
    echo -e "${GREEN}启动UDP主程序...${NC}"
    check_python_deps "udp" || return 1
    
    cd "$TOOLS_DIR"
    python3 connect_main.py
}

# 启动状态监控
start_status_monitor() {
    echo -e "${GREEN}启动状态监控...${NC}"
    check_python_deps "udp" || return 1
    
    cd "$TOOLS_DIR"
    python3 status_receiver.py
}

# 启动演示程序
start_demo() {
    echo -e "${GREEN}启动演示程序...${NC}"
    check_python_deps "demo" || return 1
    
    cd "$SCRIPTS_DIR"
    python3 ardusub_demo.py
}

# 显示系统状态
show_status() {
    echo -e "${BLUE}=== ArduSub控制系统状态 ===${NC}"
    echo -e "包目录: $PACKAGE_DIR"
    echo -e "脚本目录: $SCRIPTS_DIR"
    echo -e "工具目录: $TOOLS_DIR"
    echo
    
    # 检查进程状态
    echo -e "${BLUE}运行中的进程:${NC}"
    pgrep -f "ardusub" | while read pid; do
        echo -e "  PID $pid: $(ps -p $pid -o comm=)"
    done
    
    # 检查ROS
    if [ -n "$ROS_DISTRO" ]; then
        echo -e "
${BLUE}ROS状态:${NC}"
        echo -e "  ROS版本: $ROS_DISTRO"
        if pgrep -f rosmaster > /dev/null; then
            echo -e "  roscore: ${GREEN}运行中${NC}"
        else
            echo -e "  roscore: ${RED}未运行${NC}"
        fi
    fi
}

# 停止所有进程
stop_all() {
    echo -e "${YELLOW}停止所有ArduSub进程...${NC}"
    pkill -f ardusub
    pkill -f "python.*ardusub"
    echo -e "${GREEN}已停止所有进程${NC}"
}

# 显示主菜单
show_menu() {
    echo -e "${BLUE}╔═══════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║         ArduSub控制系统启动器           ║${NC}"
    echo -e "${BLUE}║              (重构版 v1.0)            ║${NC}"
    echo -e "${BLUE}╚═══════════════════════════════════════╝${NC}"
    echo
    echo -e "${GREEN}推荐启动方式:${NC}"
    echo -e "  ${YELLOW}1)${NC} GUI控制器 ${BLUE}(最简单，推荐新手)${NC}"
    echo -e "  ${YELLOW}2)${NC} 完整ROS系统 ${BLUE}(roslaunch方式)${NC}"
    echo
    echo -e "${GREEN}ROS模式:${NC}"
    echo -e "  ${YELLOW}3)${NC} ROS桥接节点"
    echo -e "  ${YELLOW}4)${NC} 键盘控制 (ROS版本)"
    echo
    echo -e "${GREEN}UDP兼容模式:${NC}"
    echo -e "  ${YELLOW}5)${NC} UDP主程序"
    echo -e "  ${YELLOW}6)${NC} 键盘控制 (UDP版本)"
    echo -e "  ${YELLOW}7)${NC} 状态监控器"
    echo
    echo -e "${GREEN}工具和演示:${NC}"
    echo -e "  ${YELLOW}8)${NC} 演示程序"
    echo -e "  ${YELLOW}9)${NC} 系统状态"
    echo
    echo -e "${GREEN}系统管理:${NC}"
    echo -e "  ${YELLOW}s)${NC} 停止所有进程"
    echo -e "  ${YELLOW}q)${NC} 退出"
    echo
}

# 主程序循环
main() {
    while true; do
        show_menu
        echo -n -e "${YELLOW}请选择启动模式 [1-9/s/q]: ${NC}"
        read -r choice
        echo
        
        case $choice in
            1)
                start_gui
                ;;
            2)
                start_full_ros
                ;;
            3)
                start_ros_bridge
                ;;
            4)
                start_keyboard_ros
                ;;
            5)
                start_udp_main
                ;;
            6)
                start_keyboard_udp
                ;;
            7)
                start_status_monitor
                ;;
            8)
                start_demo
                ;;
            9)
                show_status
                ;;
            s|S)
                stop_all
                ;;
            q|Q)
                echo -e "${GREEN}退出启动器${NC}"
                break
                ;;
            *)
                echo -e "${RED}无效选择，请重新输入${NC}"
                ;;
        esac
        
        echo
        echo -e "${YELLOW}按回车键继续...${NC}"
        read -r
        clear
    done
}

# 处理命令行参数
if [ $# -eq 0 ]; then
    # 无参数时显示交互菜单
    clear
    main
else
    # 有参数时直接执行对应功能
    case $1 in
        gui)
            start_gui
            ;;
        ros)
            start_full_ros
            ;;
        bridge)
            start_ros_bridge
            ;;
        keyboard-ros)
            start_keyboard_ros
            ;;
        keyboard-udp)
            start_keyboard_udp
            ;;
        udp)
            start_udp_main
            ;;
        status)
            start_status_monitor
            ;;
        demo)
            start_demo
            ;;
        stop)
            stop_all
            ;;
        info)
            show_status
            ;;
        *)
            echo -e "${RED}未知参数: $1${NC}"
            echo -e "${YELLOW}可用参数: gui, ros, bridge, keyboard-ros, keyboard-udp, udp, status, demo, stop, info${NC}"
            exit 1
            ;;
    esac
fi

echo "ArduSub ROS控制系统启动脚本"
echo "================================"

# 检查ROS环境
if [ -z "$ROS_MASTER_URI" ]; then
    echo "警告: ROS环境未设置，正在设置默认环境..."
    export ROS_MASTER_URI=http://localhost:11311
    export ROS_HOSTNAME=localhost
fi

echo "ROS Master URI: $ROS_MASTER_URI"

# 创建启动菜单
echo "请选择启动模式:"
echo "1. ROS桥接模式 (推荐)"
echo "2. GUI控制界面 (无需sudo)"
echo "3. 原始UDP模式"
echo "4. 状态监控"
echo "5. 全部启动"
echo "0. 退出"

read -p "请输入选择 (0-5): " choice

case $choice in
    1)
        echo "启动ROS桥接模式..."
        python3 ardusub_ros_bridge.py
        ;;
    2)
        echo "启动GUI控制界面..."
        python3 ardusub_gui_controller.py
        ;;
    3)
        echo "启动原始UDP模式..."
        python3 connect_main.py
        ;;
    4)
        echo "启动状态监控..."
        python3 status_receiver.py
        ;;
    5)
        echo "启动全部服务..."
        echo "1. 启动ROS桥接..."
        python3 ardusub_ros_bridge.py &
        sleep 2
        
        echo "2. 启动GUI控制..."
        python3 ardusub_gui_controller.py &
        
        echo "所有服务已启动，按Ctrl+C停止"
        wait
        ;;
    0)
        echo "退出"
        exit 0
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac
