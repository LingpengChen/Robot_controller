#!/bin/bash

# ArduSub控制系统安装脚本

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}╔═══════════════════════════════════════╗${NC}"
echo -e "${BLUE}║      ArduSub控制系统安装器             ║${NC}"
echo -e "${BLUE}╚═══════════════════════════════════════╝${NC}"
echo

# 检查系统
check_system() {
    echo -e "${BLUE}检查系统环境...${NC}"
    
    # 检查操作系统
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        echo -e "${GREEN}✓ Linux系统${NC}"
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        echo -e "${GREEN}✓ macOS系统${NC}"
    else
        echo -e "${YELLOW}! 未测试的系统: $OSTYPE${NC}"
    fi
    
    # 检查Python
    if command -v python3 &> /dev/null; then
        PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
        echo -e "${GREEN}✓ Python3: $PYTHON_VERSION${NC}"
    else
        echo -e "${RED}✗ Python3 未安装${NC}"
        exit 1
    fi
    
    # 检查pip
    if command -v pip3 &> /dev/null; then
        echo -e "${GREEN}✓ pip3 已安装${NC}"
    else
        echo -e "${RED}✗ pip3 未安装${NC}"
        exit 1
    fi
    
    echo
}

# 安装Python依赖
install_python_deps() {
    echo -e "${BLUE}安装Python依赖包...${NC}"
    
    if [ -f "$PACKAGE_DIR/requirements.txt" ]; then
        pip3 install -r "$PACKAGE_DIR/requirements.txt"
        echo -e "${GREEN}✓ Python依赖安装完成${NC}"
    else
        echo -e "${YELLOW}requirements.txt 不存在，手动安装核心依赖...${NC}"
        pip3 install pymavlink pynput PyYAML numpy
    fi
    
    echo
}

# 安装系统依赖
install_system_deps() {
    echo -e "${BLUE}安装系统依赖...${NC}"
    
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        # Ubuntu/Debian
        if command -v apt-get &> /dev/null; then
            echo -e "${YELLOW}检测到apt包管理器，安装tkinter...${NC}"
            sudo apt-get update
            sudo apt-get install -y python3-tk
            echo -e "${GREEN}✓ 系统依赖安装完成${NC}"
        # CentOS/RHEL
        elif command -v yum &> /dev/null; then
            echo -e "${YELLOW}检测到yum包管理器，安装tkinter...${NC}"
            sudo yum install -y tkinter
            echo -e "${GREEN}✓ 系统依赖安装完成${NC}"
        else
            echo -e "${YELLOW}! 未知的包管理器，请手动安装python3-tk${NC}"
        fi
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS
        echo -e "${GREEN}✓ macOS通常自带tkinter${NC}"
    fi
    
    echo
}

# 检查ROS环境
check_ros() {
    echo -e "${BLUE}检查ROS环境...${NC}"
    
    if [ -n "$ROS_DISTRO" ]; then
        echo -e "${GREEN}✓ ROS $ROS_DISTRO 环境已设置${NC}"
        
        # 检查roscore
        if command -v roscore &> /dev/null; then
            echo -e "${GREEN}✓ ROS核心工具可用${NC}"
        else
            echo -e "${YELLOW}! ROS工具可能未正确安装${NC}"
        fi
    else
        echo -e "${YELLOW}! ROS环境未设置${NC}"
        echo -e "${BLUE}如需使用ROS功能，请运行:${NC}"
        echo -e "  source /opt/ros/noetic/setup.bash"
        echo -e "  # 或其他ROS版本"
    fi
    
    echo
}

# 设置权限
setup_permissions() {
    echo -e "${BLUE}设置文件权限...${NC}"
    
    # 设置脚本可执行权限
    chmod +x "$PACKAGE_DIR"/scripts/*.py
    chmod +x "$PACKAGE_DIR"/tools/*.py
    chmod +x "$PACKAGE_DIR"/config/*.sh
    
    echo -e "${GREEN}✓ 权限设置完成${NC}"
    echo
}

# 创建工作空间 (如果在ROS环境中)
setup_workspace() {
    if [ -n "$ROS_DISTRO" ]; then
        echo -e "${BLUE}配置ROS工作空间...${NC}"
        
        # 获取工作空间根目录
        WORKSPACE_ROOT=$(echo "$PACKAGE_DIR" | sed 's|/src/.*||')
        
        if [ -f "$WORKSPACE_ROOT/devel/setup.bash" ]; then
            echo -e "${GREEN}✓ 工作空间已构建${NC}"
        else
            echo -e "${YELLOW}构建ROS工作空间...${NC}"
            cd "$WORKSPACE_ROOT"
            catkin_make
            echo -e "${GREEN}✓ 工作空间构建完成${NC}"
        fi
        
        echo -e "${BLUE}要使用ROS功能，请运行:${NC}"
        echo -e "  source $WORKSPACE_ROOT/devel/setup.bash"
        echo
    fi
}

# 创建桌面快捷方式 (可选)
create_shortcuts() {
    echo -e "${BLUE}是否创建桌面快捷方式？ [y/N]${NC}"
    read -r create_shortcut
    
    if [[ $create_shortcut =~ ^[Yy]$ ]]; then
        DESKTOP_DIR="$HOME/Desktop"
        if [ -d "$DESKTOP_DIR" ]; then
            cat > "$DESKTOP_DIR/ArduSub控制器.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=ArduSub控制器
Comment=ArduSub水下机器人控制系统
Exec=$PACKAGE_DIR/config/start_ardusub.sh
Icon=applications-engineering
Terminal=true
Categories=Development;Science;
EOF
            chmod +x "$DESKTOP_DIR/ArduSub控制器.desktop"
            echo -e "${GREEN}✓ 桌面快捷方式已创建${NC}"
        else
            echo -e "${YELLOW}! 桌面目录不存在${NC}"
        fi
    fi
    
    echo
}

# 运行测试
run_tests() {
    echo -e "${BLUE}运行基础测试...${NC}"
    
    # 测试导入
    echo -e "${BLUE}测试Python模块导入...${NC}"
    python3 -c "
import sys
sys.path.append('$PACKAGE_DIR/src')

try:
    import pymavlink
    print('✓ pymavlink')
except ImportError as e:
    print('✗ pymavlink:', e)

try:
    import pynput
    print('✓ pynput')
except ImportError as e:
    print('✗ pynput:', e)

try:
    import yaml
    print('✓ PyYAML')
except ImportError as e:
    print('✗ PyYAML:', e)

try:
    import tkinter
    print('✓ tkinter')
except ImportError as e:
    print('✗ tkinter:', e)
"
    
    echo -e "${GREEN}✓ 基础测试完成${NC}"
    echo
}

# 显示安装完成信息
show_completion() {
    echo -e "${GREEN}╔═══════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║           安装完成！                   ║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════╝${NC}"
    echo
    echo -e "${BLUE}快速启动:${NC}"
    echo -e "  cd $PACKAGE_DIR"
    echo -e "  ./config/start_ardusub.sh"
    echo
    echo -e "${BLUE}或者直接启动GUI:${NC}"
    echo -e "  cd $PACKAGE_DIR/scripts"
    echo -e "  python3 ardusub_gui_controller.py"
    echo
    echo -e "${BLUE}如需ROS功能，请先source环境:${NC}"
    echo -e "  source /opt/ros/\$ROS_DISTRO/setup.bash"
    if [ -n "$ROS_DISTRO" ]; then
        WORKSPACE_ROOT=$(echo "$PACKAGE_DIR" | sed 's|/src/.*||')
        echo -e "  source $WORKSPACE_ROOT/devel/setup.bash"
    fi
    echo
}

# 主安装流程
main() {
    check_system
    install_system_deps
    install_python_deps
    check_ros
    setup_permissions
    setup_workspace
    create_shortcuts
    run_tests
    show_completion
    
    echo -e "${GREEN}安装程序完成！${NC}"
}

# 处理命令行参数
case "${1:-}" in
    --deps-only)
        install_python_deps
        install_system_deps
        ;;
    --test-only)
        run_tests
        ;;
    --no-shortcuts)
        check_system
        install_system_deps
        install_python_deps
        check_ros
        setup_permissions
        setup_workspace
        run_tests
        show_completion
        ;;
    *)
        main
        ;;
esac
