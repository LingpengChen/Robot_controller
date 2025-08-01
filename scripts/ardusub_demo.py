# -*- coding: utf-8 -*-
"""
ArduSub控制系统使用示例

运行步骤:
1. 确保ArduSub已连接并运行connect_main.py
2. 运行此脚本开始控制和监控
"""

import threading
import time
import sys
import sys
import os
# 添加工具和源码路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'tools'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from status_receiver import StatusReceiver
from keyboard_controller import ArduSubKeyboardController

def main():
    print("ArduSub控制系统启动中...")
    print("=" * 60)
    
    # 创建状态接收器
    print("启动状态监控...")
    status_receiver = StatusReceiver()
    status_receiver.start_receiving()
    
    # 等待状态接收器启动
    time.sleep(1)
    
    # 创建键盘控制器
    print("启动键盘控制器...")
    controller = ArduSubKeyboardController()
    
    # 启动状态监控线程
    def status_monitor():
        while True:
            try:
                status_receiver.print_status()
                time.sleep(2)  # 每2秒打印一次状态
            except:
                break
    
    status_thread = threading.Thread(target=status_monitor)
    status_thread.daemon = True
    status_thread.start()
    
    print("系统就绪！")
    print("=" * 60)
    
    try:
        # 启动键盘控制
        controller.start_control()
    except KeyboardInterrupt:
        print("\n正在关闭系统...")
    except Exception as e:
        print(f"系统错误: {e}")
    finally:
        status_receiver.stop()
        print("系统已关闭")

if __name__ == '__main__':
    main()
