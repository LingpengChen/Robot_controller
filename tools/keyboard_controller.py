# -*- coding: utf-8 -*-
import socket
import keyboard
import time
import threading
import os
import sys

class ArduSubKeyboardController:
    def __init__(self, host='127.0.0.1', port=9999):
        """
        ArduSub键盘控制器
        host: 目标地址
        port: 目标端口 (与UDP代理的监听端口对应)
        """
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 控制状态
        self.is_armed = False
        self.current_mode = "手动"
        
        print("ArduSub键盘控制器已启动!")
        print("控制说明:")
        print("=" * 50)
        print("基础控制:")
        print("  W/S: 前进/后退")
        print("  A/D: 左平移/右平移") 
        print("  Q/E: 上浮/下潜")
        print("  J/L: 左转/右转")
        print("  I/K: 抬头/低头")
        print("  U/O: 左倾/右倾")
        print()
        print("模式切换:")
        print("  1: 手动模式")
        print("  2: 自稳模式")
        print("  3: 定深模式")
        print()
        print("设备控制:")
        print("  F: 机械手合")
        print("  G: 机械手张")
        print("  T: 主灯开关")
        print("  Y: 光圈灯开关")
        print()
        print("系统控制:")
        print("  Space: 解锁/锁定")
        print("  H: 心跳包")
        print("  ESC: 退出")
        print("=" * 50)
        
    def send_command(self, cmd_type, sub_cmd, value=128):
        """
        发送控制指令
        cmd_type: 命令类型
        sub_cmd: 子命令
        value: 控制值 (0-255, 128为中位)
        """
        # 构建数据包: [0xff, 0xfe, 0xfd, 0xfc, cmd_type, sub_cmd, value]
        packet = bytes([0xff, 0xfe, 0xfd, 0xfc, cmd_type, sub_cmd, value])
        self.sock.sendto(packet, (self.host, self.port))
        
    def send_heartbeat(self):
        """发送心跳包"""
        self.send_command(0x0a, 0x00, 0x00)
        
    def toggle_arm(self):
        """切换解锁/锁定状态"""
        arm_value = 0x00 if self.is_armed else 0x01
        self.send_command(0x0b, 0xfb, arm_value)
        self.is_armed = not self.is_armed
        status = "解锁" if self.is_armed else "锁定"
        print(f"电机状态: {status}")
        
    def set_mode(self, mode):
        """设置飞行模式"""
        mode_map = {
            1: (0x07, "手动模式"),
            2: (0x08, "自稳模式"),
            3: (0x09, "定深模式")
        }
        if mode in mode_map:
            cmd_value, mode_name = mode_map[mode]
            self.send_command(0x0a, cmd_value, 0x00)
            self.current_mode = mode_name
            print(f"切换到: {mode_name}")
            
    def control_movement(self, channel, direction):
        """
        控制运动
        channel: 控制通道 (1-6)
        direction: 方向 (-1=反向, 0=停止, 1=正向)
        """
        # 计算PWM值: 中位128, 范围约为50-200
        if direction == 0:
            value = 128  # 停止
        elif direction > 0:
            value = 180  # 正向
        else:
            value = 76   # 反向
            
        channel_map = {
            1: 0x05,  # 俯仰
            2: 0x06,  # 横滚
            3: 0x01,  # 垂直(上浮/下潜)
            4: 0x04,  # 偏航
            5: 0x03,  # 前后
            6: 0x02   # 左右
        }
        
        if channel in channel_map:
            self.send_command(0x0b, channel_map[channel], value)
            
    def control_device(self, device, action):
        """控制设备"""
        if device == "gripper":
            # 机械手控制
            if action == "close":
                self.send_command(0x0b, 0x0A, 1)
                print("机械手: 合")
            elif action == "open":
                self.send_command(0x0b, 0x0A, 2)
                print("机械手: 张")
        elif device == "light_main":
            # 主灯控制 (切换)
            self.send_command(0x0b, 0x0C, 2)
            print("主灯: 切换")
        elif device == "light_circle":
            # 光圈灯控制 (切换)
            self.send_command(0x0b, 0x0D, 2)
            print("光圈灯: 切换")
            
    def start_control(self):
        """启动键盘控制"""
        print("键盘控制已激活，按ESC退出")
        
        # 定期发送心跳包
        def heartbeat_loop():
            while True:
                self.send_heartbeat()
                time.sleep(1)
                
        heartbeat_thread = threading.Thread(target=heartbeat_loop)
        heartbeat_thread.daemon = True
        heartbeat_thread.start()
        
        try:
            while True:
                # 基础运动控制
                if keyboard.is_pressed('w'):
                    self.control_movement(5, 1)  # 前进
                elif keyboard.is_pressed('s'):
                    self.control_movement(5, -1)  # 后退
                elif keyboard.is_pressed('a'):
                    self.control_movement(6, -1)  # 左平移
                elif keyboard.is_pressed('d'):
                    self.control_movement(6, 1)   # 右平移
                elif keyboard.is_pressed('q'):
                    self.control_movement(3, 1)   # 上浮
                elif keyboard.is_pressed('e'):
                    self.control_movement(3, -1)  # 下潜
                elif keyboard.is_pressed('j'):
                    self.control_movement(4, -1)  # 左转
                elif keyboard.is_pressed('l'):
                    self.control_movement(4, 1)   # 右转
                elif keyboard.is_pressed('i'):
                    self.control_movement(1, 1)   # 抬头
                elif keyboard.is_pressed('k'):
                    self.control_movement(1, -1)  # 低头
                elif keyboard.is_pressed('u'):
                    self.control_movement(2, -1)  # 左倾
                elif keyboard.is_pressed('o'):
                    self.control_movement(2, 1)   # 右倾
                else:
                    # 如果没有按键，发送停止指令
                    for channel in range(1, 7):
                        self.control_movement(channel, 0)
                
                # 模式切换
                if keyboard.is_pressed('1'):
                    self.set_mode(1)
                    time.sleep(0.5)  # 避免重复触发
                elif keyboard.is_pressed('2'):
                    self.set_mode(2)
                    time.sleep(0.5)
                elif keyboard.is_pressed('3'):
                    self.set_mode(3)
                    time.sleep(0.5)
                
                # 设备控制
                if keyboard.is_pressed('f'):
                    self.control_device("gripper", "close")
                    time.sleep(0.5)
                elif keyboard.is_pressed('g'):
                    self.control_device("gripper", "open")
                    time.sleep(0.5)
                elif keyboard.is_pressed('t'):
                    self.control_device("light_main", "toggle")
                    time.sleep(0.5)
                elif keyboard.is_pressed('y'):
                    self.control_device("light_circle", "toggle")
                    time.sleep(0.5)
                
                # 系统控制
                if keyboard.is_pressed('space'):
                    self.toggle_arm()
                    time.sleep(0.5)
                elif keyboard.is_pressed('h'):
                    self.send_heartbeat()
                    print("发送心跳包")
                    time.sleep(0.5)
                elif keyboard.is_pressed('esc'):
                    print("退出控制器")
                    break
                    
                time.sleep(0.05)  # 控制循环频率
                
        except KeyboardInterrupt:
            print("控制器已停止")
        except Exception as e:
            print(f"控制器错误: {e}")
        finally:
            self.sock.close()

if __name__ == '__main__':
    # 检查是否以管理员权限运行 (Linux下需要监听键盘)
    if os.name == 'posix':
        try:
            # 尝试创建控制器
            controller = ArduSubKeyboardController()
            controller.start_control()
        except PermissionError:
            print("需要管理员权限来监听键盘输入")
            print("请使用: sudo python3 keyboard_controller.py")
    else:
        controller = ArduSubKeyboardController()
        controller.start_control()
