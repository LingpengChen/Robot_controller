#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
import time
import struct

class ArduSubGUIController:
    def __init__(self):
        # UDP连接设置
        self.host = '127.0.0.1'
        self.port = 9999
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 状态接收
        self.status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_port = 8888
        
        # 控制状态
        self.armed = False
        self.current_mode = "手动"
        
        # 状态数据
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.depth = 0.0
        self.temperature = 0.0
        
        self.setup_gui()
        self.start_status_receiver()
        self.start_heartbeat()
        
    def setup_gui(self):
        """设置GUI界面"""
        self.root = tk.Tk()
        self.root.title("ArduSub控制面板")
        self.root.geometry("800x600")
        
        # 创建主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 状态显示区域
        self.setup_status_panel(main_frame)
        
        # 控制区域
        self.setup_control_panel(main_frame)
        
        # 键盘绑定
        self.setup_keyboard_bindings()
        
        # 使窗口获得焦点
        self.root.focus_set()
        
    def setup_status_panel(self, parent):
        """设置状态显示面板"""
        status_frame = ttk.LabelFrame(parent, text="状态信息")
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 状态标签
        self.status_labels = {}
        
        row = 0
        for label_text, key in [
            ("俯仰角", "pitch"), ("横滚角", "roll"), ("偏航角", "yaw"),
            ("深度", "depth"), ("温度", "temperature"), ("状态", "armed"),
            ("模式", "mode")
        ]:
            ttk.Label(status_frame, text=f"{label_text}:").grid(row=row//2, column=(row%2)*2, sticky=tk.W, padx=5, pady=2)
            self.status_labels[key] = ttk.Label(status_frame, text="--", foreground="blue")
            self.status_labels[key].grid(row=row//2, column=(row%2)*2+1, sticky=tk.W, padx=5, pady=2)
            row += 1
            
    def setup_control_panel(self, parent):
        """设置控制面板"""
        control_frame = ttk.LabelFrame(parent, text="控制面板")
        control_frame.pack(fill=tk.BOTH, expand=True)
        
        # 创建两列布局
        left_frame = ttk.Frame(control_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        right_frame = ttk.Frame(control_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        
        # 运动控制
        self.setup_movement_controls(left_frame)
        
        # 模式和设备控制
        self.setup_mode_device_controls(right_frame)
        
        # 键盘说明
        self.setup_keyboard_help(control_frame)
        
    def setup_movement_controls(self, parent):
        """设置运动控制"""
        move_frame = ttk.LabelFrame(parent, text="运动控制")
        move_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 按钮网格布局
        buttons = [
            ("↑抬头(I)", 5, lambda: self.send_movement_cmd(0x05, 180), 1, 1),
            ("←左倾(U)", 6, lambda: self.send_movement_cmd(0x06, 76), 2, 0),
            ("→右倾(O)", 6, lambda: self.send_movement_cmd(0x06, 180), 2, 2),
            ("↓低头(K)", 5, lambda: self.send_movement_cmd(0x05, 76), 3, 1),
            
            ("↑前进(W)", 3, lambda: self.send_movement_cmd(0x03, 180), 1, 5),
            ("←左移(A)", 2, lambda: self.send_movement_cmd(0x02, 76), 2, 4),
            ("→右移(D)", 2, lambda: self.send_movement_cmd(0x02, 180), 2, 6),
            ("↓后退(S)", 3, lambda: self.send_movement_cmd(0x03, 76), 3, 5),
            
            ("↑上浮(Q)", 1, lambda: self.send_movement_cmd(0x01, 180), 1, 9),
            ("←左转(J)", 4, lambda: self.send_movement_cmd(0x04, 76), 2, 8),
            ("→右转(L)", 4, lambda: self.send_movement_cmd(0x04, 180), 2, 10),
            ("↓下潜(E)", 1, lambda: self.send_movement_cmd(0x01, 76), 3, 9),
        ]
        
        for text, channel, command, row, col in buttons:
            btn = ttk.Button(move_frame, text=text, command=command, width=10)
            btn.grid(row=row, column=col, padx=2, pady=2)
            
        # 停止按钮
        stop_btn = ttk.Button(move_frame, text="全部停止", command=self.stop_all_movement, width=15)
        stop_btn.grid(row=4, column=0, columnspan=11, pady=10)
        
    def setup_mode_device_controls(self, parent):
        """设置模式和设备控制"""
        # 模式控制
        mode_frame = ttk.LabelFrame(parent, text="模式控制")
        mode_frame.pack(fill=tk.X, pady=(0, 10))
        
        modes = [("手动模式(1)", "manual"), ("自稳模式(2)", "stabilize"), ("定深模式(3)", "depth_hold")]
        for i, (text, mode) in enumerate(modes):
            btn = ttk.Button(mode_frame, text=text, command=lambda m=mode: self.set_mode(m))
            btn.grid(row=0, column=i, padx=5, pady=5)
            
        # 系统控制
        system_frame = ttk.LabelFrame(parent, text="系统控制")
        system_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.arm_btn = ttk.Button(system_frame, text="解锁电机(Space)", command=self.toggle_arm)
        self.arm_btn.grid(row=0, column=0, padx=5, pady=5)
        
        heartbeat_btn = ttk.Button(system_frame, text="心跳(H)", command=self.send_heartbeat)
        heartbeat_btn.grid(row=0, column=1, padx=5, pady=5)
        
        # 设备控制
        device_frame = ttk.LabelFrame(parent, text="设备控制")
        device_frame.pack(fill=tk.X, pady=(0, 10))
        
        devices = [
            ("机械手合(F)", lambda: self.control_device(0x0A, 1)),
            ("机械手张(G)", lambda: self.control_device(0x0A, 2)),
            ("主灯切换(T)", lambda: self.control_device(0x0C, 2)),
            ("光圈灯切换(Y)", lambda: self.control_device(0x0D, 2))
        ]
        
        for i, (text, command) in enumerate(devices):
            btn = ttk.Button(device_frame, text=text, command=command)
            btn.grid(row=i//2, column=i%2, padx=5, pady=2, sticky=tk.W+tk.E)
            
    def setup_keyboard_help(self, parent):
        """设置键盘帮助"""
        help_frame = ttk.LabelFrame(parent, text="键盘快捷键")
        help_frame.pack(fill=tk.X, pady=10)
        
        help_text = """
运动: WASD(前后左右) QE(上下) JL(转向) IK(俯仰) UO(横滚)
模式: 1(手动) 2(自稳) 3(定深)  设备: F(夹合) G(夹开) T(主灯) Y(光圈)
系统: Space(解锁) H(心跳) ESC(退出)
        """
        
        help_label = ttk.Label(help_frame, text=help_text.strip(), justify=tk.LEFT)
        help_label.pack(padx=10, pady=5)
        
    def setup_keyboard_bindings(self):
        """设置键盘绑定"""
        # 运动控制
        movement_bindings = {
            'w': (0x03, 180), 's': (0x03, 76),  # 前后
            'a': (0x02, 76), 'd': (0x02, 180),  # 左右
            'q': (0x01, 180), 'e': (0x01, 76),  # 上下
            'j': (0x04, 76), 'l': (0x04, 180),  # 转向
            'i': (0x05, 180), 'k': (0x05, 76),  # 俯仰
            'u': (0x06, 76), 'o': (0x06, 180)   # 横滚
        }
        
        for key, (channel, value) in movement_bindings.items():
            self.root.bind(f'<KeyPress-{key}>', lambda e, c=channel, v=value: self.send_movement_cmd(c, v))
            self.root.bind(f'<KeyRelease-{key}>', lambda e, c=channel: self.send_movement_cmd(c, 128))
            
        # 模式控制
        self.root.bind('<KeyPress-1>', lambda e: self.set_mode("manual"))
        self.root.bind('<KeyPress-2>', lambda e: self.set_mode("stabilize"))
        self.root.bind('<KeyPress-3>', lambda e: self.set_mode("depth_hold"))
        
        # 设备控制
        self.root.bind('<KeyPress-f>', lambda e: self.control_device(0x0A, 1))
        self.root.bind('<KeyPress-g>', lambda e: self.control_device(0x0A, 2))
        self.root.bind('<KeyPress-t>', lambda e: self.control_device(0x0C, 2))
        self.root.bind('<KeyPress-y>', lambda e: self.control_device(0x0D, 2))
        
        # 系统控制
        self.root.bind('<KeyPress-space>', lambda e: self.toggle_arm())
        self.root.bind('<KeyPress-h>', lambda e: self.send_heartbeat())
        self.root.bind('<KeyPress-Escape>', lambda e: self.root.quit())
        
    def send_command(self, cmd_type, sub_cmd, value=128):
        """发送控制命令"""
        try:
            packet = bytes([0xff, 0xfe, 0xfd, 0xfc, cmd_type, sub_cmd, value])
            self.sock.sendto(packet, (self.host, self.port))
        except Exception as e:
            messagebox.showerror("错误", f"命令发送失败: {e}")
            
    def send_movement_cmd(self, channel, value):
        """发送运动控制命令"""
        self.send_command(0x0b, channel, value)
        
    def stop_all_movement(self):
        """停止所有运动"""
        for channel in [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]:
            self.send_command(0x0b, channel, 128)
            
    def set_mode(self, mode):
        """设置飞行模式"""
        mode_map = {
            "manual": 0x07,
            "stabilize": 0x08,
            "depth_hold": 0x09
        }
        
        if mode in mode_map:
            self.send_command(0x0a, mode_map[mode], 0x00)
            self.current_mode = mode
            self.status_labels["mode"].config(text=mode)
            
    def toggle_arm(self):
        """切换解锁状态"""
        arm_value = 0x00 if self.armed else 0x01
        self.send_command(0x0b, 0xfb, arm_value)
        self.armed = not self.armed
        
        status_text = "已解锁" if self.armed else "已锁定"
        color = "green" if self.armed else "red"
        self.status_labels["armed"].config(text=status_text, foreground=color)
        self.arm_btn.config(text=f"{'锁定' if self.armed else '解锁'}电机(Space)")
        
    def control_device(self, device_cmd, value):
        """控制设备"""
        self.send_command(0x0b, device_cmd, value)
        
    def send_heartbeat(self):
        """发送心跳"""
        self.send_command(0x0a, 0x00, 0x00)
        
    def start_status_receiver(self):
        """启动状态接收线程"""
        def receive_status():
            try:
                self.status_sock.bind(('127.0.0.1', self.status_port))
                while True:
                    data, addr = self.status_sock.recvfrom(1024)
                    self.parse_status_data(data)
            except Exception as e:
                print(f"状态接收错误: {e}")
                
        thread = threading.Thread(target=receive_status)
        thread.daemon = True
        thread.start()
        
    def parse_status_data(self, data):
        """解析状态数据"""
        try:
            if len(data) >= 19 and data[0:6] == bytes([0xfb, 0xfa, 0xf9, 0xf8, 0x15, 0x0f]):
                pitch_raw = struct.unpack(">H", data[6:8])[0]
                roll_raw = struct.unpack(">H", data[8:10])[0]
                yaw_raw = struct.unpack(">H", data[10:12])[0]
                depth_raw = struct.unpack(">H", data[12:14])[0]
                temp_raw = struct.unpack(">H", data[14:16])[0]
                status = data[16]
                
                self.pitch = (pitch_raw / 100.0) - 360.0
                self.roll = (roll_raw / 100.0) - 360.0
                self.yaw = yaw_raw / 100.0
                self.depth = depth_raw / 100.0
                self.temperature = temp_raw / 100.0
                self.armed = (status == 1)
                
                # 更新GUI显示
                self.root.after(0, self.update_status_display)
                
        except Exception as e:
            print(f"状态解析错误: {e}")
            
    def update_status_display(self):
        """更新状态显示"""
        self.status_labels["pitch"].config(text=f"{self.pitch:.1f}°")
        self.status_labels["roll"].config(text=f"{self.roll:.1f}°")
        self.status_labels["yaw"].config(text=f"{self.yaw:.1f}°")
        self.status_labels["depth"].config(text=f"{self.depth:.2f}m")
        self.status_labels["temperature"].config(text=f"{self.temperature:.1f}°C")
        
        status_text = "已解锁" if self.armed else "已锁定"
        color = "green" if self.armed else "red"
        self.status_labels["armed"].config(text=status_text, foreground=color)
        
    def start_heartbeat(self):
        """启动心跳线程"""
        def heartbeat_loop():
            while True:
                self.send_heartbeat()
                time.sleep(1)
                
        thread = threading.Thread(target=heartbeat_loop)
        thread.daemon = True
        thread.start()
        
    def run(self):
        """运行GUI"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.sock.close()
            self.status_sock.close()

if __name__ == '__main__':
    try:
        controller = ArduSubGUIController()
        controller.run()
    except Exception as e:
        print(f"GUI控制器错误: {e}")
        messagebox.showerror("错误", f"控制器启动失败: {e}")
