#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from pynput import keyboard
import threading
import time

class ArduSubKeyboardController:
    def __init__(self):
        rospy.init_node('ardusub_keyboard_controller', anonymous=True)
        
        # ROS发布者
        self.cmd_pub = rospy.Publisher('/ardusub/cmd_vel', Twist, queue_size=1)
        self.mode_pub = rospy.Publisher('/ardusub/set_mode', String, queue_size=1)
        self.arm_pub = rospy.Publisher('/ardusub/arm', Bool, queue_size=1)
        self.device_pub = rospy.Publisher('/ardusub/device_cmd', String, queue_size=1)
        
        # 控制状态
        self.current_twist = Twist()
        self.armed = False
        self.current_mode = "manual"
        
        # 按键状态
        self.pressed_keys = set()
        
        # 控制参数
        self.max_velocity = 1.0  # 最大速度
        self.velocity_step = 0.1  # 速度步长
        
        self.print_instructions()
        
        # 启动键盘监听
        self.start_keyboard_listener()
        
        # 启动控制循环
        self.start_control_loop()
        
        rospy.loginfo("ArduSub键盘控制器已启动 (无需sudo权限)")
        
    def print_instructions(self):
        """打印控制说明"""
        print("\n" + "="*60)
        print("ArduSub键盘控制器 (基于pynput，无需sudo权限)")
        print("="*60)
        print("运动控制:")
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
        print("  H: 发送心跳")
        print("  ESC: 退出")
        print("="*60)
        print("注意: 请确保此窗口获得焦点以接收键盘输入")
        print("="*60)
        
    def start_keyboard_listener(self):
        """启动键盘监听器"""
        self.listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )
        self.listener.start()
        
    def on_key_press(self, key):
        """按键按下事件"""
        try:
            # 获取按键字符
            if hasattr(key, 'char') and key.char:
                key_char = key.char.lower()
                self.pressed_keys.add(key_char)
                
                # 立即处理的按键
                if key_char == '1':
                    self.set_flight_mode("manual")
                elif key_char == '2':
                    self.set_flight_mode("stabilize")
                elif key_char == '3':
                    self.set_flight_mode("depth_hold")
                elif key_char == 'f':
                    self.control_device("gripper", "close")
                elif key_char == 'g':
                    self.control_device("gripper", "open")
                elif key_char == 't':
                    self.control_device("light_main", "toggle")
                elif key_char == 'y':
                    self.control_device("light_circle", "toggle")
                elif key_char == 'h':
                    self.send_heartbeat()
                    
            # 处理特殊键
            elif key == keyboard.Key.space:
                self.toggle_arm()
            elif key == keyboard.Key.esc:
                rospy.loginfo("退出键盘控制器")
                rospy.signal_shutdown("用户请求退出")
                return False
                
        except Exception as e:
            rospy.logwarn(f"按键处理错误: {e}")
            
    def on_key_release(self, key):
        """按键释放事件"""
        try:
            if hasattr(key, 'char') and key.char:
                key_char = key.char.lower()
                self.pressed_keys.discard(key_char)
        except:
            pass
            
    def start_control_loop(self):
        """启动控制循环"""
        def control_loop():
            rate = rospy.Rate(20)  # 20Hz
            
            while not rospy.is_shutdown():
                self.update_movement()
                self.cmd_pub.publish(self.current_twist)
                rate.sleep()
                
        self.control_thread = threading.Thread(target=control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
    def update_movement(self):
        """根据当前按键状态更新运动命令"""
        twist = Twist()
        
        # 线性运动
        if 'w' in self.pressed_keys:
            twist.linear.x = self.max_velocity
        elif 's' in self.pressed_keys:
            twist.linear.x = -self.max_velocity
            
        if 'd' in self.pressed_keys:
            twist.linear.y = self.max_velocity
        elif 'a' in self.pressed_keys:
            twist.linear.y = -self.max_velocity
            
        if 'q' in self.pressed_keys:
            twist.linear.z = self.max_velocity
        elif 'e' in self.pressed_keys:
            twist.linear.z = -self.max_velocity
            
        # 角运动
        if 'l' in self.pressed_keys:
            twist.angular.z = self.max_velocity
        elif 'j' in self.pressed_keys:
            twist.angular.z = -self.max_velocity
            
        if 'i' in self.pressed_keys:
            twist.angular.y = self.max_velocity
        elif 'k' in self.pressed_keys:
            twist.angular.y = -self.max_velocity
            
        if 'o' in self.pressed_keys:
            twist.angular.x = self.max_velocity
        elif 'u' in self.pressed_keys:
            twist.angular.x = -self.max_velocity
            
        self.current_twist = twist
        
    def set_flight_mode(self, mode):
        """设置飞行模式"""
        self.current_mode = mode
        self.mode_pub.publish(String(mode))
        rospy.loginfo(f"切换到{mode}模式")
        
    def toggle_arm(self):
        """切换解锁状态"""
        self.armed = not self.armed
        self.arm_pub.publish(Bool(self.armed))
        status = "解锁" if self.armed else "锁定"
        rospy.loginfo(f"电机{status}")
        
    def control_device(self, device, action):
        """控制设备"""
        command = f"{device}:{action}"
        self.device_pub.publish(String(command))
        rospy.loginfo(f"设备控制: {device} {action}")
        
    def send_heartbeat(self):
        """发送心跳信号"""
        # 通过发布空的twist消息作为心跳
        rospy.loginfo("发送心跳信号")
        
    def run(self):
        """运行控制器"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.listener.stop()

if __name__ == '__main__':
    try:
        controller = ArduSubKeyboardController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("键盘控制器已停止")
    except Exception as e:
        rospy.logerr(f"控制器错误: {e}")
