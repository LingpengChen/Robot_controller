#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Header
import threading
import time
import math
import struct
import socket
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu, FluidPressure, Temperature
from std_msgs.msg import Bool, String, Float64

# 如果没有自定义消息，使用标准ROS消息
try:
    from ardusub_control.msg import ArduSubStatus, ArduSubCommand
    USE_CUSTOM_MSG = True
except ImportError:
    print("使用标准ROS消息类型")
    USE_CUSTOM_MSG = False

import sys
import os
# 添加包路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import control_ardusub

class ArduSubROSBridge:
    def __init__(self):
        rospy.init_node('ardusub_ros_bridge', anonymous=True)
        
        # ArduSub连接
        self.ardusub_connect = control_ardusub.ardusub_control()
        
        # 状态数据
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.depth = 0.0
        self.temperature = 0.0
        self.armed = False
        self.flight_mode = "unknown"
        
        # 控制状态
        self.x = 0
        self.y = 0
        self.z = 500
        self.r = 0
        
        self.setup_publishers()
        self.setup_subscribers()
        
        # UDP兼容性 (可选)
        self.setup_udp_compat()
        
        # 启动数据接收线程
        self.start_mavlink_thread()
        
        rospy.loginfo("ArduSub ROS Bridge 已启动")
        
    def setup_publishers(self):
        """设置ROS发布者"""
        if USE_CUSTOM_MSG:
            self.status_pub = rospy.Publisher('/ardusub/status', ArduSubStatus, queue_size=10)
        else:
            # 使用标准消息分别发布
            self.imu_pub = rospy.Publisher('/ardusub/imu', Imu, queue_size=10)
            self.depth_pub = rospy.Publisher('/ardusub/depth', Float64, queue_size=10)
            self.temp_pub = rospy.Publisher('/ardusub/temperature', Float64, queue_size=10)
            self.armed_pub = rospy.Publisher('/ardusub/armed', Bool, queue_size=10)
            self.mode_pub = rospy.Publisher('/ardusub/flight_mode', String, queue_size=10)
            
        rospy.loginfo("状态发布者已设置")
        
    def setup_subscribers(self):
        """设置ROS订阅者"""
        if USE_CUSTOM_MSG:
            self.cmd_sub = rospy.Subscriber('/ardusub/command', ArduSubCommand, self.command_callback)
        else:
            # 使用标准消息分别订阅
            self.twist_sub = rospy.Subscriber('/ardusub/cmd_vel', Twist, self.twist_callback)
            self.mode_sub = rospy.Subscriber('/ardusub/set_mode', String, self.mode_callback)
            self.arm_sub = rospy.Subscriber('/ardusub/arm', Bool, self.arm_callback)
            self.device_sub = rospy.Subscriber('/ardusub/device_cmd', String, self.device_callback)
            
        rospy.loginfo("命令订阅者已设置")
        
    def setup_udp_compat(self):
        """设置UDP兼容模式 (保持与原有UDP客户端的兼容性)"""
        try:
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_address = ('127.0.0.1', 8888)  # 发送状态数据的地址
            self.udp_enabled = True
            rospy.loginfo("UDP兼容模式已启用")
        except Exception as e:
            rospy.logwarn(f"UDP兼容模式启动失败: {e}")
            self.udp_enabled = False
            
    def start_mavlink_thread(self):
        """启动MAVLink数据接收线程"""
        try:
            # 解锁电机
            control_ardusub.master.arducopter_arm()
            control_ardusub.master.motors_armed_wait()
            rospy.loginfo("电机已解锁")
            
            # 启动数据接收线程
            self.mavlink_thread = threading.Thread(target=self.mavlink_receive_loop)
            self.mavlink_thread.daemon = True
            self.mavlink_thread.start()
            
            rospy.loginfo("MAVLink数据接收线程已启动")
        except Exception as e:
            rospy.logerr(f"MAVLink连接失败: {e}")
            
    def mavlink_receive_loop(self):
        """MAVLink数据接收循环"""
        while not rospy.is_shutdown():
            try:
                msg = control_ardusub.master.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue
                    
                if msg.get_type() == 'ATTITUDE':
                    self.pitch = msg.pitch * 180 / math.pi + 360
                    self.roll = msg.roll * 180 / math.pi + 360  
                    self.yaw = (msg.yaw * 180 / math.pi + 360) % 360
                    
                elif msg.get_type() == 'VFR_HUD':
                    self.depth = abs(msg.alt * 100)  # 转换为厘米并取绝对值
                    
                elif msg.get_type() == 'SCALED_PRESSURE2':
                    self.temperature = msg.temperature / 100.0  # 转换为摄氏度
                    
                elif msg.get_type() == 'HEARTBEAT':
                    self.armed = (msg.base_mode == 209)
                    # 简单的模式映射
                    mode_map = {209: "armed", 81: "disarmed"}
                    self.flight_mode = mode_map.get(msg.base_mode, "unknown")
                    
                # 发布状态数据
                self.publish_status()
                    
            except Exception as e:
                rospy.logwarn(f"MAVLink接收错误: {e}")
                time.sleep(0.1)
                
    def publish_status(self):
        """发布状态数据到ROS话题"""
        current_time = rospy.Time.now()
        
        if USE_CUSTOM_MSG:
            # 使用自定义消息
            status_msg = ArduSubStatus()
            status_msg.header.stamp = current_time
            status_msg.header.frame_id = "ardusub"
            
            status_msg.pitch = self.pitch - 360.0
            status_msg.roll = self.roll - 360.0
            status_msg.yaw = self.yaw
            status_msg.depth = self.depth / 100.0  # 转换为米
            status_msg.temperature = self.temperature
            status_msg.armed = self.armed
            status_msg.flight_mode = self.flight_mode
            status_msg.timestamp = current_time.to_sec()
            
            self.status_pub.publish(status_msg)
        else:
            # 使用标准消息分别发布
            # IMU数据
            imu_msg = Imu()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = "ardusub"
            # 这里简化处理，只设置角度信息
            imu_msg.orientation.x = math.radians(self.roll - 360.0)
            imu_msg.orientation.y = math.radians(self.pitch - 360.0)  
            imu_msg.orientation.z = math.radians(self.yaw)
            self.imu_pub.publish(imu_msg)
            
            # 深度数据
            self.depth_pub.publish(Float64(self.depth / 100.0))
            
            # 温度数据
            self.temp_pub.publish(Float64(self.temperature))
            
            # 解锁状态
            self.armed_pub.publish(Bool(self.armed))
            
            # 飞行模式
            self.mode_pub.publish(String(self.flight_mode))
            
        # UDP兼容性发送
        if self.udp_enabled:
            self.send_udp_status()
            
    def send_udp_status(self):
        """发送UDP状态数据 (兼容模式)"""
        try:
            send_data = bytearray()
            
            # 数据包格式与原UDP相同
            data_pitch = struct.pack(">H", int(self.pitch * 100))
            data_roll = struct.pack(">H", int(self.roll * 100))
            data_yaw = struct.pack(">H", int(self.yaw * 100))
            data_depth = struct.pack(">H", int(self.depth))
            data_temp = struct.pack(">H", int(self.temperature * 100))
            
            data_head = [0xfb, 0xfa, 0xf9, 0xf8, 0x15, 0x0f]
            data_end = [0x05, 0x06, 0x07, 0x08]
            
            # 构建数据包
            for num in data_head:
                send_data.append(num)
            
            send_data.extend(data_pitch)
            send_data.extend(data_roll)  
            send_data.extend(data_yaw)
            send_data.extend(data_depth)
            send_data.extend(data_temp)
            
            send_data.append(1 if self.armed else 0)
            
            for num in data_end:
                send_data.append(num)
                
            self.udp_sock.sendto(send_data, self.udp_address)
            
        except Exception as e:
            rospy.logwarn_throttle(5, f"UDP发送失败: {e}")
            
    def command_callback(self, msg):
        """处理自定义控制命令"""
        try:
            if msg.control_type == "movement":
                self.handle_movement_command(msg)
            elif msg.control_type == "mode":
                self.handle_mode_command(msg)
            elif msg.control_type == "device":
                self.handle_device_command(msg)
            elif msg.control_type == "system":
                self.handle_system_command(msg)
                
        except Exception as e:
            rospy.logwarn(f"命令处理错误: {e}")
            
    def twist_callback(self, msg):
        """处理Twist运动命令"""
        try:
            # 将Twist消息转换为ArduSub控制指令
            forward = max(-1.0, min(1.0, msg.linear.x))
            strafe = max(-1.0, min(1.0, msg.linear.y))
            vertical = max(-1.0, min(1.0, msg.linear.z))
            yaw_rate = max(-1.0, min(1.0, msg.angular.z))
            pitch_rate = max(-1.0, min(1.0, msg.angular.y))
            roll_rate = max(-1.0, min(1.0, msg.angular.x))
            
            # 转换为PWM值并发送
            self.send_movement_pwm(forward, strafe, vertical, yaw_rate, pitch_rate, roll_rate)
            
        except Exception as e:
            rospy.logwarn(f"Twist命令处理错误: {e}")
            
    def mode_callback(self, msg):
        """处理模式切换命令"""
        try:
            mode = msg.data.lower()
            if mode == "manual":
                control_ardusub.press_release_buttons(control_ardusub.master, [0, 5], self.x, self.y, self.z, self.r)
            elif mode == "stabilize":
                control_ardusub.press_release_buttons(control_ardusub.master, [1, 5], self.x, self.y, self.z, self.r)
            elif mode == "depth_hold":
                control_ardusub.press_release_buttons(control_ardusub.master, [3, 5], self.x, self.y, self.z, self.r)
                
            rospy.loginfo(f"切换到{mode}模式")
            
        except Exception as e:
            rospy.logwarn(f"模式切换错误: {e}")
            
    def arm_callback(self, msg):
        """处理解锁/锁定命令"""
        try:
            if msg.data:
                control_ardusub.master.arducopter_arm()
                rospy.loginfo("电机解锁")
            else:
                control_ardusub.master.arducopter_disarm()
                rospy.loginfo("电机锁定")
                
        except Exception as e:
            rospy.logwarn(f"解锁/锁定错误: {e}")
            
    def device_callback(self, msg):
        """处理设备控制命令"""
        try:
            # 解析设备命令 格式: "device_name:action:value"
            parts = msg.data.split(':')
            if len(parts) >= 2:
                device = parts[0]
                action = parts[1]
                value = float(parts[2]) if len(parts) > 2 else 0
                
                if device == "gripper":
                    if action == "close":
                        control_ardusub.press_release_buttons(control_ardusub.master, [3], self.x, self.y, self.z, self.r)
                    elif action == "open":
                        control_ardusub.press_release_buttons(control_ardusub.master, [0], self.x, self.y, self.z, self.r)
                        
                elif device == "light_main":
                    if action == "on":
                        control_ardusub.press_release_buttons(control_ardusub.master, [14], self.x, self.y, self.z, self.r)
                    elif action == "off":
                        control_ardusub.press_release_buttons(control_ardusub.master, [13], self.x, self.y, self.z, self.r)
                        
                rospy.loginfo(f"设备控制: {device} {action}")
                
        except Exception as e:
            rospy.logwarn(f"设备控制错误: {e}")
            
    def send_movement_pwm(self, forward, strafe, vertical, yaw_rate, pitch_rate, roll_rate):
        """发送运动控制PWM信号"""
        try:
            # 转换为PWM值 (1100-1900, 中位1500)
            def to_pwm(value):
                return int(1500 + value * 400)  # -1.0~1.0 -> 1100~1900
                
            forward_pwm = to_pwm(forward)
            strafe_pwm = to_pwm(strafe)
            vertical_pwm = to_pwm(vertical)
            yaw_pwm = to_pwm(yaw_rate)
            pitch_pwm = to_pwm(pitch_rate)
            roll_pwm = to_pwm(roll_rate)
            
            # 发送到对应通道
            self.ardusub_connect.set_rc_channel_pwm(5, forward_pwm)   # 前后
            self.ardusub_connect.set_rc_channel_pwm(6, strafe_pwm)    # 左右
            self.ardusub_connect.set_rc_channel_pwm(3, vertical_pwm)  # 上下
            self.ardusub_connect.set_rc_channel_pwm(4, yaw_pwm)       # 偏航
            self.ardusub_connect.set_rc_channel_pwm(1, pitch_pwm)     # 俯仰
            self.ardusub_connect.set_rc_channel_pwm(2, roll_pwm)      # 横滚
            
        except Exception as e:
            rospy.logwarn(f"PWM发送错误: {e}")
            
    def handle_movement_command(self, msg):
        """处理运动控制命令"""
        self.send_movement_pwm(
            msg.forward, msg.strafe, msg.vertical,
            msg.yaw_rate, msg.pitch_rate, msg.roll_rate
        )
        
    def handle_mode_command(self, msg):
        """处理模式命令"""
        mode = msg.flight_mode.lower()
        if mode == "manual":
            control_ardusub.press_release_buttons(control_ardusub.master, [0, 5], self.x, self.y, self.z, self.r)
        elif mode == "stabilize":
            control_ardusub.press_release_buttons(control_ardusub.master, [1, 5], self.x, self.y, self.z, self.r)
        elif mode == "depth_hold":
            control_ardusub.press_release_buttons(control_ardusub.master, [3, 5], self.x, self.y, self.z, self.r)
            
    def handle_device_command(self, msg):
        """处理设备控制命令"""
        device = msg.device_name
        action = msg.device_action
        
        if device == "gripper":
            if action == "close":
                control_ardusub.press_release_buttons(control_ardusub.master, [3], self.x, self.y, self.z, self.r)
            elif action == "open":
                control_ardusub.press_release_buttons(control_ardusub.master, [0], self.x, self.y, self.z, self.r)
                
        elif device == "light_main":
            if action == "on":
                control_ardusub.press_release_buttons(control_ardusub.master, [14], self.x, self.y, self.z, self.r)
            elif action == "off":  
                control_ardusub.press_release_buttons(control_ardusub.master, [13], self.x, self.y, self.z, self.r)
                
    def handle_system_command(self, msg):
        """处理系统命令"""
        if msg.arm_request:
            if self.armed:
                control_ardusub.master.arducopter_disarm()
            else:
                control_ardusub.master.arducopter_arm()
                
        if msg.heartbeat:
            control_ardusub.heartbeat_trigetr()
            
    def run(self):
        """运行节点"""
        rate = rospy.Rate(20)  # 20Hz
        
        while not rospy.is_shutdown():
            # 发送心跳
            control_ardusub.heartbeat_trigetr()
            rate.sleep()

if __name__ == '__main__':
    try:
        bridge = ArduSubROSBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ArduSub ROS Bridge 已停止")
    except Exception as e:
        rospy.logerr(f"节点错误: {e}")
