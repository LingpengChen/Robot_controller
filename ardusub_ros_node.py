#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import time
import threading
from std_msgs.msg import Float64, Bool, Int32
from geometry_msgs.msg import Twist, Vector3, Quaternion, PoseStamped
from sensor_msgs.msg import FluidPressure, Temperature
from mavros_msgs.msg import AttitudeTarget, PositionTarget, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# 导入你的现有模块
import control_ardusub
from pymavlink import mavutil


class ArdusubRosNode:
    def __init__(self):
        rospy.init_node('ardusub_control_node', anonymous=True)
        
        # 初始化ArduSub控制对象
        self.ardusub_control = control_ardusub.ardusub_control()
        self.master = control_ardusub.master
        
        # ROS Publishers - 发布飞控状态数据
        self.attitude_pub = rospy.Publisher('/ardusub/attitude', Vector3, queue_size=10)
        self.depth_pub = rospy.Publisher('/ardusub/depth', Float64, queue_size=10)
        self.temperature_pub = rospy.Publisher('/ardusub/temperature', Temperature, queue_size=10)
        self.armed_status_pub = rospy.Publisher('/ardusub/armed', Bool, queue_size=10)
        self.mode_pub = rospy.Publisher('/ardusub/mode', Int32, queue_size=10)
        
        # ROS Subscribers - 接收控制指令
        self.cmd_vel_sub = rospy.Subscriber('/ardusub/cmd_vel', Twist, self.cmd_vel_callback)
        self.depth_cmd_sub = rospy.Subscriber('/ardusub/cmd_depth', Float64, self.depth_cmd_callback)
        self.attitude_cmd_sub = rospy.Subscriber('/ardusub/cmd_attitude', Vector3, self.attitude_cmd_callback)
        self.arm_cmd_sub = rospy.Subscriber('/ardusub/cmd_arm', Bool, self.arm_cmd_callback)
        self.mode_cmd_sub = rospy.Subscriber('/ardusub/cmd_mode', Int32, self.mode_cmd_callback)
        self.servo_cmd_sub = rospy.Subscriber('/ardusub/cmd_servo', Vector3, self.servo_cmd_callback)
        self.light_cmd_sub = rospy.Subscriber('/ardusub/cmd_light', Bool, self.light_cmd_callback)
        self.camera_tilt_sub = rospy.Subscriber('/ardusub/cmd_camera_tilt', Float64, self.camera_tilt_callback)
        
        # ROS Services - 提供服务接口
        self.arm_service = rospy.Service('/ardusub/arm', CommandBool, self.arm_service_callback)
        self.set_mode_service = rospy.Service('/ardusub/set_mode', SetMode, self.set_mode_service_callback)
        
        # 内部状态变量
        self.current_attitude = Vector3()
        self.current_depth = 0.0
        self.current_temperature = 0.0
        self.is_armed = False
        self.current_mode = 0
        
        # 启动MAVLink消息接收线程
        self.mavlink_thread = threading.Thread(target=self.mavlink_message_handler)
        self.mavlink_thread.daemon = True
        self.mavlink_thread.start()
        
        rospy.loginfo("ArduSub ROS Node initialized")
        
        # 心跳定时器
        self.heartbeat_timer = rospy.Timer(rospy.Duration(1.0), self.send_heartbeat)
    
    def mavlink_message_handler(self):
        """处理MAVLink消息并发布到ROS话题"""
        while not rospy.is_shutdown():
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue
                    
                msg_type = msg.get_type()
                
                if msg_type == 'ATTITUDE':
                    # 姿态数据
                    self.current_attitude.x = msg.roll * 180 / math.pi  # 转换为度
                    self.current_attitude.y = msg.pitch * 180 / math.pi
                    self.current_attitude.z = msg.yaw * 180 / math.pi
                    self.attitude_pub.publish(self.current_attitude)
                    
                elif msg_type == 'VFR_HUD':
                    # 深度数据
                    self.current_depth = abs(msg.alt)
                    depth_msg = Float64()
                    depth_msg.data = self.current_depth
                    self.depth_pub.publish(depth_msg)
                    
                elif msg_type == 'SCALED_PRESSURE2':
                    # 温度数据
                    self.current_temperature = msg.temperature / 100.0
                    temp_msg = Temperature()
                    temp_msg.temperature = self.current_temperature
                    self.temperature_pub.publish(temp_msg)
                    
                elif msg_type == 'HEARTBEAT':
                    # 武装状态和模式
                    if msg.base_mode == 209:
                        self.is_armed = True
                    elif msg.base_mode == 81:
                        self.is_armed = False
                    
                    armed_msg = Bool()
                    armed_msg.data = self.is_armed
                    self.armed_status_pub.publish(armed_msg)
                    
                    mode_msg = Int32()
                    mode_msg.data = msg.base_mode
                    self.mode_pub.publish(mode_msg)
                    
            except Exception as e:
                rospy.logwarn(f"MAVLink message handling error: {e}")
    
    def cmd_vel_callback(self, msg):
        """处理速度控制指令"""
        # 将ROS Twist消息转换为ArduSub控制指令
        # linear.x: 前进/后退 (前进为正)
        # linear.y: 左/右平移 (右为正)  
        # linear.z: 上浮/下潜 (上浮为正)
        # angular.z: 偏航 (逆时针为正)
        
        x = int(msg.linear.x * 400)  # 缩放到合适范围
        y = int(msg.linear.y * 400)
        z = int(msg.linear.z * 400 + 500)  # 加上中点偏移
        r = int(msg.angular.z * 400)
        
        # 限制范围
        x = max(-2000, min(2000, x))
        y = max(-2000, min(2000, y))
        z = max(0, min(1000, z))
        r = max(-2000, min(2000, r))
        
        # 发送控制指令
        control_ardusub.press_release_buttons(self.master, 0, x, y, z, r)
        
        rospy.logdebug(f"Velocity command: x={x}, y={y}, z={z}, r={r}")
    
    def depth_cmd_callback(self, msg):
        """处理深度控制指令"""
        target_depth = msg.data
        self.ardusub_control.set_target_depth(target_depth)
        rospy.loginfo(f"Depth command: {target_depth}m")
    
    def attitude_cmd_callback(self, msg):
        """处理姿态控制指令"""
        roll = msg.x
        pitch = msg.y
        yaw = msg.z
        self.ardusub_control.set_target_attitude(roll, pitch, yaw)
        rospy.loginfo(f"Attitude command: roll={roll}, pitch={pitch}, yaw={yaw}")
    
    def arm_cmd_callback(self, msg):
        """处理武装/解除武装指令"""
        if msg.data:
            self.master.arducopter_arm()
            rospy.loginfo("Arming vehicle")
        else:
            self.master.arducopter_disarm()
            rospy.loginfo("Disarming vehicle")
    
    def mode_cmd_callback(self, msg):
        """处理模式切换指令"""
        mode = msg.data
        if mode == 0:  # Manual
            control_ardusub.press_release_buttons(self.master, [0, 5], 0, 0, 500, 0)
            rospy.loginfo("Switching to Manual mode")
        elif mode == 1:  # Stabilize
            control_ardusub.press_release_buttons(self.master, [1, 5], 0, 0, 500, 0)
            rospy.loginfo("Switching to Stabilize mode")
        elif mode == 2:  # Depth Hold
            control_ardusub.press_release_buttons(self.master, [3, 5], 0, 0, 500, 0)
            rospy.loginfo("Switching to Depth Hold mode")
    
    def servo_cmd_callback(self, msg):
        """处理舵机控制指令"""
        # x: 舵机1, y: 舵机2, z: 舵机3
        if msg.x != 0:
            self.ardusub_control.set_servo_pwm(1, int(msg.x))
        if msg.y != 0:
            self.ardusub_control.set_servo_pwm(2, int(msg.y))
        if msg.z != 0:
            self.ardusub_control.set_servo_pwm(3, int(msg.z))
    
    def light_cmd_callback(self, msg):
        """处理灯光控制指令"""
        if msg.data:
            # 开灯
            control_ardusub.press_release_buttons(self.master, [14], 0, 0, 500, 0)
            rospy.loginfo("Lights ON")
        else:
            # 关灯
            control_ardusub.press_release_buttons(self.master, [13], 0, 0, 500, 0)
            rospy.loginfo("Lights OFF")
    
    def camera_tilt_callback(self, msg):
        """处理相机云台倾斜角度控制"""
        tilt_angle = msg.data
        self.ardusub_control.look_at(tilt_angle)
        rospy.loginfo(f"Camera tilt: {tilt_angle} degrees")
    
    def arm_service_callback(self, req):
        """武装/解除武装服务回调"""
        try:
            if req.value:
                self.master.arducopter_arm()
                self.master.motors_armed_wait()
                return True, "Vehicle armed"
            else:
                self.master.arducopter_disarm()
                return True, "Vehicle disarmed"
        except Exception as e:
            return False, f"Failed to change arm state: {str(e)}"
    
    def set_mode_service_callback(self, req):
        """模式设置服务回调"""
        try:
            mode_map = {
                'MANUAL': [0, 5],
                'STABILIZE': [1, 5], 
                'DEPTH_HOLD': [3, 5]
            }
            
            if req.custom_mode in mode_map:
                buttons = mode_map[req.custom_mode]
                control_ardusub.press_release_buttons(self.master, buttons, 0, 0, 500, 0)
                return True, f"Mode changed to {req.custom_mode}"
            else:
                return False, f"Unknown mode: {req.custom_mode}"
        except Exception as e:
            return False, f"Failed to change mode: {str(e)}"
    
    def send_heartbeat(self, event):
        """发送心跳"""
        control_ardusub.heartbeat_trigetr()
    
    def run(self):
        """运行ROS节点"""
        rospy.loginfo("ArduSub ROS Node is running...")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ArdusubRosNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ArduSub ROS Node shutting down")
