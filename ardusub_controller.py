#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import threading
from std_msgs.msg import Float64, Bool, Int32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Temperature


class ArdusubController:
    """ArduSub ROS控制客户端示例"""
    
    def __init__(self):
        rospy.init_node('ardusub_controller', anonymous=True)
        
        # 发布控制指令的话题
        self.cmd_vel_pub = rospy.Publisher('/ardusub/cmd_vel', Twist, queue_size=1)
        self.depth_cmd_pub = rospy.Publisher('/ardusub/cmd_depth', Float64, queue_size=1)
        self.attitude_cmd_pub = rospy.Publisher('/ardusub/cmd_attitude', Vector3, queue_size=1)
        self.arm_cmd_pub = rospy.Publisher('/ardusub/cmd_arm', Bool, queue_size=1)
        self.mode_cmd_pub = rospy.Publisher('/ardusub/cmd_mode', Int32, queue_size=1)
        self.light_cmd_pub = rospy.Publisher('/ardusub/cmd_light', Bool, queue_size=1)
        self.camera_tilt_pub = rospy.Publisher('/ardusub/cmd_camera_tilt', Float64, queue_size=1)
        
        # 订阅状态话题
        self.attitude_sub = rospy.Subscriber('/ardusub/attitude', Vector3, self.attitude_callback)
        self.depth_sub = rospy.Subscriber('/ardusub/depth', Float64, self.depth_callback)
        self.temperature_sub = rospy.Subscriber('/ardusub/temperature', Temperature, self.temperature_callback)
        self.armed_sub = rospy.Subscriber('/ardusub/armed', Bool, self.armed_callback)
        
        # 当前状态
        self.current_attitude = Vector3()
        self.current_depth = 0.0
        self.current_temperature = 0.0
        self.is_armed = False
        
        rospy.loginfo("ArduSub Controller initialized")
        
        # 启动用户输入线程
        self.input_thread = threading.Thread(target=self.user_input_handler)
        self.input_thread.daemon = True
        self.input_thread.start()
    
    def attitude_callback(self, msg):
        """姿态回调"""
        self.current_attitude = msg
    
    def depth_callback(self, msg):
        """深度回调"""
        self.current_depth = msg.data
    
    def temperature_callback(self, msg):
        """温度回调"""
        self.current_temperature = msg.temperature
    
    def armed_callback(self, msg):
        """武装状态回调"""
        self.is_armed = msg.data
    
    def print_status(self):
        """打印当前状态"""
        print("\n=== ArduSub状态 ===")
        print(f"武装状态: {'已武装' if self.is_armed else '未武装'}")
        print(f"姿态: Roll={self.current_attitude.x:.1f}°, Pitch={self.current_attitude.y:.1f}°, Yaw={self.current_attitude.z:.1f}°")
        print(f"深度: {self.current_depth:.2f}m")
        print(f"温度: {self.current_temperature:.1f}°C")
        print("==================")
    
    def arm_vehicle(self):
        """武装载具"""
        msg = Bool()
        msg.data = True
        self.arm_cmd_pub.publish(msg)
        rospy.loginfo("武装指令已发送")
    
    def disarm_vehicle(self):
        """解除武装"""
        msg = Bool()
        msg.data = False
        self.arm_cmd_pub.publish(msg)
        rospy.loginfo("解除武装指令已发送")
    
    def move_forward(self, speed=1.0):
        """前进"""
        cmd = Twist()
        cmd.linear.x = speed
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo(f"前进指令已发送，速度: {speed}")
    
    def move_backward(self, speed=1.0):
        """后退"""
        cmd = Twist()
        cmd.linear.x = -speed
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo(f"后退指令已发送，速度: {speed}")
    
    def move_left(self, speed=1.0):
        """左移"""
        cmd = Twist()
        cmd.linear.y = -speed
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo(f"左移指令已发送，速度: {speed}")
    
    def move_right(self, speed=1.0):
        """右移"""
        cmd = Twist()
        cmd.linear.y = speed
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo(f"右移指令已发送，速度: {speed}")
    
    def move_up(self, speed=1.0):
        """上浮"""
        cmd = Twist()
        cmd.linear.z = speed
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo(f"上浮指令已发送，速度: {speed}")
    
    def move_down(self, speed=1.0):
        """下潜"""
        cmd = Twist()
        cmd.linear.z = -speed
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo(f"下潜指令已发送，速度: {speed}")
    
    def turn_left(self, speed=1.0):
        """左转"""
        cmd = Twist()
        cmd.angular.z = speed
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo(f"左转指令已发送，角速度: {speed}")
    
    def turn_right(self, speed=1.0):
        """右转"""
        cmd = Twist()
        cmd.angular.z = -speed
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo(f"右转指令已发送，角速度: {speed}")
    
    def stop(self):
        """停止"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("停止指令已发送")
    
    def set_depth(self, depth):
        """设置目标深度"""
        msg = Float64()
        msg.data = depth
        self.depth_cmd_pub.publish(msg)
        rospy.loginfo(f"深度设置指令已发送: {depth}m")
    
    def set_mode(self, mode):
        """设置飞行模式"""
        msg = Int32()
        msg.data = mode
        self.mode_cmd_pub.publish(msg)
        mode_names = {0: "手动", 1: "自稳", 2: "定深"}
        rospy.loginfo(f"模式切换指令已发送: {mode_names.get(mode, '未知')}")
    
    def toggle_lights(self, on=True):
        """切换灯光"""
        msg = Bool()
        msg.data = on
        self.light_cmd_pub.publish(msg)
        rospy.loginfo(f"灯光{'开启' if on else '关闭'}指令已发送")
    
    def tilt_camera(self, angle):
        """倾斜相机"""
        msg = Float64()
        msg.data = angle
        self.camera_tilt_pub.publish(msg)
        rospy.loginfo(f"相机倾斜指令已发送: {angle}°")
    
    def user_input_handler(self):
        """处理用户输入"""
        print("\n=== ArduSub控制器 ===")
        print("命令列表:")
        print("s - 显示状态")
        print("a - 武装/解除武装")
        print("w/s - 前进/后退")
        print("a/d - 左移/右移") 
        print("q/e - 上浮/下潜")
        print("j/l - 左转/右转")
        print("space - 停止")
        print("m - 切换模式")
        print("o - 切换灯光")
        print("t - 倾斜相机")
        print("depth <值> - 设置深度")
        print("quit - 退出")
        print("====================\n")
        
        while not rospy.is_shutdown():
            try:
                cmd = input("输入命令: ").strip().lower()
                
                if cmd == 'quit' or cmd == 'q':
                    break
                elif cmd == 's':
                    self.print_status()
                elif cmd == 'a':
                    if self.is_armed:
                        self.disarm_vehicle()
                    else:
                        self.arm_vehicle()
                elif cmd == 'w':
                    self.move_forward()
                elif cmd == 's':
                    self.move_backward()
                elif cmd == 'a':
                    self.move_left()
                elif cmd == 'd':
                    self.move_right()
                elif cmd == 'q':
                    self.move_up()
                elif cmd == 'e':
                    self.move_down()
                elif cmd == 'j':
                    self.turn_left()
                elif cmd == 'l':
                    self.turn_right()
                elif cmd == ' ' or cmd == 'space':
                    self.stop()
                elif cmd == 'm':
                    mode = int(input("输入模式 (0=手动, 1=自稳, 2=定深): "))
                    self.set_mode(mode)
                elif cmd == 'o':
                    self.toggle_lights(not getattr(self, 'lights_on', False))
                    self.lights_on = not getattr(self, 'lights_on', False)
                elif cmd == 't':
                    angle = float(input("输入相机倾斜角度: "))
                    self.tilt_camera(angle)
                elif cmd.startswith('depth'):
                    try:
                        depth = float(cmd.split()[1])
                        self.set_depth(depth)
                    except:
                        print("格式错误，请使用: depth <值>")
                else:
                    print("未知命令")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"输入错误: {e}")
        
        rospy.signal_shutdown("用户退出")
    
    def run(self):
        """运行控制器"""
        rospy.loginfo("ArduSub Controller running...")
        rospy.spin()


if __name__ == '__main__':
    try:
        controller = ArdusubController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller shutting down")
