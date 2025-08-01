# -*- coding: utf-8 -*-
import socket
import struct
import threading
import time

class StatusReceiver:
    def __init__(self, host='127.0.0.1', port=8888):
        """
        接收ArduSub状态反馈数据
        host: 接收地址
        port: 接收端口 (与connect_main.py中设置的server_port对应)
        """
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        
        # 状态数据
        self.pitch = 0.0      # 俯仰角
        self.roll = 0.0       # 横滚角  
        self.yaw = 0.0        # 偏航角
        self.depth = 0.0      # 深度
        self.temperature = 0.0 # 温度
        self.status = 0       # 系统状态 (0=锁定, 1=解锁)
        
        self.running = True
        self.receive_thread = None
        
    def start_receiving(self):
        """启动接收线程"""
        self.receive_thread = threading.Thread(target=self._receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        print(f"状态接收器已启动，监听 {self.host}:{self.port}")
        
    def _receive_loop(self):
        """接收数据循环"""
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                self._parse_status_data(data)
            except Exception as e:
                print(f"接收数据错误: {e}")
                
    def _parse_status_data(self, data):
        """解析状态数据包"""
        try:
            # 数据包格式: 头部(6字节) + pitch(2字节) + roll(2字节) + yaw(2字节) + depth(2字节) + temp(2字节) + status(1字节) + 尾部(4字节)
            if len(data) >= 19:
                # 检查数据包头部
                if data[0:6] == bytes([0xfb, 0xfa, 0xf9, 0xf8, 0x15, 0x0f]):
                    # 解析数据
                    pitch_raw = struct.unpack(">H", data[6:8])[0]
                    roll_raw = struct.unpack(">H", data[8:10])[0]
                    yaw_raw = struct.unpack(">H", data[10:12])[0]
                    depth_raw = struct.unpack(">H", data[12:14])[0]
                    temp_raw = struct.unpack(">H", data[14:16])[0]
                    status = data[16]
                    
                    # 转换为实际值
                    self.pitch = (pitch_raw / 100.0) - 360.0
                    self.roll = (roll_raw / 100.0) - 360.0
                    self.yaw = yaw_raw / 100.0
                    self.depth = depth_raw / 100.0
                    self.temperature = temp_raw / 100.0
                    self.status = status
                    
        except Exception as e:
            print(f"解析数据包错误: {e}")
            
    def get_status(self):
        """获取当前状态"""
        return {
            'pitch': self.pitch,
            'roll': self.roll,
            'yaw': self.yaw,
            'depth': self.depth,
            'temperature': self.temperature,
            'status': 'Armed' if self.status == 1 else 'Disarmed'
        }
        
    def print_status(self):
        """打印当前状态"""
        status = self.get_status()
        print(f"俯仰角: {status['pitch']:.2f}°, "
              f"横滚角: {status['roll']:.2f}°, "
              f"偏航角: {status['yaw']:.2f}°, "
              f"深度: {status['depth']:.2f}m, "
              f"温度: {status['temperature']:.2f}°C, "
              f"状态: {status['status']}")
              
    def stop(self):
        """停止接收"""
        self.running = False
        self.sock.close()

if __name__ == '__main__':
    # 创建状态接收器
    receiver = StatusReceiver()
    receiver.start_receiving()
    
    try:
        # 每秒打印一次状态
        while True:
            receiver.print_status()
            time.sleep(1)
    except KeyboardInterrupt:
        print("停止状态监控...")
        receiver.stop()
