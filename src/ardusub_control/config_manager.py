#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArduSub控制系统配置管理器

用于加载和管理系统配置参数
"""

import os
import yaml
import rospy
from typing import Dict, Any, Optional

class ArduSubConfig:
    """ArduSub配置管理类"""
    
    def __init__(self, config_file: Optional[str] = None):
        """
        初始化配置管理器
        
        Args:
            config_file: 配置文件路径，如果为None则使用默认路径
        """
        if config_file is None:
            # 获取默认配置文件路径
            package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            config_file = os.path.join(package_path, 'config', 'ardusub_config.yaml')
        
        self.config_file = config_file
        self.config_data = {}
        self._load_config()
    
    def _load_config(self):
        """加载配置文件"""
        try:
            with open(self.config_file, 'r', encoding='utf-8') as file:
                self.config_data = yaml.safe_load(file)
            print(f"配置文件加载成功: {self.config_file}")
        except FileNotFoundError:
            print(f"配置文件未找到: {self.config_file}，使用默认配置")
            self._load_default_config()
        except yaml.YAMLError as e:
            print(f"配置文件解析错误: {e}，使用默认配置")
            self._load_default_config()
    
    def _load_default_config(self):
        """加载默认配置"""
        self.config_data = {
            'network': {
                'ardusub': {'host': '127.0.0.1', 'port': 14550, 'protocol': 'udp'},
                'command_port': 9999,
                'status_port': 8888
            },
            'control': {
                'default_mode': 'manual',
                'heartbeat_interval': 1.0
            },
            'gui': {
                'window_title': 'ArduSub GUI控制器',
                'window_size': [800, 600],
                'update_rate': 10
            }
        }
    
    def get(self, path: str, default: Any = None) -> Any:
        """
        获取配置值
        
        Args:
            path: 配置路径，使用点号分隔，如 'network.ardusub.host'
            default: 默认值
            
        Returns:
            配置值
        """
        keys = path.split('.')
        value = self.config_data
        
        try:
            for key in keys:
                value = value[key]
            return value
        except (KeyError, TypeError):
            return default
    
    def set(self, path: str, value: Any):
        """
        设置配置值
        
        Args:
            path: 配置路径
            value: 配置值
        """
        keys = path.split('.')
        data = self.config_data
        
        for key in keys[:-1]:
            if key not in data:
                data[key] = {}
            data = data[key]
        
        data[keys[-1]] = value
    
    def save(self):
        """保存配置到文件"""
        try:
            with open(self.config_file, 'w', encoding='utf-8') as file:
                yaml.dump(self.config_data, file, default_flow_style=False, 
                         allow_unicode=True, indent=2)
            print(f"配置已保存到: {self.config_file}")
        except Exception as e:
            print(f"配置保存失败: {e}")
    
    def setup_ros_params(self):
        """将配置参数设置到ROS参数服务器"""
        try:
            # 网络配置
            rospy.set_param('/ardusub/network/host', self.get('network.ardusub.host'))
            rospy.set_param('/ardusub/network/port', self.get('network.ardusub.port'))
            rospy.set_param('/ardusub/network/command_port', self.get('network.command_port'))
            rospy.set_param('/ardusub/network/status_port', self.get('network.status_port'))
            
            # 控制配置
            rospy.set_param('/ardusub/control/default_mode', self.get('control.default_mode'))
            rospy.set_param('/ardusub/control/heartbeat_interval', self.get('control.heartbeat_interval'))
            
            # ROS话题配置
            ros_topics = self.get('ros.topics', {})
            for topic_name, topic_path in ros_topics.items():
                rospy.set_param(f'/ardusub/topics/{topic_name}', topic_path)
            
            print("ROS参数设置完成")
        except Exception as e:
            print(f"ROS参数设置失败: {e}")

# 全局配置实例
_config_instance = None

def get_config() -> ArduSubConfig:
    """获取全局配置实例"""
    global _config_instance
    if _config_instance is None:
        _config_instance = ArduSubConfig()
    return _config_instance

if __name__ == "__main__":
    # 测试配置管理器
    config = ArduSubConfig()
    print(f"ArduSub主机: {config.get('network.ardusub.host')}")
    print(f"命令端口: {config.get('network.command_port')}")
    print(f"默认模式: {config.get('control.default_mode')}")
