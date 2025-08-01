#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArduSub控制系统Python包

这个包提供了ArduSub水下机器人的完整控制功能，包括：
- ROS话题接口
- GUI控制界面
- 键盘控制
- UDP通信
- MAVLink协议支持

主要模块：
- control_ardusub: ArduSub底层控制库
- udp_connect: UDP通信核心类
"""

__version__ = "1.0.0"
__author__ = "ArduSub Control Team"

# 导入核心控制模块
try:
    from .control_ardusub import ardusub_control
    from .udp_connect import UDPProxy
except ImportError:
    # 如果作为独立脚本运行
    pass
