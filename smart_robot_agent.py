# -*- coding: utf-8 -*-
"""Smart Robot Agent with USB Serial Communication"""

import os
import json
import sqlite3
from datetime import datetime
import threading
import time
import asyncio
import logging
from typing import Optional, Dict, Any, List
import re
import queue

# 导入USB串口管理器
from usb_serial_manager import SerialManager

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ROS2 可用性标志
ROS2_AVAILABLE = False
rclpy = None
Node = None
geometry_msgs = None
jqr_ros_msgs = None

# jqr_ros_msgs 相关的导入，设置为None以避免未定义错误
BatteryLevel = None
MedicineBoxState = None
MedicineBoxSwitch = None
MoveMode = None
RobotRise = None
RobotRiseState = None
RobotTilt = None
RobotTiltState = None
ScreenTilt = None
ScreenTiltState = None
RgbLightStrip = None
RgbLightStripState = None
LaserPointer = None
LaserPointerState = None

# 电池监控相关全局变量
battery_node = None
battery_thread = None
battery_thread_running = False
battery_level = 100.0

robot_pose_node = None
robot_pose_thread = None
robot_pose_thread_running = False
# 尝试导入rclpy，如果不存在则忽略
try:
    import rclpy
    from rclpy.node import Node
    import geometry_msgs.msg as geometry_msgs
    # 尝试导入tf2_ros
    try:
        import tf2_ros
    except ImportError:
        tf2_ros = None
    # 尝试导入jqr_ros_msgs
    try:
        from jqr_ros_msgs.msg import BatteryLevel
        from jqr_ros_msgs.srv import (
            MedicineBoxState, MedicineBoxSwitch,
            MoveMode,
            RobotRise, RobotRiseState,
            RobotTilt, RobotTiltState,
            ScreenTilt, ScreenTiltState
        )
        jqr_ros_msgs = True
        logger.info("jqr_ros_msgs 导入成功")
    except ImportError as e:
        logger.warning(f"jqr_ros_msgs 不可用: {e}")
        jqr_ros_msgs = False
    ROS2_AVAILABLE = True
    logger.info("ROS2 rclpy 导入成功")
except ImportError as e:
    logger.warning(f"ROS2 rclpy 不可用: {e}")
    geometry_msgs = None
    jqr_ros_msgs = False

# 尝试导入cv2，如果不存在则忽略
try:
    import cv2
except ImportError:
    cv2 = None
    logger.warning("cv2模块未安装，视频处理功能将不可用")

# 导入subprocess用于系统调用
import subprocess

# ======================
# 配置
# ======================
ASM_JSON_PATH = "asm_data.json"
VIDEO_BASE_DIR = "videos"
DB_PATH = "history.db"

os.makedirs(VIDEO_BASE_DIR, exist_ok=True)
# os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)

# USB串口配置
USB_SERIAL_PORT = "/dev/ttyACM0"
USB_SERIAL_BAUDRATE = 115200

# ======================
# JSON修复函数
# ======================

def parse_ros2_response(response: str) -> Dict[str, Any]:
    """
    解析ROS2服务响应（支持多种格式）
    
    Args:
        response (str): ROS2服务返回的响应字符串
        
    Returns:
        Dict[str, Any]: 解析后的响应数据
    """
    if not response or not response.strip():
        return {}
    
    try:
        # 首先尝试JSON解析（以防某些服务返回JSON）
        try:
            return json.loads(response)
        except json.JSONDecodeError:
            pass
        
        # 处理ROS2响应格式: jqr_ros_msgs.srv.MoveMode_Response(move_mode=0 linear_vel=0.0 result_number=std_msgs.msg.UInt8(data=1) result_msg=std_msgs.msg.String(data='获取运动模式成功'))
        if 'response:' in response:
            # 提取response行
            response_line = ''
            for line in response.strip().split('\n'):
                if line.startswith('response:'):
                    response_line = line.replace('response:', '').strip()
                    break
            
            if not response_line:
                # 如果没有找到response:行，尝试直接解析
                response_line = response.strip()
        else:
            response_line = response.strip()
        
        result = {}
        
        # 使用正则表达式提取字段值
        # 匹配格式: field_name=value 或者 field_name=type(value)
        pattern = r'(\w+)=([^,\s]+(?:\([^)]*\))?)'
        matches = re.findall(pattern, response_line)
        
        for field_name, value_str in matches:
            # 解析不同类型的值
            if value_str.isdigit():
                # 整数
                result[field_name] = int(value_str)
            elif '.' in value_str and value_str.replace('.', '').isdigit():
                # 浮点数
                result[field_name] = float(value_str)
            elif 'UInt8' in value_str and 'data=' in value_str:
                # 处理std_msgs.msg.UInt8(data=0)格式
                data_match = re.search(r'data=(\d+)', value_str)
                if data_match:
                    result[field_name] = int(data_match.group(1))
                else:
                    result[field_name] = value_str
            elif 'String' in value_str and 'data=' in value_str:
                # 处理std_msgs.msg.String(data="xxx")格式
                data_match = re.search(r'data=\["\']([^"\']+)["\']', value_str)
                if data_match:
                    result[field_name] = data_match.group(1)
                else:
                    result[field_name] = value_str
            elif value_str.startswith('"') and value_str.endswith('"'):
                # 字符串
                result[field_name] = value_str[1:-1]
            elif value_str.startswith("'") and value_str.endswith("'"):
                # 字符串
                result[field_name] = value_str[1:-1]
            elif value_str.lower() in ['true', 'false']:
                # 布尔值
                result[field_name] = value_str.lower() == 'true'
            else:
                # 其他情况保持原样
                result[field_name] = value_str
        
        return result
        
    except Exception as e:
        logger.error(f"解析ROS2响应失败: {e}, 原始响应: {response}")
        return {}

def fix_asm_json_format():
    """修复ASM JSON文件格式 - 只修复格式问题，不修改数据内容"""
    if not os.path.exists(ASM_JSON_PATH):
        logger.warning(f"ASM JSON文件不存在: {ASM_JSON_PATH}")
        return False
    
    try:
        # 读取原文件
        with open(ASM_JSON_PATH, 'r', encoding='utf-8') as f:
            original_content = f.read().strip()
        
        # 检查是否已经是有效的JSON
        try:
            json.loads(original_content)
            logger.info("ASM JSON文件格式正确，无需修复")
            return True
        except json.JSONDecodeError:
            logger.warning("ASM JSON文件格式错误，开始修复...")
        
        # 备份原文件
        backup_path = f"{ASM_JSON_PATH}.backup.{int(time.time())}"
        os.makedirs(os.path.dirname(backup_path), exist_ok=True)
        with open(backup_path, 'w', encoding='utf-8') as f:
            f.write(original_content)
        logger.info(f"已备份原文件到: {backup_path}")
        
        # 尝试修复常见的JSON格式问题
        fixed_content = original_content
        
        # 1. 移除BOM头
        if fixed_content.startswith('\ufeff'):
            fixed_content = fixed_content[1:]
            logger.info("移除BOM头")
        
        # 2. 确保字符串引号统一为双引号
        # 这里只做最简单的检查，确保外层是大括号
        fixed_content = fixed_content.strip()
        if not fixed_content.startswith('{'):
            # 如果不是以{开头，尝试修复
            fixed_content = '{' + fixed_content
            logger.info("添加缺失的起始大括号")
        if not fixed_content.endswith('}'):
            # 如果不是以}结尾，尝试修复
            fixed_content = fixed_content + '}'
            logger.info("添加缺失的结束大括号")
        
        # 验证修复后的JSON是否有效
        try:
            json_data = json.loads(fixed_content)
            # 写入修复后的内容
            with open(ASM_JSON_PATH, 'w', encoding='utf-8') as f:
                json.dump(json_data, f, ensure_ascii=False, indent=2)
            logger.info("ASM JSON文件修复成功")
            return True
        except json.JSONDecodeError as e:
            logger.error(f"修复后的JSON仍然无效: {e}")
            logger.info("保持原文件不变")
            # 恢复原文件
            with open(ASM_JSON_PATH, 'w', encoding='utf-8') as f:
                f.write(original_content)
            return False
            
    except Exception as e:
        logger.error(f"修复ASM JSON文件时出错: {e}")
        return False

# ======================
# 数据库操作
# ======================

def init_database():
    """初始化数据库"""
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS history (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                command TEXT NOT NULL,
                response TEXT NOT NULL
            )
        ''')
        
        conn.commit()
        conn.close()
        logger.info("数据库初始化成功")
    except Exception as e:
        logger.error(f"数据库初始化失败: {e}")

def save_to_history(command: str, response: str):
    """保存命令和响应到数据库"""
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        cursor.execute(
            'INSERT INTO history (timestamp, command, response) VALUES (?, ?, ?)',
            (timestamp, command, response)
        )
        
        conn.commit()
        conn.close()
    except Exception as e:
        logger.error(f"保存到数据库失败: {e}")

# ======================
# ROS2接口
# ======================

class ROS2Interface:
    """ROS2接口类，用于与ROS2系统进行交互"""
    
    def __init__(self):
        """初始化ROS2接口"""
        self.battery_level = 100.0  # 初始电池电量
        self.last_position = None  # 记录最后一个位置
        self.position_subscribed = False  # 是否已订阅位置信息
        self.battery_subscribed = False  # 是否已订阅电池电量信息
        self.battery_subscription = None  # 电池电量订阅对象
        self.position_subscription = None  # 位置订阅对象
        self.node = None  # ROS2节点
        self.initialized = False  # ROS2是否已初始化
        self.ros2_thread = None  # ROS2处理线程
        self.ros2_thread_running = False  # ROS2线程是否运行
        
        # 如果ROS2可用，初始化rclpy
        if ROS2_AVAILABLE:
            self._initialize_ros2()
    def set_laser_pointer(self, *args, **kwargs) -> Dict[str, Any]:
        """控制激光笔开关/查询状态 (jqr_ros_msgs版本)
        
        Args:
            laser_pointer (bool): True=开启, False=关闭
            
        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            # 处理参数 - 兼容错误的调用方式 set_laser_pointer(bool=True)
            laser_pointer_value = None
            if 'bool' in kwargs:
                # 修复错误的参数名：bool -> laser_pointer
                laser_pointer_value = kwargs.pop('bool')
                kwargs['laser_pointer'] = laser_pointer_value
            
            if 'laser_pointer' in kwargs:
                laser_pointer_value = kwargs['laser_pointer']
            elif len(args) > 0:
                laser_pointer_value = args[0]
            else:
                return self.get_laser_pointer_state()
            
            request_data = f'{{"laser_pointer": {laser_pointer_value}}}'
            response = self._call_ros2_service(
                "/set_laser_pointer",
                "jqr_ros_msgs/srv/LaserPointer",
                request_data
            )
            if response is None:
                result = {
                    "success": False,
                    "description": "服务 /set_laser_pointer 不存在或调用失败"
                }
                logger.error(f"[ROS2] 设置激光笔失败: {result}")
                return result
            else:
                if not response or not response.strip():
                    return {
                        "success": False,
                        "description": "激光笔服务返回空响应"
                    }
                try:
                    response_data = parse_ros2_response(response)
                    result_number = response_data.get("result_number", 0)
                    result_msg = response_data.get("result_msg", "")
                    success = (result_number in [1, 2, 3])
                    result = {
                        "success": success,
                        "description": result_msg,
                        "result_number": result_number
                    }
                    if success:
                        logger.info(f"[ROS2] 激光笔控制成功: {result}")
                    else:
                        logger.error(f"[ROS2] 激光笔控制失败: {result}")
                    return result
                except Exception as e:
                    logger.error(f"[ROS2] 激光笔响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            logger.error(f"[ROS2] 设置激光笔失败: {e}")
            return {
                "success": False,
                "description": f"设置激光笔失败: {str(e)}"
            }
    
    def get_laser_pointer_state(self) -> Dict[str, Any]:
        """获取激光笔状态
        
        Returns:
            Dict[str, Any]: 激光笔状态信息
        """
        try:
            response = self._call_ros2_service(
                "/get_laser_pointer_state",
                "jqr_ros_msgs/srv/LaserPointerState",
                "{}"
            )
            if response is None:
                result = {
                    "success": False,
                    "description": "服务 /get_laser_pointer_state 不存在或调用失败"
                }
                logger.error(f"[ROS2] 获取激光笔状态失败: {result}")
                return result
            else:
                if not response or not response.strip():
                    return {
                        "success": False,
                        "description": "激光笔状态服务返回空响应"
                    }
                try:
                    response_data = parse_ros2_response(response)
                    laser_pointer_state = response_data.get("laser_pointer_state", False)
                    result_number = response_data.get("result_number", 0)
                    result_msg = response_data.get("result_msg", "")
                    success = (result_number == 1)
                    result = {
                        "success": success,
                        "laser_pointer_state": laser_pointer_state,
                        "description": result_msg if success else f"获取失败: {result_msg}",
                        "result_number": result_number
                    }
                    if success:
                        logger.info(f"[ROS2] 获取激光笔状态成功: {result}")
                    else:
                        logger.error(f"[ROS2] 获取激光笔状态失败: {result}")
                    return result
                except Exception as e:
                    logger.error(f"[ROS2] 激光笔状态响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            logger.error(f"[ROS2] 获取激光笔状态失败: {e}")
            return {
                "success": False,
                "description": f"获取激光笔状态失败: {str(e)}"
            }

    def start_battery_monitoring(self) -> bool:
        """开始电池电量监控（订阅模式）"""
        global battery_level
        
        if hasattr(self, 'battery_subscribed') and self.battery_subscribed:
            logger.warning("电池电量监控已在运行")
            return True
            
        try:
            # 检查ROS2和节点是否可用
            if not ROS2_AVAILABLE or not self.initialized or not self.node:
                logger.warning("ROS2不可用或未初始化，无法启动电池电量监控")
                return False
            
            # 使用主节点创建电池电量订阅
            if jqr_ros_msgs:
                self.battery_subscription = self.node.create_subscription(
                    BatteryLevel,
                    '/battery_level',  # 电池电量话题
                    battery_callback,
                    10  # 队列大小
                )
                self.battery_subscribed = True
                logger.info("电池电量订阅者已创建")
                logger.info("电池电量订阅监控已启动")
                return True
            else:
                logger.warning("jqr_ros_msgs不可用，无法创建电池电量订阅者")
                return False
            
        except Exception as e:
            logger.error(f"启动电池电量监控失败: {e}")
            return False
        
    def stop_battery_monitoring(self):
        """停止电池电量监控"""
        try:
            if hasattr(self, 'battery_subscription') and self.battery_subscription:
                # 销毁订阅
                self.battery_subscription.destroy()
                self.battery_subscription = None
                
            self.battery_subscribed = False
            logger.info("电池电量监控已停止")
            return True
            
        except Exception as e:
            logger.error(f"停止电池电量监控失败: {e}")
            return False

    def _initialize_ros2(self):
        """初始化ROS2"""
        global rclpy
        try:
            if not rclpy:
                logger.warning("[ROS2] rclpy模块不可用，跳过初始化")
                self.initialized = False
                return False
            
            # 检查是否已经初始化
            try:
                # 尝试获取rclpy状态来判断是否已初始化
                if hasattr(rclpy, 'get_instance'):
                    instance = rclpy.get_instance()
                    if instance is not None:
                        logger.info("[ROS2] rclpy已经初始化")
                        # 如果rclpy已初始化但没有节点，创建节点
                        if self.node is None:
                            self.node = rclpy.create_node('smart_robot_agent_ros2')
                            logger.info("[ROS2] 节点创建成功: smart_robot_agent_ros2")
                        self.initialized = True
                        # 启动ROS2处理线程
                        self._start_ros2_spin_thread()
                        return True
                else:
                    # 备用检查方法
                    logger.info("[ROS2] 检查rclpy初始化状态")
            except Exception as check_error:
                logger.debug(f"[ROS2] 检查初始化状态时出错: {check_error}")
                # 未初始化，进行初始化
                pass
            
            # 初始化rclpy
            rclpy.init()
            logger.info("[ROS2] rclpy初始化成功")
            
            # 创建节点
            self.node = rclpy.create_node('smart_robot_agent_ros2')
            logger.info("[ROS2] 节点创建成功: smart_robot_agent_ros2")
            
            self.initialized = True
            # 启动ROS2处理线程
            self._start_ros2_spin_thread()
            return True
            
        except Exception as e:
            logger.error(f"[ROS2] 初始化失败: {e}")
            self.initialized = False
            return False
    
    def _start_ros2_spin_thread(self):
        """启动ROS2独立处理线程"""
        if self.ros2_thread is not None and self.ros2_thread.is_alive():
            logger.warning("[ROS2] ROS2处理线程已在运行")
            return
            
        self.ros2_thread_running = True
        self.ros2_thread = threading.Thread(target=self._ros2_spin_worker, daemon=True)
        self.ros2_thread.start()
        logger.info("[ROS2] 独立处理线程已启动")
    
    def _ros2_spin_worker(self):
        """ROS2独立处理线程工作函数"""
        global rclpy
        try:
            logger.info("[ROS2] 处理线程开始运行")
            while self.ros2_thread_running and rclpy and rclpy.ok() and self.node:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                # 短暂休眠避免CPU占用过高
                time.sleep(0.01)
        except Exception as e:
            logger.error(f"[ROS2] 处理线程出错: {e}")
        finally:
            logger.info("[ROS2] 处理线程已退出")
    
    def stop_ros2_spin_thread(self):
        """停止ROS2处理线程"""
        try:
            self.ros2_thread_running = False
            if self.ros2_thread and self.ros2_thread.is_alive():
                self.ros2_thread.join(timeout=2.0)
            logger.info("[ROS2] 处理线程已停止")
        except Exception as e:
            logger.error(f"[ROS2] 停止处理线程失败: {e}")

    def cleanup_ros2(self):
        """清理ROS2资源"""
        global rclpy
        try:
            # 停止处理线程
            self.stop_ros2_spin_thread()
            
            if self.initialized and rclpy:
                if self.node:
                    self.node.destroy_node()
                    self.node = None
                rclpy.shutdown()
                self.initialized = False
                logger.info("[ROS2] 资源已清理")
        except Exception as e:
            logger.error(f"清理ROS2资源时出错: {e}")
        
    def _check_ros2_service_exists(self, service_name: str) -> bool:
        """检查ROS2服务是否存在
        
        Args:
            service_name (str): 服务名称
            
        Returns:
            bool: 服务是否存在
        """
        try:
            # 使用ros2 service list命令检查服务是否存在
            cmd = f"ros2 service list"
            result = os.popen(cmd).read().strip()
            
            # 检查服务名称是否在服务列表中
            services = result.split('\n')
            for service in services:
                if service.strip() == service_name:
                    logger.info(f"[ROS2] 服务 {service_name} 存在")
                    return True
            
            logger.warning(f"[ROS2] 服务 {service_name} 不存在")
            return False
        except Exception as e:
            logger.error(f"[ROS2] 检查服务存在性失败: {e}")
            return False
    
    def _check_ros2_action_exists(self, action_name: str) -> bool:
        """检查ROS2动作是否存在
        
        Args:
            action_name (str): 动作名称
            
        Returns:
            bool: 动作是否存在
        """
        try:
            # 使用ros2 action list命令检查动作是否存在
            cmd = f"ros2 action list"
            result = os.popen(cmd).read().strip()
            
            # 检查动作名称是否在动作列表中
            actions = result.split('\n')
            for action in actions:
                if action.strip() == action_name:
                    logger.info(f"[ROS2] 动作 {action_name} 存在")
                    return True
            
            logger.warning(f"[ROS2] 动作 {action_name} 不存在")
            return False
        except Exception as e:
            logger.error(f"[ROS2] 检查动作存在性失败: {e}")
            return False
    
    def _call_ros2_service(self, service_name: str, service_type: str, request_data: str) -> Optional[str]:
        """调用ROS2服务
        
        Args:
            service_name (str): 服务名称
            service_type (str): 服务类型
            request_data (str): 请求数据
            
        Returns:
            Optional[str]: 服务响应，如果调用失败则返回None
        """
        # 首先检查服务是否存在
        if not self._check_ros2_service_exists(service_name):
            logger.error(f"[ROS2] 服务 {service_name} 不存在，无法调用")
            return None
            
        try:
            # 构造ROS2服务调用命令
            # 注意：request_data可能是字典格式，需要转换为YAML字符串
            if isinstance(request_data, dict):
                import yaml
                request_str = yaml.dump(request_data, default_flow_style=False)
            else:
                request_str = str(request_data)
            
            cmd = f"ros2 service call {service_name} {service_type} '{request_str}'"
            logger.info(f"[ROS2] 执行命令: {cmd}")
            
            # 使用subprocess而不是os.popen来获得更好的控制
            # 设置ROS环境变量
            import os
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = os.environ.get('ROS_DOMAIN_ID', '0')
            
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10, env=env)
            
            if result.returncode != 0:
                logger.error(f"[ROS2] 服务调用命令执行失败，返回码: {result.returncode}")
                logger.error(f"[ROS2] stderr: {result.stderr}")
                return None
            
            response = result.stdout.strip()
            logger.info(f"[ROS2] 服务调用结果: {response}")
            
            # 检查结果是否为空
            if not response:
                logger.error(f"[ROS2] 服务 {service_name} 返回空响应")
                return None
                
            return response
        except subprocess.TimeoutExpired:
            logger.error(f"[ROS2] 服务调用超时: {service_name}")
            return None
        except Exception as e:
            logger.error(f"[ROS2] 服务调用失败: {e}")
            return None    
    
    def subscribe_robot_position(self) -> bool:
        """订阅机器人位置信息
        
        Returns:
            bool: 订阅是否成功
        """
        try:
            if not ROS2_AVAILABLE or not self.initialized or not self.node:
                logger.warning("[ROS2] ROS2不可用或未初始化，无法订阅位置信息")
                return False
            
            if not geometry_msgs:
                logger.error("[ROS2] geometry_msgs不可用，无法订阅位置话题")
                return False
                
            # 使用主节点创建位置订阅，订阅的回调由主spin循环处理
            self.position_subscription = self.node.create_subscription(
                geometry_msgs.PoseStamped,
                '/tracked_pose',  # 假设SLAM发布的话题名为 /tracked_pose
                self._position_callback,
                10
            )
            
            self.position_subscribed = True
            logger.info("[ROS2] 已使用主节点订阅机器人位置话题: /tracked_pose")
            return True
            
        except Exception as e:
            logger.error(f"[ROS2] 订阅位置信息失败: {e}")
            return False
    
    def _position_callback(self, msg):
        """位置回调函数"""
        try:
            position = {
                'position': {
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.orientation.x,
                    'y': msg.pose.orientation.y,
                    'z': msg.pose.orientation.z,
                    'w': msg.pose.orientation.w
                },
                'header': {
                    'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    'frame_id': msg.header.frame_id
                }
            }
            self.last_position = position
            logger.info(f"[ROS2] 收到位置更新: ({position['position']['x']:.2f}, {position['position']['y']:.2f})")
            
            # 构造位置更新消息
            # position_message = {
            #     "type": "position_update",
            #     "position": position['position'],
            #     "orientation": position['orientation'],
            #     "timestamp": position['header']['stamp']
            # }
            
            # 通过同步方式发送位置信息，避免异步问题
            # global smart_robot_agent_instance
            # if smart_robot_agent_instance and hasattr(smart_robot_agent_instance, 'usb_manager'):
            #     # 创建新的事件循环来处理异步任务
            #     try:
            #         loop = asyncio.new_event_loop()
            #         asyncio.set_event_loop(loop)
            #         loop.run_until_complete(smart_robot_agent_instance.usb_manager.send_message(position_message))
            #         loop.close()
            #         logger.info(f'位置信息已通过USB发送: ({position["position"]["x"]:.2f}, {position["position"]["y"]:.2f})')
            #     except Exception as async_error:
            #         logger.error(f'异步发送位置信息失败: {async_error}')
            #         # 备用方案：添加到消息队列
            #         smart_robot_agent_instance.message_queue.put(position_message)
            # else:
            #     logger.warning('USB管理器不可用，无法发送位置信息')
            
        except Exception as e:
            logger.error(f"[ROS2] 位置回调处理失败: {e}")
    
    def record_current_position(self) -> bool:
        """记录当前位置
        
        Returns:
            bool: 记录是否成功
        """
        if self.last_position:
            logger.info(f"[ROS2] 已记录当前位置: {self.last_position['position']}")
            return True
        else:
            logger.warning("[ROS2] 没有可用的位置信息")
            return False
    
    def get_last_position(self) -> Optional[Dict[str, Any]]:
        """获取最后记录的位置
        
        Returns:
            Optional[Dict[str, Any]]: 位置信息，如果没有则返回None
        """
        return self.last_position
    
    def navigate_to_position(self, position: Dict[str, Any]) -> Dict[str, Any]:
        """导航到指定位置
        
        注意：/navigate_to_pose 是一个action server，不是service
        
        Args:
            position (Dict[str, Any]): 目标位置信息
            
        Returns:
            Dict[str, Any]: 导航结果
        """
        try:
            if not position or 'position' not in position:
                return {
                    "success": False,
                    "error_msg": "无效的位置信息"
                }
            
            target_x = position['position']['x']
            target_y = position['position']['y']
            
            # 检查导航action是否可用
            if not self._check_ros2_action_exists("/navigate_to_pose"):
                logger.error(f"[ROS2] 导航action /navigate_to_pose 不可用")
                return {
                    "success": False,
                    "error_msg": "导航action /navigate_to_pose 不可用"
                }
            
            # 调用导航action
            # 构造NavigateToPose goal
            goal_data = {
                "pose": {
                    "header": {
                        "stamp": {"sec": 0, "nanosec": 0},
                        "frame_id": "map"
                    },
                    "pose": {
                        "position": {
                            "x": target_x,
                            "y": target_y,
                            "z": 0.0
                        },
                        "orientation": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": 0.0,
                            "w": 1.0
                        }
                    }
                }
            }
            
            response = self._call_ros2_action(
                "/navigate_to_pose",
                "nav2_msgs/action/NavigateToPose",
                str(goal_data)
            )
            
            if response:
                return {
                    "success": True,
                    "position": position['position'],
                    "description": f"导航已开始，目标位置 ({target_x}, {target_y})"
                }
            else:
                return {
                    "success": False,
                    "position": position['position'],
                    "error_msg": "导航action调用失败"
                }
                
        except Exception as e:
            logger.error(f"[ROS2] 导航到位置失败: {e}")
            return {
                "success": False,
                "error_msg": f"导航失败: {str(e)}"
            }
        
    # ======================
    # 机器人运动控制相关接口
    # ======================
    
    def get_move_mode(self) -> Dict[str, Any]:
        """获取运动模式
        
        Returns:
            Dict[str, Any]: 运动模式信息
        """
        try:
            # 调用ROS2服务获取运动模式
            response = self._call_ros2_service(
                "/get_move_mode",
                "jqr_ros_msgs/srv/MoveMode",
                '{}'
            )
            
            if response is None:
                # 服务调用失败，可能是服务不存在
                result = {
                    "success": False,
                    "move_mode": -1,
                    "linear_vel": 0.0,
                    "description": "服务 /get_move_mode 不存在或调用失败"
                }
                logger.error(f"[ROS2] 获取运动模式失败: {result}")
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
                    logger.error(f"[ROS2] 运动模式服务返回空响应")
                    return {
                        "success": False,
                        "move_mode": -1,
                        "linear_vel": 0.0,
                        "description": "运动模式服务返回空响应"
                    }
                
                # 解析响应数据
                try:
                    # 使用新的解析函数解析YAML响应
                    response_data = parse_ros2_response(response)
                    
                    move_mode = response_data.get("move_mode", -1)
                    linear_vel = response_data.get("linear_vel", 0.0)
                    result_number = response_data.get("result_number", 1)  # 0表示成功
                    result_msg = response_data.get("result_msg", "")
                    
                    success = (result_number == 1)
                    
                    result = {
                        "success": success,
                        "move_mode": move_mode,
                        "linear_vel": linear_vel,
                        "description": result_msg if success else f"获取失败: {result_msg}",
                        "result_number": result_number
                    }
                    
                    if success:
                        logger.info(f"[ROS2] 获取运动模式成功: {result}")
                    else:
                        logger.error(f"[ROS2] 获取运动模式失败: {result}")
                    
                    return result
                    
                except Exception as e:
                    logger.error(f"[ROS2] 运动模式响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "move_mode": -1,
                        "linear_vel": 0.0,
                        "description": f"响应解析失败: {str(e)}"
                    }
                    
        except Exception as e:
            logger.error(f"[ROS2] 获取运动模式失败: {e}")
            return {
                "success": False,
                "move_mode": -1,
                "linear_vel": 0.0,
                "description": f"获取运动模式失败: {str(e)}"
            }
    def set_robot_rise_jqr(self, rise: bool, duration: int = 0) -> Dict[str, Any]:
        """控制机器人升降 (jqr_ros_msgs版本)
        
        Args:
            rise (bool): 升降状态 (True: 上升, False: 下降)
            duration (int): 执行时间（单位0.1s），缺省表示希望以最快的速度执行
            
        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            # 构造请求数据
            if duration > 0:
                request_data = f'{{"robot_rise": {str(rise).lower()}, "duration": {duration}}}'
            else:
                request_data = f'{{"robot_rise": {str(rise).lower()}}}'
                
            # 调用ROS2服务控制机器人升降
            response = self._call_ros2_service(
                "/set_robot_rise",
                "jqr_ros_msgs/srv/RobotRise",
                request_data
            )
            
            if response is None:
                # 服务调用失败，可能是服务不存在
                result = {
                    "success": False,
                    "rise": rise,
                    "description": "服务 /set_robot_rise 不存在或调用失败"
                }
                logger.error(f"[ROS2] 设置机器人升降失败: {result}")
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
                    logger.error(f"[ROS2] 机器人升降服务返回空响应")
                    return {
                        "success": False,
                        "rise": rise,
                        "description": "机器人升降服务返回空响应"
                    }
                
                # 解析响应数据
                try:
                    # 使用parse_ros2_response工具函数解析响应
                    response_data = parse_ros2_response(response)
                    # 根据jqr_ros_msgs的RobotRise响应格式解析
                    # 响应应包含: result_number, result_msg
                    result_number = response_data.get("result_number", 0)
                    result_msg = response_data.get("result_msg", "")
                    
                    success = (result_number == 1)
                    
                    result = {
                        "success": success,
                        "rise": rise,
                        "description": result_msg if success else f"设置失败: {result_msg}",
                        "result_number": result_number
                    }
                    
                    if success:
                        logger.info(f"[ROS2] 设置机器人升降成功: {result}")
                    else:
                        logger.error(f"[ROS2] 设置机器人升降失败: {result}")
                    
                    return result
                    
                except (json.JSONDecodeError, KeyError) as e:
                    logger.error(f"[ROS2] 设置机器人升降响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "rise": rise,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            logger.error(f"[ROS2] 设置机器人升降失败: {e}")
            return {
                "success": False,
                "rise": rise,
                "description": f"设置机器人{'上升' if rise else '下降'}失败: {str(e)}"
            }
        
    def get_robot_rise_state(self) -> Dict[str, Any]:
        """获取机器人升降状态
        
        Returns:
            Dict[str, Any]: 升降状态信息
        """
        try:
            # 调用ROS2服务获取机器人升降状态
            response = self._call_ros2_service(
                "/get_robot_rise",
                "jqr_ros_msgs/srv/RobotRiseState",
                "{}"
            )
            
            if response is None:
                # 服务调用失败，可能是服务不存在
                result = {
                    "success": False,
                    "state": False,
                    "description": "服务 /get_robot_rise_state 不存在或调用失败"
                }
                logger.error(f"[ROS2] 获取机器人升降状态失败: {result}")
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
                    logger.error(f"[ROS2] 机器人升降状态服务返回空响应")
                    return {
                        "success": False,
                        "state": False,
                        "description": "机器人升降状态服务返回空响应"
                    }
                
                # 解析响应数据
                try:
                    # 使用新的解析函数解析YAML响应
                    response_data = parse_ros2_response(response)
                    
                    robot_rise_state = response_data.get("robot_rise_state", False)
                    result_number = response_data.get("result_number", 0)  # 1表示成功
                    result_msg = response_data.get("result_msg", "")
                    
                    success = (result_number == 1)
                    
                    result = {
                        "success": success,
                        "state": robot_rise_state,
                        "description": result_msg if success else f"获取失败: {result_msg}",
                        "result_number": result_number
                    }
                    
                    if success:
                        logger.info(f"[ROS2] 获取机器人升降状态成功: {result}")
                    else:
                        logger.error(f"[ROS2] 获取机器人升降状态失败: {result}")
                    
                    return result
                    
                except Exception as e:
                    logger.error(f"[ROS2] 机器人升降状态响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "state": False,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            logger.error(f"[ROS2] 获取机器人升降状态失败: {e}")
            return {
                "success": False,
                "state": False,
                "description": f"获取机器人升降状态失败: {str(e)}"
            }
        
    # ======================
    # 机器人俯仰控制相关接口
    # ======================
    
    def set_robot_tilt_jqr(self, angle: float, duration: int = 0) -> Dict[str, Any]:
        """控制机器人俯仰 (jqr_ros_msgs版本)
        
        Args:
            angle (float): 俯仰角度
            duration (int): 执行时间（单位0.1s），缺省表示希望以最快的速度执行
            
        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            # 构造请求数据
            if duration > 0:
                request_data = f'{{"robot_tilt": {angle}, "duration": {duration}}}'
            else:
                request_data = f'{{"robot_tilt": {angle}}}'
                
            # 调用ROS2服务控制机器人俯仰
            response = self._call_ros2_service(
                "/set_robot_tilt",
                "jqr_ros_msgs/srv/RobotTilt",
                request_data
            )
            
            if response is None:
                # 服务调用失败，可能是服务不存在
                result = {
                    "success": False,
                    "angle": angle,
                    "description": "服务 /set_robot_tilt 不存在或调用失败"
                }
                logger.error(f"[ROS2] 设置机器人俯仰角度失败: {result}")
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
                    logger.error(f"[ROS2] 机器人俯仰服务返回空响应")
                    return {
                        "success": False,
                        "angle": angle,
                        "description": "机器人俯仰服务返回空响应"
                    }
                
                # 解析响应数据
                try:
                    # 使用parse_ros2_response工具函数解析响应
                    response_data = parse_ros2_response(response)
                    # 根据jqr_ros_msgs的RobotTilt响应格式解析
                    # 响应应包含: result_number, result_msg
                    result_number = response_data.get("result_number", 0)
                    result_msg = response_data.get("result_msg", "")
                    
                    success = (result_number == 1)
                    
                    result = {
                        "success": success,
                        "angle": angle,
                        "description": result_msg if success else f"设置失败: {result_msg}",
                        "result_number": result_number
                    }
                    
                    if success:
                        logger.info(f"[ROS2] 设置机器人俯仰角度成功: {result}")
                    else:
                        logger.error(f"[ROS2] 设置机器人俯仰角度失败: {result}")
                    
                    return result
                    
                except (json.JSONDecodeError, KeyError) as e:
                    logger.error(f"[ROS2] 设置机器人俯仰角度响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "angle": angle,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            logger.error(f"[ROS2] 设置机器人俯仰角度失败: {e}")
            return {
                "success": False,
                "angle": angle,
                "description": f"设置机器人俯仰角度失败: {str(e)}"
            }
    def _call_ros2_action(self, action_name: str, action_type: str, goal_data: str) -> Optional[str]:
        """调用ROS2动作
        
        Args:
            action_name (str): 动作名称
            action_type (str): 动作类型
            goal_data (str): 目标数据
            
        Returns:
            Optional[str]: 动作执行结果，如果调用失败则返回None
        """
        # 首先检查动作是否存在
        if not self._check_ros2_action_exists(action_name):
            logger.error(f"[ROS2] 动作 {action_name} 不存在，无法调用")
            return None
            
        try:
            # 构造ROS2动作调用命令
            cmd = f"ros2 action send_goal {action_name} {action_type} '{goal_data}'"
            logger.info(f"[ROS2] 执行命令: {cmd}")
            
            # 使用subprocess而不是os.popen来获得更好的控制
            # 设置ROS环境变量
            import os
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = os.environ.get('ROS_DOMAIN_ID', '0')
            
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=30, env=env)
            
            if result.returncode != 0:
                logger.error(f"[ROS2] 动作调用命令执行失败，返回码: {result.returncode}")
                logger.error(f"[ROS2] stderr: {result.stderr}")
                return None
            
            response = result.stdout.strip()
            logger.info(f"[ROS2] 动作调用结果: {response}")
            
            # 检查结果是否为空
            if not response:
                logger.error(f"[ROS2] 动作 {action_name} 返回空响应")
                return None
                
            return response
        except subprocess.TimeoutExpired:
            logger.error(f"[ROS2] 动作调用超时: {action_name}")
            return None
        except Exception as e:
            logger.error(f"[ROS2] 动作调用失败: {e}")
            return None
            
    # ======================
    # 药箱控制相关接口
    # ======================
    
    def set_medicine_box_switch(self, switch: bool, speed_stage: int) -> Dict[str, Any]:
        """控制药箱开关
        
        Args:
            switch (bool): 药箱开关状态 (True: 打开, False: 关闭)
            speed_stage (int): 速度档位 (1: 慢档, 2: 快档)
            
        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            # 构造请求数据
            request_data = f'{{"medicine_box_switch": {str(switch).lower()}, "speed_stage": {speed_stage}}}'
            
            # 调用ROS2服务控制药箱开关
            response = self._call_ros2_service(
                "/set_medicine_box_switch",
                "jqr_ros_msgs/srv/MedicineBoxSwitch",
                request_data
            )
            
            if response is None:
                # 服务调用失败，可能是服务不存在
                result = {
                    "type": "set_medicine_box_switch",
                    "success": False,
                    "error_msg": "服务 /set_medicine_box_switch 不存在或调用失败"
                }
                logger.error(f"[ROS2] 设置药箱开关失败: {result}")
                return result

            # 检查响应是否为空或无效
            if not response or not response.strip():
                logger.error(f"[ROS2] 药箱开关服务返回空响应")
                result = {
                    "type": "set_medicine_box_switch",
                    "success": False,
                    "error_msg": "药箱开关服务返回空响应"
                }
                return result

            # 解析响应数据
            try:
                # 使用parse_ros2_response工具函数解析响应
                response_data = parse_ros2_response(response)
                # 根据jqr_ros_msgs的MedicineBoxSwitch响应格式解析
                # 响应应包含: result_number, result_msg
                result_number = response_data.get("result_number", 0)
                result_msg = response_data.get("result_msg", "")
                
                success = (result_number == 1)
                
                result = {
                    "type": "set_medicine_box_switch",
                    "success": success
                }
                if not success:
                    result["error_msg"] = result_msg or "服务异常"
                
                if success:
                    logger.info(f"[ROS2] 设置药箱开关成功: {result}")
                else:
                    logger.error(f"[ROS2] 设置药箱开关失败: {result}")
                
                return result
                
            except (json.JSONDecodeError, KeyError) as e:
                logger.error(f"[ROS2] 设置药箱开关响应解析失败: {e}, 原始响应: {response}")
                return {
                    "type": "set_medicine_box_switch",
                    "success": False,
                    "error_msg": f"响应解析失败: {str(e)}"
                }
        except Exception as e:
            logger.error(f"[ROS2] 设置药箱开关失败: {e}")
            return {
                "type": "set_medicine_box_switch",
                "success": False,
                "error_msg": f"设置药箱{'打开' if switch else '关闭'}失败: {str(e)}"
            }
    
    def get_medicine_box_state(self) -> Dict[str, Any]:
        """获取药箱状态
        
        Returns:
            Dict[str, Any]: 药箱状态信息
        """
        try:
            # 调用ROS2服务获取药箱状态
            response = self._call_ros2_service(
                "/get_medicine_box_state",
                "jqr_ros_msgs/srv/MedicineBoxState",
                "{}"
            )
            
            if response is None:
                # 服务调用失败，可能是服务不存在
                result = {
                    "success": False,
                    "state": False,
                    "description": "服务 /get_medicine_box_state 不存在或调用失败"
                }
                logger.error(f"[ROS2] 获取药箱状态失败: {result}")
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
                    logger.error(f"[ROS2] 药箱状态服务返回空响应")
                    return {
                        "success": False,
                        "state": False,
                        "description": "药箱状态服务返回空响应"
                    }
                
                # 解析响应数据
                try:
                    # 使用新的解析函数解析YAML响应
                    response_data = parse_ros2_response(response)
                    
                    medicine_box_state = response_data.get("medicine_box_switch_state", False)
                    result_number = response_data.get("result_number", 1)  # 0表示成功
                    result_msg = response_data.get("result_msg", "")
                    
                    success = (result_number == 1)
                    
                    result = {
                        "success": success,
                        "state": medicine_box_state,
                        "description": result_msg if success else f"获取失败: {result_msg}",
                        "result_number": result_number
                    }
                    
                    if success:
                        logger.info(f"[ROS2] 获取药箱状态成功: {result}")
                    else:
                        logger.error(f"[ROS2] 获取药箱状态失败: {result}")
                    
                    return result
                    
                except Exception as e:
                    logger.error(f"[ROS2] 药箱状态响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "state": False,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            logger.error(f"[ROS2] 获取药箱状态失败: {e}")
            return {
                "success": False,
                "state": False,
                "description": f"获取药箱状态失败: {str(e)}"
            }

    def set_rgb_light_strip(self, red: int, green: int, blue: int, brightness: int = 255) -> Dict[str, Any]:
        """控制RGB灯带颜色与亮度 (jqr_ros_msgs版本)
        
        Args:
            red (int): 红色分量 0~255
            green (int): 绿色分量 0~255
            blue (int): 蓝色分量 0~255
            brightness (int): 亮度 0~255，默认255
            
        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            request_data = f'{{"red": {red}, "green": {green}, "blue": {blue}, "brightness": {brightness}}}'
            response = self._call_ros2_service(
                "/set_rgb_light_strip",
                "jqr_ros_msgs/srv/RgbLightStrip",
                request_data
            )
            if response is None:
                result = {
                    "success": False,
                    "description": "服务 /set_rgb_light_strip 不存在或调用失败"
                }
                logger.error(f"[ROS2] 设置RGB灯带失败: {result}")
                return result
            else:
                if not response or not response.strip():
                    return {
                        "success": False,
                        "description": "RGB灯带服务返回空响应"
                    }
                try:
                    response_data = parse_ros2_response(response)
                    result_number = response_data.get("result_number", 0)
                    result_msg = response_data.get("result_msg", "")
                    success = (result_number == 1)
                    result = {
                        "success": success,
                        "description": result_msg if success else f"设置失败: {result_msg}",
                        "result_number": result_number
                    }
                    if success:
                        logger.info(f"[ROS2] 设置RGB灯带成功: {result}")
                    else:
                        logger.error(f"[ROS2] 设置RGB灯带失败: {result}")
                    return result
                except Exception as e:
                    logger.error(f"[ROS2] RGB灯带响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            logger.error(f"[ROS2] 设置RGB灯带失败: {e}")
            return {
                "success": False,
                "description": f"设置RGB灯带失败: {str(e)}"
            }

    def get_rgb_light_strip_state(self) -> Dict[str, Any]:
        """获取RGB灯带状态
        
        Returns:
            Dict[str, Any]: 灯带状态信息
        """
        try:
            response = self._call_ros2_service(
                "/get_rgb_light_strip_state",
                "jqr_ros_msgs/srv/RgbLightStripState",
                "{}"
            )
            if response is None:
                result = {
                    "success": False,
                    "description": "服务 /get_rgb_light_strip_state 不存在或调用失败"
                }
                logger.error(f"[ROS2] 获取RGB灯带状态失败: {result}")
                return result
            else:
                if not response or not response.strip():
                    return {
                        "success": False,
                        "description": "RGB灯带状态服务返回空响应"
                    }
                try:
                    response_data = parse_ros2_response(response)
                    red = response_data.get("red", 0)
                    green = response_data.get("green", 0)
                    blue = response_data.get("blue", 0)
                    brightness = response_data.get("brightness", 0)
                    result_number = response_data.get("result_number", 0)
                    result_msg = response_data.get("result_msg", "")
                    success = (result_number == 1)
                    result = {
                        "success": success,
                        "red": red,
                        "green": green,
                        "blue": blue,
                        "brightness": brightness,
                        "description": result_msg if success else f"获取失败: {result_msg}",
                        "result_number": result_number
                    }
                    if success:
                        logger.info(f"[ROS2] 获取RGB灯带状态成功: {result}")
                    else:
                        logger.error(f"[ROS2] 获取RGB灯带状态失败: {result}")
                    return result
                except Exception as e:
                    logger.error(f"[ROS2] RGB灯带状态响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            logger.error(f"[ROS2] 获取RGB灯带状态失败: {e}")
            return {
                "success": False,
                "description": f"获取RGB灯带状态失败: {str(e)}"
            }

    def get_robot_tilt_state(self) -> Dict[str, Any]:
        """获取机器人俯仰状态
        
        Returns:
            Dict[str, Any]: 俯仰状态信息
        """
        try:
            # 调用ROS2服务获取机器人俯仰状态
            response = self._call_ros2_service(
                "/get_robot_tilt",
                "jqr_ros_msgs/srv/RobotTiltState",
                "{}"
            )
            
            if response is None:
                # 服务调用失败，可能是服务不存在
                result = {
                    "success": False,
                    "angle": 0.0,
                    "description": "服务 /get_robot_tilt_state 不存在或调用失败"
                }
                logger.error(f"[ROS2] 获取机器人俯仰状态失败: {result}")
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
                    logger.error(f"[ROS2] 机器人俯仰状态服务返回空响应")
                    return {
                        "success": False,
                        "angle": 0.0,
                        "description": "机器人俯仰状态服务返回空响应"
                    }
                
                # 解析响应数据
                try:
                    # 尝试解析JSON响应
                    # 使用新的解析函数解析YAML响应
                    response_data = parse_ros2_response(response)
                    
                    robot_tilt_state = response_data.get("robot_tilt_state", 0.0)
                    result_number = response_data.get("result_number", 1)  # 0表示成功
                    result_msg = response_data.get("result_msg", "")
                    
                    success = (result_number == 1)
                    
                    result = {
                        "success": success,
                        "angle": robot_tilt_state,
                        "description": result_msg if success else f"获取失败: {result_msg}",
                        "result_number": result_number
                    }
                    
                    if success:
                        logger.info(f"[ROS2] 获取机器人俯仰状态成功: {result}")
                    else:
                        logger.error(f"[ROS2] 获取机器人俯仰状态失败: {result}")
                    
                    return result
                    
                except Exception as e:
                    logger.error(f"[ROS2] 机器人俯仰状态响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "angle": 0.0,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            logger.error(f"[ROS2] 获取机器人俯仰状态失败: {e}")
            return {
                "success": False,
                "angle": 0.0,
                "description": f"获取机器人俯仰状态失败: {str(e)}"
            }
        
    def set_robot_rise(self, height: float) -> Dict[str, Any]:
        """控制机身升降
        
        Args:
            height (float): 升降高度（米）
            
        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            # 调用ROS2动作控制机身升降
            # 假设有一个/set_robot_rise动作
            response = self._call_ros2_action(
                "/set_robot_rise",
                "jqr_ros_msgs/action/SetRobotRise",
                f"{{height: {height}}}"
            )
            
            if response:
                result = {
                    "success": True,
                    "height": height,
                    "description": f"机身升降高度已设置为 {height} 米"
                }
                logger.info(f"[ROS2] 设置机身升降高度: {result}")
                return result
            else:
                result = {
                    "success": False,
                    "height": height,
                    "description": f"设置机身升降高度失败"
                }
                logger.error(f"[ROS2] 设置机身升降高度失败: {result}")
                return result
        except Exception as e:
            logger.error(f"[ROS2] 设置机身升降高度失败: {e}")
            return {
                "success": False,
                "height": height,
                "description": f"设置机身升降高度失败: {str(e)}"
            }
        
    def get_robot_rise(self) -> Dict[str, Any]:
        """获取机身升降状态
        
        Returns:
            Dict[str, Any]: 升降状态信息
        """
        try:
            # 调用ROS2服务获取机身升降状态
            # 假设有一个/get_robot_rise服务
            response = self._call_ros2_service(
                "/get_robot_rise",
                "jqr_ros_msgs/srv/GetRobotRise",
                "{}"
            )
            
            if response:
                # 使用parse_ros2_response工具函数解析响应
                response_data = parse_ros2_response(response)
                # 根据实际的服务响应格式进行解析
                height = response_data.get("height", 0.0)
                result_number = response_data.get("result_number", 0)
                result_msg = response_data.get("result_msg", "")
                result = {
                    "height": height,  # 从响应中提取的实际值
                    "description": f"机身升降高度为 {height} 米"
                }
                logger.info(f"[ROS2] 获取机身升降状态: {result}")
                return result
            else:
                # 如果服务调用失败，返回默认值
                result = {
                    "success": False,
                }
                logger.info(f"[ROS2] 获取机身升降状态(默认值): {result}")
                return result
        except Exception as e:
            logger.error(f"[ROS2] 获取机身升降状态失败: {e}")
            return {
                "success": False,
                "height": 0.0,
                "description": f"获取机身升降状态失败: {str(e)}"
            }
        
    # ======================
    # 屏幕俯仰控制相关接口
    # ======================
    
    def set_screen_tilt_jqr(self, angle: float, duration: int = 0) -> Dict[str, Any]:
        """控制屏幕俯仰 (jqr_ros_msgs版本)
        
        Args:
            angle (float): 俯仰角度
            duration (int): 执行时间（单位0.1s），缺省表示希望以最快的速度执行
            
        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            # 构造请求数据
            if duration > 0:
                request_data = f'{{"screen_tilt": {angle}, "duration": {duration}}}'
            else:
                request_data = f'{{"screen_tilt": {angle}}}'
                
            # 调用ROS2服务控制屏幕俯仰
            response = self._call_ros2_service(
                "/set_screen_tilt",
                "jqr_ros_msgs/srv/ScreenTilt",
                request_data
            )
            
            if response is None:
                # 服务调用失败，可能是服务不存在
                result = {
                    "success": False,
                    "angle": angle,
                    "description": "服务 /set_screen_tilt 不存在或调用失败"
                }
                logger.error(f"[ROS2] 设置屏幕俯仰角度失败: {result}")
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
                    logger.error(f"[ROS2] 屏幕俯仰服务返回空响应")
                    return {
                        "success": False,
                        "angle": angle,
                        "description": "屏幕俯仰服务返回空响应"
                    }
                
                # 解析响应数据
                try:
                    # 使用parse_ros2_response工具函数解析响应
                    response_data = parse_ros2_response(response)
                    # 根据jqr_ros_msgs的ScreenTilt响应格式解析
                    # 响应应包含: result_number, result_msg
                    result_number = response_data.get("result_number", 0)
                    result_msg = response_data.get("result_msg", "")
                    
                    success = (result_number == 1)
                    
                    result = {
                        "success": success,
                        "angle": angle,
                        "description": result_msg if success else f"设置失败: {result_msg}",
                        "result_number": result_number
                    }
                    
                    if success:
                        logger.info(f"[ROS2] 设置屏幕俯仰角度成功: {result}")
                    else:
                        logger.error(f"[ROS2] 设置屏幕俯仰角度失败: {result}")
                    
                    return result
                    
                except (json.JSONDecodeError, KeyError) as e:
                    logger.error(f"[ROS2] 设置屏幕俯仰角度响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "angle": angle,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            logger.error(f"[ROS2] 设置屏幕俯仰角度失败: {e}")
            return {
                "success": False,
                "angle": angle,
                "description": f"设置屏幕俯仰角度失败: {str(e)}"
            }
        
    def get_screen_tilt_state(self) -> Dict[str, Any]:
        """获取屏幕俯仰状态
        
        Returns:
            Dict[str, Any]: 俯仰状态信息
        """
        try:
            # 调用ROS2服务获取屏幕俯仰状态
            response = self._call_ros2_service(
                "/get_screen_tilt",
                "jqr_ros_msgs/srv/ScreenTiltState",
                "{}"
            )
            
            if response is None:
                # 服务调用失败，可能是服务不存在
                result = {
                    "success": False,
                    "angle": 0.0,
                    "description": "服务 /get_screen_tilt_state 不存在或调用失败"
                }
                logger.error(f"[ROS2] 获取屏幕俯仰状态失败: {result}")
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
                    logger.error(f"[ROS2] 屏幕俯仰状态服务返回空响应")
                    return {
                        "success": False,
                        "angle": 0.0,
                        "description": "屏幕俯仰状态服务返回空响应"
                    }
                
                # 解析响应数据
                try:
                    # 使用新的解析函数解析YAML响应
                    response_data = parse_ros2_response(response)
                    
                    screen_tilt_state = response_data.get("screen_tilt_state", 0.0)
                    result_number = response_data.get("result_number", 1)  # 0表示成功
                    result_msg = response_data.get("result_msg", "")
                    
                    success = (result_number == 1)
                    
                    result = {
                        "success": success,
                        "angle": screen_tilt_state,
                        "description": result_msg if success else f"获取失败: {result_msg}",
                        "result_number": result_number
                    }
                    
                    if success:
                        logger.info(f"[ROS2] 获取屏幕俯仰状态成功: {result}")
                    else:
                        logger.error(f"[ROS2] 获取屏幕俯仰状态失败: {result}")
                    
                    return result
                    
                except Exception as e:
                    logger.error(f"[ROS2] 屏幕俯仰状态响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "angle": 0.0,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            logger.error(f"[ROS2] 获取屏幕俯仰状态失败: {e}")
            return {
                "success": False,
                "angle": 0.0,
                "description": f"获取屏幕俯仰状态失败: {str(e)}"
            }
def battery_callback(msg):
    """电池电量回调函数 - 收到信息后立马通过USB串口发送
    
    Args:
        msg: 电池电量消息
    """
    global battery_level, smart_robot_agent_instance
    
    try:
        # 更新电池电量
        battery_level = msg.battery_power_state
        logger.info(f"[BATTERY] 收到电池电量更新: {battery_level}%")
        
        # 构造电池电量消息
        battery_message = {
            "type": "battery_update",
            "battery_level": battery_level,
            "timestamp": int(time.time())
        }
        
        # 通过USB串口发送电池电量信息
        if smart_robot_agent_instance and hasattr(smart_robot_agent_instance, 'usb_manager'):
            try:
                # 尝试获取当前事件循环
                loop = asyncio.get_event_loop()
                if loop.is_running():
                    # 如果事件循环正在运行，创建任务
                    asyncio.create_task(smart_robot_agent_instance.usb_manager.send_message(battery_message))
                else:
                    # 如果事件循环没有运行，使用run_until_complete
                    loop.run_until_complete(smart_robot_agent_instance.usb_manager.send_message(battery_message))
            except RuntimeError:
                # 如果没有事件循环，创建一个新的
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(smart_robot_agent_instance.usb_manager.send_message(battery_message))
            logger.info(f'电池电量已通过USB发送: {battery_level:.1f}%')
        else:
            logger.warning('USB管理器不可用，无法发送电池电量信息')
            
    except Exception as e:
        logger.error(f'处理电池电量回调时出错: {e}')

# 注释：不再需要单独的spin循环，使用主节点的spin
# def ros2_spin_loop():
#     """ROS2 spin循环（已弃用，使用主节点spin）"""
#     global battery_node, battery_thread_running
#     
#     if not ROS2_AVAILABLE:
#         logger.error("ROS2不可用，无法启动ROS2 spin循环")
#         return
#         
#     try:
#         while rclpy is not None and rclpy.ok() and battery_thread_running and battery_node:
#             rclpy.spin_once(battery_node, timeout_sec=0.1)
#             time.sleep(0.1)  # 短暂休眠以避免CPU占用过高
#     except Exception as e:
#         logger.error(f"ROS2 spin循环出错: {e}")
#     finally:
#         if battery_node:
#             battery_node.destroy_node()
#             battery_node = None


# ======================
# USB串口服务器管理器
# ======================

class USBCoordinateManager:
    """USB坐标管理器，替代WebSocketServer"""
    
    def __init__(self, agent=None):
        self.agent = agent
        self.serial_manager = SerialManager(port=USB_SERIAL_PORT, baudrate=USB_SERIAL_BAUDRATE)
        self.connected = False
        

        
    async def initialize(self):
        """初始化USB串口连接"""
        try:
            # 连接到串口设备
            self.connected = await self.serial_manager.connect()
            if self.connected:
                logger.info("USB串口连接成功")
                # 添加消息回调
                self.serial_manager.add_callback(self._handle_received_message)
                # 开始接收数据
                self.serial_manager.start_receiving()
                logger.info("USB串口通信已启动，直接双向通信")
                return True
            else:
                logger.error("USB串口连接失败")
                return False
        except Exception as e:
            logger.error(f"初始化USB串口失败: {e}")
            return False
    
    def _handle_received_message(self, message: Dict[Any, Any]):
        """处理接收到的消息"""
        try:
            logger.info(f"接收到USB消息: {message}")
            
            # 将消息转发给agent处理
            if self.agent and hasattr(self.agent, 'handle_client_message'):
                # 将消息添加到消息队列中
                self.agent.message_queue.put_nowait(message)
                logger.info(f"消息已添加到队列，队列大小: {self.agent.message_queue.qsize()}")
            else:
                logger.warning("Agent或handle_client_message方法不可用")
        except Exception as e:
            logger.error(f"处理USB消息失败: {e}")
    
    async def send_message(self, message: Dict[Any, Any]) -> bool:
        """发送消息到客户端"""
        try:
            if not self.connected:
                logger.warning("USB串口未连接，无法发送消息")
                return False
            
            success = self.serial_manager.send_message(message)
            if success:
                logger.info(f"已发送USB消息: {message}")
            else:
                logger.error(f"发送USB消息失败: {message}")
            return success
        except Exception as e:
            logger.error(f"发送USB消息异常: {e}")
            return False
    

    

    

    
    def cleanup(self):
        """清理资源"""
        try:
            self.serial_manager.stop_receiving()
            logger.info("USB串口资源已清理")
        except Exception as e:
            logger.error(f"清理USB串口资源失败: {e}")

# ======================
# 工具函数：外部系统控制
# ======================
def stop_following() -> bool:
    """停止跟随"""
    # TODO: 实现真实的停止跟随功能
    logger.warning("停止跟随功能尚未实现")
    return False

def stop_navigation() -> bool:
    """停止导航"""
    # TODO: 实现真实的停止导航功能
    logger.warning("停止导航功能尚未实现")
    return False

def notify_navigation_model_stop_following():
    """通知导航模型停止跟随"""
    # TODO: 实现真实的通知导航模型功能
    logger.warning("通知导航模型停止跟随功能尚未实现")

def stop_follow() -> Dict[str, Any]:
    """
    停止跟随
    
    Returns:
        Dict[str, Any]: 工具执行结果
    """
    try:
        result = stop_following()
        if result:
            notify_navigation_model_stop_following()
            success_msg = "已停止跟随"
            
            result_data = {
                "type": "stop_follow",
                "success": True,
                "result": success_msg
            }
            
            return result_data
        else:
            error_msg = "停止跟随失败"
            
            result_data = {
                "type": "stop_follow",
                "success": False,
                "result": error_msg
            }
            
            return result_data
    except Exception as e:
        error_msg = f"停止跟随失败: {str(e)}"
        
        result_data = {
            "type": "stop_follow",
            "success": False,
            "result": error_msg
        }
        
        return result_data

def stop_navigate() -> Dict[str, Any]:
    """
    停止导航
    
    Returns:
        Dict[str, Any]: 工具执行结果
    """
    try:
        result = stop_navigation()
        if result:
            success_msg = "已停止导航"
            
            result_data = {
                "type": "stop_navigate",
                "success": True,
                "result": success_msg
            }
            
            return result_data
        else:
            error_msg = "停止导航失败"
            
            result_data = {
                "type": "stop_navigate",
                "success": False,
                "result": error_msg
            }
            
            return result_data
    except Exception as e:
        error_msg = f"停止导航失败: {str(e)}"
        
        result_data = {
            "type": "stop_navigate",
            "success": False,
            "result": error_msg
        }
        
        return result_data

# ======================
# 智能机器人Agent
# ======================

class SmartRobotAgent:
    """智能机器人Agent - USB串口版本"""
    
    def __init__(self):
        self.ros2_interface = ROS2Interface()
        
        # 创建USB串口通信管理器
        self.usb_manager = USBCoordinateManager(self)
        
        # 任务中断标志
        self._task_interrupted = False
        
        # 本地模型连接相关
        self.local_model_websocket = None
        self.local_model_connected = False
        # self.local_model_uri = "ws://localhost:8769"
        # self.local_model_uri = "ws://192.168.50.144:8000/ws/navigate"
        self.local_model_uri = "ws://192.168.8.229:8000/ws/navigate"
        # 任务执行状态跟踪
        self.active_navigation_tasks = set()  # 正在执行的导航任务ID集合
        self.task_execution_lock = asyncio.Lock()  # 任务执行锁
        
        # 本地模型连接锁
        self.local_model_lock = asyncio.Lock()
        
        # 消息队列用于处理USB接收的消息
        self.message_queue = queue.Queue()
        
        # 退出控制标志
        self._running = False
    
    async def initialize(self):
        """初始化agent"""
        try:
            # 设置全局实例引用
            global smart_robot_agent_instance
            smart_robot_agent_instance = self
            
            # 启动ROS2订阅
            if ROS2_AVAILABLE:
                # 启动电池电量监控
                battery_success = self.ros2_interface.start_battery_monitoring()
                if battery_success:
                    logger.info("电池电量监控已启动")
                else:
                    logger.warning("电池电量监控启动失败")
                
                # 启动位置订阅
                position_success = self.ros2_interface.subscribe_robot_position()
                if position_success:
                    logger.info("机器人位置订阅已启动")
                else:
                    logger.warning("机器人位置订阅启动失败")
            
            # 初始化USB串口通信
            usb_connected = await self.usb_manager.initialize()
            if not usb_connected:
                logger.warning("USB串口连接失败，无法继续初始化Agent")
                return False
            
            # 连接到本地模型
            max_retries = 1
            for attempt in range(max_retries):
                try:
                    logger.info(f"尝试连接本地模型服务器 (第{attempt + 1}次)...")
                    connected = await self.connect_to_local_model()
                    if connected:
                        logger.info("成功连接到本地模型服务器")
                        break
                    else:
                        logger.warning(f"连接本地模型服务器失败 (第{attempt + 1}次)")
                        if attempt < max_retries - 1:
                            await asyncio.sleep(2)
                except Exception as e:
                    logger.error(f"连接本地模型服务器时出错 (第{attempt + 1}次): {e}")
                    if attempt < max_retries - 1:
                        await asyncio.sleep(1)
            
            # 启动消息处理循环
            self._running = True
            asyncio.create_task(self._message_processor())
            
            logger.info("SmartRobotAgent初始化完成")
            return True
        except Exception as e:
            logger.error(f"初始化SmartRobotAgent失败: {e}")
            return False
    
    async def _message_processor(self):
        """消息处理循环"""
        logger.info("消息处理循环已启动")
        while self._running:
            try:
                # ROS2回调现在由独立线程处理，这里不再需要spin_once
                
                # 从队列中获取消息
                try:
                    message = self.message_queue.get(timeout=0.1)
                    await self.handle_client_message(message)
                except queue.Empty:
                    await asyncio.sleep(0.01)
                    continue
            except Exception as e:
                logger.error(f"处理消息时出错: {e}")
                await asyncio.sleep(0.1)
        logger.info("消息处理循环已退出")
    
    async def handle_client_message(self, message: Dict[Any, Any]):
        """处理来自客户端的消息"""
        try:
            logger.info(f"收到客户端消息: {message}")
            
            # 检查消息格式并提取实际的任务
            task_to_execute = None
            
            # 格式1: {"name": "任务名", "task": {"type": "...", "params": {...}}}
            if "task" in message and isinstance(message["task"], dict):
                task_to_execute = message["task"]
                # 添加任务名称信息
                task_to_execute["task_name"] = message.get("name", "未知任务")
                logger.info(f"检测到格式1消息: {task_to_execute}")
            
            # 格式2: 直接的任务格式 {"type": "...", "params": {...}}
            elif "type" in message:
                task_to_execute = message
                logger.info(f"检测到格式2消息: {task_to_execute}")
            
            # 格式3: 其他格式（如ping命令）
            else:
                # 对于不识别的格式，尝试作为简单的command处理
                if "command" in message:
                    result = {
                        "type": message.get("command"),
                        "success": True,
                        "message": f"收到命令: {message.get('command')}",
                        "response": message,
                        "timestamp": int(time.time())
                    }
                    await self.send_response_to_client(result)
                    return
                
                # 无法识别的格式
                error_response = {
                    "type": message.get("type", "unknown"),
                    "success": False,
                    "error_msg": f"无法识别的消息格式: {list(message.keys())}",
                    "timestamp": int(time.time())
                }
                await self.send_response_to_client(error_response)
                return
            
            # 执行任务
            if task_to_execute:
                logger.info(f"准备执行任务: {task_to_execute}")
                result = await self.execute_task(task_to_execute)
                
                # 添加任务名称到响应中
                if "task_name" in task_to_execute:
                    result["task_name"] = task_to_execute["task_name"]
                
                # 发送响应给客户端
                await self.send_response_to_client(result)
        
        except Exception as e:
            logger.error(f"处理客户端消息失败: {e}")
            error_response = {
                "type": message.get("type", "unknown"),
                "success": False,
                "error_msg": str(e)
            }
            await self.send_response_to_client(error_response)
    
    async def send_response_to_client(self, response: Dict[Any, Any]):
        """发送响应给客户端"""
        try:
            # 发送到串口
            success = await self.usb_manager.send_message(response)
            if success:
                logger.info(f"响应已发送到客户端: {response.get('type', 'unknown')}")
            else:
                logger.error(f"发送响应失败: {response}")
        except Exception as e:
            logger.error(f"发送响应失败: {e}")
    
    async def execute_task(self, task: Dict[str, Any]) -> Dict[str, Any]:
        """执行单个任务
        
        Args:
            task (Dict[str, Any]): 任务字典，包含任务类型和参数
            
        Returns:
            Dict[str, Any]: 任务执行结果
        """
        try:
            task_type = task.get("type")
            task_params = task.get("params", {})
            
            logger.info(f"[EXECUTE_TASK] 执行任务类型: {task_type}, 参数: {task_params}")
            
            if not task_type:
                return {"status": "error", "result": "任务类型为空"}
            
            # 直接使用params中的参数，通过_execute_task_by_type执行
            result = await self._execute_task_by_type(task_type, task_params)
            
            # 直接返回字典结果
            return result
                
        except Exception as e:
            logger.error(f"执行任务时出错: {e}")
            return {
                "status": "error",
                "result": f"执行任务时出错: {str(e)}"
            }

    def _convert_agent_result_to_client_response(self, result: Dict[str, Any]) -> Dict[str, Any]:
        """
        将agent内部的执行结果转换为客户端期望的响应格式
        
        Args:
            result (Dict[str, Any]): agent内部工具函数的返回结果
            
        Returns:
            Dict[str, Any]: 客户端期望的响应格式
        """
        # 判断任务是否成功 - 兼容多种返回格式
        success = False
        error_msg = ""
        
        # 1. 检查标准格式
        if result.get("success") is not None:
            success = bool(result.get("success"))
        # 2. 检查status格式
        elif result.get("status") == "error":
            success = False
        # 3. 检查result字段是否包含错误信息
        elif isinstance(result.get("result"), str) and any(word in result.get("result", "").lower() for word in ["error", "失败", "异常", "failed"]):
            success = False
        else:
            # 默认情况下，如果没有明确的错误标识，认为成功
            success = True
        
        # 提取错误信息
        if not success:
            # 优先使用error_msg字段
            if result.get("error_msg"):
                error_msg = result.get("error_msg")
            # 其次使用result字段（如果是字符串）
            elif isinstance(result.get("result"), str):
                error_msg = result.get("result")
            # 最后使用status字段
            elif result.get("status") == "error":
                error_msg = "任务执行失败"
            else:
                error_msg = "未知错误"
        
        # 构建客户端响应
        client_response = {
            "result": success,
            "error_msg": error_msg,
            "type": result.get("type", "unknown")
        }
        
        # 保留原始result中的所有其他字段（除了已处理的字段）
        # 注意：这里我们排除"result"字段，因为我们已经设置了布尔值
        excluded_keys = {"success", "status", "error_msg", "type", "result"}
        for key, value in result.items():
            if key not in excluded_keys:
                client_response[key] = value
        
        # 只有当原始result字段不是错误字符串时，才保留非字符串类型的result值
        if "result" in result and not isinstance(result["result"], str):
            client_response["result"] = result["result"]
        
        return client_response
    
    async def _execute_task_by_type(self, task_type: str, params: Dict[str, Any]) -> Dict[str, Any]:
        """根据任务类型执行相应的工具函数"""
        # 处理嵌套的参数结构（兼容测试脚本的新格式）
        if 'tool' in params and 'arguments' in params:
            # 新格式：{"tool": "go_to_object", "arguments": {"user_prompt": "...", "world_position": [...]}}
            nested_params = params.get('arguments', {})
            # 合并参数，优先使用arguments中的参数
            merged_params = {**params, **nested_params}
            params = merged_params
        
        # 对于导航相关任务，在开始前记录当前位置
        if task_type in ["go_to_object", "go_find_person", "follow_person"]:
            self.record_position_before_navigation()
        
        if task_type == "find_object":
            return await self.find_object(**params)
        elif task_type == "go_to_object":
            return await self.go_to_object(**params)
        elif task_type == "go_find_person":
            return await self.go_find_person(**params)
        elif task_type == "follow_person":
            return await self.follow_person(**params)
        elif task_type == "back_to_last_position":
                result = await self.back_to_last_position(**params)
                result["type"] = task_type
                return result
        elif task_type == "stop_follow":
            return stop_follow()
        elif task_type == "stop_navigate":
            return stop_navigate()
        elif task_type == "stop_move" and hasattr(self, 'ros2_interface'):
            result = await self.stop_move()
            result["type"] = task_type
            return result
        # ROS2接口任务类型 - 确保返回结果包含type字段
        elif task_type == "get_move_mode" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.get_move_mode()
            result["type"] = task_type
            return result
        elif task_type == "get_medicine_box_state" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.get_medicine_box_state()
            result["type"] = task_type
            return result
        elif task_type == "set_medicine_box_switch" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.set_medicine_box_switch(**params)
            result["type"] = task_type
            return result
        elif task_type == "get_robot_rise_state" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.get_robot_rise_state()
            result["type"] = task_type
            return result
        elif task_type == "set_robot_rise_jqr" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.set_robot_rise_jqr(**params)
            result["type"] = task_type
            return result
        elif task_type == "get_robot_tilt_state" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.get_robot_tilt_state()
            result["type"] = task_type
            return result
        elif task_type == "set_robot_tilt_jqr" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.set_robot_tilt_jqr(**params)
            result["type"] = task_type
            return result
        elif task_type == "get_screen_tilt_state" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.get_screen_tilt_state()
            result["type"] = task_type
            return result
        elif task_type == "set_screen_tilt_jqr" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.set_screen_tilt_jqr(**params)
            result["type"] = task_type
            return result
        elif task_type == "set_laser_pointer" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.set_laser_pointer(**params)
            result["type"] = task_type
            return result
        elif task_type == "get_laser_pointer_state" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.get_laser_pointer_state()
            result["type"] = task_type
            return result
        elif task_type == "set_rgb_light_strip" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.set_rgb_light_strip(**params)
            result["type"] = task_type
            return result
        elif task_type == "get_rgb_light_strip_state" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.get_rgb_light_strip_state()
            result["type"] = task_type
            return result
        else:
            return {
                "type": task_type,
                "status": "error",
                "result": f"未知的任务类型: {task_type}"
            }    

    # ======================
    # 任务执行方法
    # ======================
    def query_asm_object(self,obj_name: str) -> Optional[Dict[str, Any]]:
        """查询ASM中的对象信息"""
        
        if not os.path.exists(ASM_JSON_PATH):
            print(f"文件 {ASM_JSON_PATH} 不存在")
            return None
            
        # 只有在文件格式确实有问题时才尝试修复
        try:
            with open(ASM_JSON_PATH, 'r', encoding='utf-8') as f:
                json.load(f)
        except json.JSONDecodeError:
            logger.warning("ASM JSON文件格式错误，尝试修复...")
            fix_asm_json_format()
        
        try:
            with open(ASM_JSON_PATH, 'r', encoding='utf-8') as f:
                data = json.load(f)
                
            # 获取objects数组
            objects = data.get("objects", [])
            logger.info(f"找到 {len(objects)} 个对象")
            
            # 遍历objects数组查找匹配的对象
            for obj in objects:
                # 确保进行精确匹配，避免部分匹配或模糊匹配
                if obj.get("name") == obj_name:
                    # 使用正确的字段名
                    world_position = obj.get("world_position", [])
                    pixel_position = obj.get("pixel_position", [])
                    
                    if len(world_position) >= 2 and len(pixel_position) >= 2:
                        return {
                            "location": {"x": pixel_position[0], "y": pixel_position[1]},
                            "world_position": world_position,
                            "pixel_position": pixel_position,
                            "last_time": obj.get("last_show_time", "2025-11-02T10:00:00"),
                            "exist_or_not": obj.get("exist_or_not", 0),
                            "object_description": obj.get("object_description", "")
                        }
                        
            logger.info(f"未找到名为 '{obj_name}' 的对象")
        except Exception as e:
            logger.error(f"[ERROR] ASM read failed: {e}")
            import traceback
            traceback.print_exc()
        return None

    def query_history_db(self,obj_name: str) -> Optional[Dict[str, Any]]:
        """查询历史数据库中的对象信息"""
        try:
            conn = sqlite3.connect(DB_PATH)
            cursor = conn.cursor()
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS objects (
                    id INTEGER PRIMARY KEY,
                    name TEXT,
                    world_x REAL,
                    world_y REAL,
                    last_show_time TEXT TIMESTAMP,
                    exist_or_not INTEGER,
                    object_description TEXT
                )
            """)
            cursor.execute(
                "SELECT id, name, world_x, world_y, last_show_time, exist_or_not, object_description FROM objects WHERE name = ? ORDER BY last_show_time DESC LIMIT 1",
                (obj_name,)
            )
            row = cursor.fetchone()
            conn.close()
            if row:
                logger.info(f"[DB] Found object {obj_name} with id {row[0]} at location ({row[2]}, {row[3]})")
                return {
                    "object_id": row[0],
                    "name": row[1],
                    "world_x": row[2],
                    "world_y": row[3],
                    "last_show_time": row[4],
                    "exist_or_not": row[5],
                    "object_description": row[6]
                }
            else:
                logger.info(f"[DB] Object {obj_name} not found in database")
        except Exception as e:
            logger.error(f"[ERROR] DB query failed: {e}")
        return None
    async def find_object(self,obj_name: str) -> Dict[str, Any]:
        """
        查找物品的位置信息，按照ASM→DB→探索的优先级执行
        
        Args:
            obj_name (str): 物品名称
        
        Returns:
            Dict[str, Any]: 工具执行结果
        """
        logger.info(f"[FIND_OBJECT] 开始查找物品/人员: {obj_name}")
        try:
            # Step 1: ASM查询（最高优先级）
            asm_res = self.query_asm_object(obj_name)
            if asm_res:
                #打印asm_res
                print(asm_res)
                loc = asm_res["location"]
                logger.info(f"[FIND_OBJECT] 在ASM中找到 {obj_name} 位置: ({loc['x']}, {loc['y']})")
                
                # ASM找到：返回位置信息，询问用户是否需要导航
                result_msg = f"找到 {obj_name} 的位置：像素坐标 ({loc['x']}, {loc['y']})"
                logger.info(f"[FIND_OBJECT] {result_msg}")
                
                # 按照新格式返回结果，包含像素位置
                result_data = {
                    "type": "find_object",
                    "success": True,
                    "pixel_position": asm_res.get("pixel_position", []),  # 添加像素位置
                    "position_description": asm_res.get("object_description", "")  # 使用ASM中的描述
                }
                
                return result_data

            # Step 2: DB查询
            db_res = self.query_history_db(obj_name)
            if not db_res:
                # DB没有找到：返回失败结果
                result_data = {
                    "type": "find_object",
                    "success": False,
                    "pixel_position": None,
                    "position_description": None
                }
                
                return result_data

            logger.info(f"[FIND_OBJECT] 在DB中找到 {obj_name} 记录，时间: {db_res['last_show_time']}")
            
            # DB找到：直接反馈结果，不询问导航
            result_data = {
                "type": "find_object",
                "success": True,
                "pixel_position": [db_res["world_x"], db_res["world_y"]],
                "position_description": db_res["object_description"]
            }
            
            return result_data
        except Exception as e:
            result_data = {
                "type": "find_object",
                "success": False,
                "pixel_position": None,
                "position_description": None
            }
        
            return result_data
    
    async def go_to_object(self, obj_name: str, pixel_position: Optional[List[float]] = None) -> Dict[str, Any]:
        """导航到物体位置"""
        try:
            logger.info(f"[GO_TO_OBJECT] 开始导航到物体: {obj_name}")
            
            # 检查是否有新格式的tool和arguments
            model_data = {
                "type": "go_to_object",
                "user_prompt": f"去{obj_name}旁边",
                "obj_name": obj_name,
                "pixel_position": pixel_position
            }
            # final_sent = False
            result_msg = {
                        "type": "go_to_object",
                        "success": False,
                        "err_msg": ""}
            response = await self.send_to_local_model(model_data)
            if response and response.get("error_msg") == "无法连接到本地模型服务器":
                result_msg = {
                        "type": "go_to_object",
                        "success": False,
                        "err_msg": "无法连接到本地模型服务器"}
                return result_msg
            result_msg["success"] = response.get("success", False)
            result_msg["err_msg"] = response.get("error_msg", "")
            return result_msg                        
            # while not final_sent:
            # # 发送到本地模型并获取响应
            #     if isinstance(response, dict) and "command" in response:
            #         cmd = response["command"]
            #         logger.info(f"[GO_TO_OBJECT] 收到中间信息: {cmd}")
            #         await self.usb_manager.send_message({"type": "go_to_object", "command": cmd})
            #         continue                
            #     if isinstance(response, dict) and "success" in response:
            #         success = response["success"]
            #         logger.info(f"[GO_TO_OBJECT] 收到最终结果: success={success}")
            #         if not success:
            #             result_msg["error_msg"] = response.get("error_msg", "导航失败")
            #         # 通过 USB 发给客户端
            #         result_msg["success"] = success
            #         await self.usb_manager.send_message(result_msg)
            #         final_sent = True
            #     await asyncio.sleep(0.2)
            # return result_msg    
        except Exception as e:
            logger.error(f"[GO_TO_OBJECT] 导航到物体失败: {e}")
            result_msg = {
                "type": "go_to_object",
                "success": False,
                "obj_name": obj_name,
                "error_msg": str(e)
            }
            return result_msg
    
    async def follow_person(self, location_info: Optional[str] = None) -> Dict[str, Any]:
        """跟随人员"""
        try:
            logger.info(f"[FOLLOW_PERSON] 开始跟随人员")
            model_data = {
                "type": "follow_person",
                "user_prompt": location_info or "跟随人员"
            }
            # final_sent = False
            result_msg = {
                        "type": "follow_person",
                        "success": False,
                        "err_msg": ""}
            response = await self.send_to_local_model(model_data)            
            if response and response.get("error_msg") == "无法连接到本地模型服务器":
                result_msg["err_msg"] = "无法连接到本地模型服务器"
                return result_msg            
            result_msg["success"] = response.get("success", False)
            result_msg["err_msg"] = response.get("error_msg", "")
            return result_msg
            # while not final_sent:
            # # 发送到本地模型并获取响应
            #     if isinstance(response, dict) and "command" in response:
            #         cmd = response["command"]
            #         logger.info(f"[follow_person] 收到中间信息: {cmd}")
            #         await self.usb_manager.send_message({"type": "follow_person", "command": cmd})
            #         continue                
            #     if isinstance(response, dict) and "success" in response:
            #         success = response["success"]
            #         logger.info(f"[follow_person] 收到最终结果: success={success}")
            #         if not success:
            #             result_msg["error_msg"] = response.get("error_msg", "导航失败")
            #         # 通过 USB 发给客户端
            #         result_msg["success"] = success
            #         await self.usb_manager.send_message(result_msg)
            #         final_sent = True
            #     await asyncio.sleep(0.2)
            # return result_msg    
        except Exception as e:
            logger.error(f"[GO_TO_OBJECT] 导航到物体失败: {e}")
            result_msg = {
                "type": "follow_person",
                "success": False,
                "error_msg": str(e)
            }
            return result_msg 
    async def stop_follow(self) -> Dict[str, Any]:
        """停止跟随"""
        try:
            logger.info("[STOP_FOLLOW] 停止跟随")
            
            model_data = {
                "type": "stop_follow",
                "user_prompt": "停止跟随"
            }
            
            response = await self.send_to_local_model(model_data)
            
            if response and response.get("result"):
                return {
                    "type": "stop_follow",
                    "success": True,
                    "message": "成功停止跟随"
                }
            else:
                err = response.get("error_msg") if isinstance(response, dict) else None
                error_msg = err or "停止跟随失败"
                return {
                    "type": "stop_follow",
                    "success": False,
                    "error_msg": error_msg
                }
        except Exception as e:
            logger.error(f"[STOP_FOLLOW] 停止跟随失败: {e}")
            return {
                "type": "stop_follow",
                "success": False,
                "error_msg": str(e)
            }
    
    async def go_find_person(self, obj_name: str, user_prompt: str, **kwargs) -> Dict[str, Any]:
        """查找人员"""
        try:
            logger.info(f"[GO_FIND_PERSON] 开始查找人员: {obj_name}")
            
            model_data = {
                "type": "go_to_person",
                "user_prompt": user_prompt,
                "person_id": obj_name
            }
            # final_sent = False
            result_msg = {
                        "type": "go_find_person",
                        "success": False,
                        "err_msg": ""}
            response = await self.send_to_local_model(model_data)
            if response and response.get("error_msg") == "无法连接到本地模型服务器":
                result_msg = {
                        "type": "go_find_person",
                        "success": False,
                        "err_msg": "无法连接到本地模型服务器"}
                return result_msg
            result_msg["success"] = response.get("success", False)
            result_msg["err_msg"] = response.get("error_msg", "")
            return result_msg
            # while not final_sent:
            #     # 中间信息 command
            #     if isinstance(response, dict) and "command" in response:
            #         cmd = response["command"]
            #         logger.info(f"[GO_FIND_PERSON] 收到中间信息: {cmd}")
            #         # 立即通过 USB 发给客户端
            #         await self.usb_manager.send_message({"type": "go_find_person", "command": cmd})
            #         continue

            #     # 最终结果
            #     if isinstance(response, dict) and "success" in response:
            #         success = response["success"]
            #         if not success:
            #             result_msg["error_msg"] = response.get("error_msg", "目标人没找到")
            #         logger.info(f"[GO_FIND_PERSON] 收到最终结果: success={success}")
            #         # 通过 USB 发给客户端
            #         result_msg["success"] = success
            #         # await self.usb_manager.send_message(result_msg)
            #         final_sent = True
            #         break
            #     await asyncio.sleep(0.2)
            # return result_msg    

        except Exception as e:
            logger.error(f"[GO_FIND_PERSON] 查找人员失败: {e}")
            result_msg = {
                "type": "go_find_person",
                "success": False,
                "error_msg": str(e)
            }
            return result_msg

    async def stop_move(self) -> Dict[str, Any]:
        """
        停止机器人移动
        
        Returns:
            Dict[str, Any]: 停止移动结果
        """
        try:
            logger.info("[STOP_MOVE] 开始停止机器人移动")
            
            # 1. 检查当前是否有本地模型导航任务在执行，如果有，停止模型任务
            if self.has_active_navigation_tasks():
                logger.info(f"[STOP_MOVE] 检测到 {len(self.active_navigation_tasks)} 个活跃导航任务，发送停止命令")
                try:
                    # 发送停止命令到本地模型
                    stop_data = {
                        "type": "stop"
                    }
                    response = await self.send_to_local_model(stop_data)
                    if response and (response.get("success") == False) :
                        return {
                            "type": "stop_move",
                            "success": False,
                            "error_msg": response.get("error_msg")
                        }
                    # 清空活跃任务集合
                    async with self.task_execution_lock:
                        self.active_navigation_tasks.clear()
                    logger.info("[STOP_MOVE] 已清空活跃任务集合")
                        
                except Exception as e:
                    logger.warning(f"[STOP_MOVE] 发送停止命令到本地模型失败: {e}")
            else:
                logger.info("[STOP_MOVE] 当前没有活跃的导航任务")
            
            # 2. 在/cmd_vel话题上发一次0
            if ROS2_AVAILABLE:
                try:
                    # 使用ros2 topic publish命令发布速度为0的消息
                    cmd = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
                    logger.info(f"[STOP_MOVE] 执行命令: {cmd}")
                    result = os.system(cmd)
                    logger.info(f"[STOP_MOVE] 发布速度命令结果: {result}")
                    
                    success_msg = "已停止机器人移动"
                    logger.info(f"[STOP_MOVE] {success_msg}")
                    
                    result_data = {
                        "type": "stop_move",
                        "success": True,
                    }
                    return result_data
                    
                except Exception as e:
                    error_msg = f"发布速度命令失败: {str(e)}"
                    logger.error(f"[STOP_MOVE] {error_msg}")
                    
                    result_data = {
                        "type": "stop_move",
                        "success": False,
                        "result": error_msg
                    }
                    return result_data
            else:
                # ROS2不可用时无法停止移动
                error_msg = "ROS2不可用，无法停止机器人移动"
                logger.error(f"[STOP_MOVE] {error_msg}")
                
                result_data = {
                    "type": "stop_move",
                    "success": False,
                    "result": error_msg
                }
                return result_data
                
        except Exception as e:
            error_msg = f"停止移动失败: {str(e)}"
            logger.error(f"[STOP_MOVE] {error_msg}")
            
            result_data = {
                "type": "stop_move",
                "success": False,
                "result": error_msg
            }
            return result_data    
    
    def has_active_navigation_tasks(self) -> bool:
        """
        检查是否有正在执行的导航任务
        
        Returns:
            bool: 如果有活跃的导航任务返回True，否则返回False
        """
        return len(self.active_navigation_tasks) > 0
    
    def record_position_before_navigation(self) -> bool:
        """
        在导航任务开始前记录当前位置
        
        Returns:
            bool: 记录是否成功
        """
        try:
            logger.info(f"[AGENT] record_position_before_navigation 调用，self类型: {type(self)}")
            # 确保位置订阅已启动
            if hasattr(self, 'ros2_interface') and not self.ros2_interface.position_subscribed:
                logger.info("[AGENT] 启动位置订阅")
                self.ros2_interface.subscribe_robot_position()
            
            # 记录当前位置
            if hasattr(self, 'ros2_interface'):
                success = self.ros2_interface.record_current_position()
                if success:
                    logger.info("[AGENT] 已在导航前记录当前位置")
                else:
                    logger.warning("[AGENT] 无法记录当前位置，可能还没有位置信息")
                return success
            else:
                logger.warning("[AGENT] ROS2接口不可用")
                return False
        except Exception as e:
            logger.error(f"[AGENT] 记录位置失败: {e}")
            return False
    
    async def back_to_last_position(self) -> Dict[str, Any]:
        """
        返回到最后记录的位置
        Returns:
            Dict[str, Any]: 返回导航结果
        """
        try:
            logger.info("[AGENT] 开始返回到最后记录的位置")
            # 获取最后记录的位置
            last_position = self.ros2_interface.get_last_position()
            if not last_position:
                logger.warning("[AGENT] 没有记录的位置信息")
                return {
                    "success": False,
                    "error_msg": "没有记录的位置信息，无法返回"
                }
            logger.info(f"[AGENT] 返回到位置: {last_position['position']}")
            # 调用导航功能
            result = self.ros2_interface.navigate_to_position(last_position)
            # 确保返回结果包含type字段
            result["type"] = "back_to_last_position"
            if result["success"]:
                logger.info("[AGENT] 成功返回到最后记录的位置")
            else:
                logger.error(f"[AGENT] 返回位置失败: {result.get('error_msg', '未知错误')}")
            return result
        except Exception as e:
            logger.error(f"[AGENT] 返回最后位置时出错: {e}")
            return {
                "success": False,
                "error_msg": f"返回位置失败: {str(e)}"
            }
    
    # ======================
    # 本地模型通信
    # ======================
    
    async def connect_to_local_model(self):
        """建立与本地模型的WebSocket连接"""
        import websockets
        
        if self.local_model_connected and self.local_model_websocket:
            # 检查连接是否仍然有效
            try:
                # 发送一个ping消息来检查连接状态
                await self.local_model_websocket.ping()
                logger.info("[LOCAL_MODEL] 已连接到本地模型服务器")
                return True
            except Exception as e:
                logger.warning(f"[LOCAL_MODEL] 现有连接失效: {e}")
                # 连接失效，重置连接状态
                self.local_model_connected = False
                self.local_model_websocket = None
        
        # 尝试建立新连接
        try:
            # 使用getattr获取connect属性，避免Pylance错误
            connect_func = getattr(websockets, 'connect')
            self.local_model_websocket = await connect_func(self.local_model_uri)
            self.local_model_connected = True
            logger.info(f"[LOCAL_MODEL] 成功连接到本地模型服务器: {self.local_model_uri}")
            return True
        except Exception as e:
            logger.error(f"[LOCAL_MODEL] 连接本地模型服务器失败: {e}")
            self.local_model_connected = False
            self.local_model_websocket = None
            return False
    
    async def send_to_local_model(self, model_data: Dict[str, Any], task_id: Optional[str] = None) -> Dict[str, Any]:
        """
        通用的发送数据到本地模型的方法
        
        Args:
            model_data (Dict[str, Any]): 要发送给本地模型的数据
            task_id (str, optional): 任务ID，用于跟踪任务状态
            
        Returns:
            Dict[str, Any]: 本地模型的响应结果
        """
        async with self.local_model_lock:  # 使用锁避免并发访问
            try:
                # 检查并建立连接（带重试机制）
                connection_success = False
                connection_success = await self.connect_to_local_model()
                if not connection_success:
                    logger.error("无法连接到本地模型服务器")
                    return {"success": False, "error_msg": "无法连接到本地模型服务器"}
                
                # 发送数据
                message_str = json.dumps(model_data, ensure_ascii=False)
                if self.local_model_websocket is not None:
                    await self.local_model_websocket.send(message_str)
                logger.info(f"已发送到本地模型: {model_data}")
                intermediate_data = {
                    "type": "",
                    "command": ""
                }
                # 持续接收响应，直到收到最终结果
                final_response = None
                while self._running and self.local_model_websocket is not None:
                    try:
                        response_str = await asyncio.wait_for(self.local_model_websocket.recv(), timeout=1.0)
                        response_data = json.loads(response_str)
                        logger.info(f"[LOCAL_MODEL] 收到响应: {response_data}")
                        
                        # 检查是否是最终结果（包含success字段或result字段）
                        if ("success" in response_data or "result" in response_data) and "command" not in response_data:
                            final_response = response_data
                            break
                        else:
                            # 中间信息，需要添加任务类型后转发给所有连接的客户端
                            # intermediate_data = response_data.copy()
                            # 从原始model_data中获取任务类型
                            task_type = model_data.get("type", "unknown")
                            intermediate_data["type"] = task_type
                            if "message" in response_data:
                                intermediate_data["command"] = response_data.get("message", "") 
                            else:
                                intermediate_data["command"] = response_data.get("command", "")
                            await self.usb_manager.send_message(intermediate_data)
                            logger.info(f"[LOCAL_MODEL] 已转发中间信息给客户端: {intermediate_data}")
                    except asyncio.TimeoutError:
                        # 超时检查运行状态
                        continue
                    except Exception as e:
                        logger.error(f"接收本地模型响应时出错: {e}")
                        break
                
                return final_response if final_response else {"success": False, "error_msg": "未收到最终响应"}
                
            except Exception as e:
                logger.error(f"与本地模型通信失败: {e}")
                # 清理连接
                try:
                    if self.local_model_websocket:
                        await self.local_model_websocket.close()
                except:
                    pass
                self.local_model_connected = False
                return {"success": False, "error_msg": f"本地模型通信失败: {str(e)}"}
            finally:
                # 关闭连接
                try:
                    if self.local_model_websocket:
                        await self.local_model_websocket.close()
                        self.local_model_websocket = None
                except:
                    pass
                self.local_model_connected = False
    
    def interrupt(self) -> None:
        """中断当前任务执行"""
        self._task_interrupted = True
        logger.info("任务执行已被中断")
    
    def cleanup(self):
        """清理资源"""
        try:
            # 设置退出标志
            self._running = False
            
            # 清理USB串口资源
            if hasattr(self, 'usb_manager'):
                self.usb_manager.cleanup()
            
            # 清理ROS2资源
            if hasattr(self, 'ros2_interface'):
                self.ros2_interface.cleanup_ros2()
            
            logger.info("SmartRobotAgent资源已清理")
        except Exception as e:
            logger.error(f"清理SmartRobotAgent资源失败: {e}")

# ======================
# 主函数
# ======================

async def main():
    """主函数"""
    print("Smart Robot Agent is running...")
    print(f"USB串口通信端口: {USB_SERIAL_PORT}@{USB_SERIAL_BAUDRATE}")
    print("Type 'exit' to quit.")
    
    # 初始化数据库
    init_database()
    
    # 修复ASM JSON文件
    fix_asm_json_format()
    
    # 创建智能机器人Agent
    global smart_robot_agent_instance
    agent = SmartRobotAgent()
    smart_robot_agent_instance = agent
    # 启动电池电量监控
    try:
        agent.ros2_interface.start_battery_monitoring()
    except Exception as e:
        logger.warning(f"启动电池监控失败: {e}")    
    try:
        # 初始化agent
        await agent.initialize()
        logger.info("SmartRobotAgent启动成功")
        
        # 保持运行
        try:
            agent._running = True
            while agent._running:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            logger.info("收到键盘中断信号，正在关闭...")
    except Exception as e:
        logger.error(f"运行时出错: {e}")
    finally:
        logger.info("Smart Robot Agent 正在关闭...")
        agent.cleanup()

# 全局变量
smart_robot_agent_instance = None

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"程序运行出错: {e}")







