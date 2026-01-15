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
GetRobotRise = None
RobotTilt = None
RobotTiltState = None
ScreenTilt = None
ScreenTiltState = None
RgbBrightnessColorSet = None
RgbState = None
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
            ScreenTilt, ScreenTiltState,
            RgbLightStrip, RgbLightStripState,
            LaserPointer, LaserPointerState
        )
        jqr_ros_msgs = True
        # logger.info("jqr_ros_msgs 导入成功")
    except ImportError as e:
        logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] jqr_ros_msgs 导入失败 (ImportError): {e}")
        logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 请检查ROS2工作空间是否正确配置和source")
        jqr_ros_msgs = False
    except Exception as e:
        logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] jqr_ros_msgs 导入失败 (未知错误): {e}")
        jqr_ros_msgs = False
    ROS2_AVAILABLE = True
    # logger.info("ROS2 rclpy 导入成功")
except ImportError as e:
    logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] ROS2 rclpy 不可用: {e}")
    geometry_msgs = None
    jqr_ros_msgs = False

# 尝试导入cv2，如果不存在则忽略
try:
    import cv2
except ImportError:
    cv2 = None
    logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] cv2模块未安装,视频处理功能将不可用")

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
USB_SERIAL_PORT = "/dev/rk"
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
        logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] ASM JSON文件不存在: {ASM_JSON_PATH}")
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
        # logger.info("数据库初始化成功")
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
# ReAct 框架核心组件
# ======================

# ======================
# ReAct框架核心类
# ======================

class Thought:
    """Agent思考过程"""
    def __init__(self, content: str, reasoning_type: str = "analysis"):
        self.content = content
        self.reasoning_type = reasoning_type  # analysis, planning, reflection
        self.timestamp = datetime.now()
        
    def __repr__(self):
        return f"Thought({self.reasoning_type}: {self.content[:50]}...)"

class AgentThought(Thought):
    """ReAct框架中的思考结果，包含推理和下一步行动"""
    def __init__(self, content: str, reasoning_type: str = "analysis"):
        super().__init__(content, reasoning_type)
        self.is_final_answer = False  # 是否是最终答案
        self.final_answer = ""  # 最终答案内容
        self.action_name: Optional[str] = None  # 下一步动作名称
        self.action_args: Dict[str, Any] = {}  # 下一步动作参数
        
    def __repr__(self):
        if self.is_final_answer:
            return f"AgentThought(FINAL: {self.final_answer[:50]}...)"
        elif self.action_name:
            return f"AgentThought(ACTION: {self.action_name} {self.action_args})"
        else:
            return f"AgentThought({self.reasoning_type}: {self.content[:50]}...)"

class Observation:
    """Agent观察结果"""
    def __init__(self, content: str, source: str, success: bool = True):
        self.content = content
        self.source = source  # environment, tool, internal
        self.success = success
        self.timestamp = datetime.now()
        
    def __repr__(self):
        return f"Observation({self.source}, success={self.success}: {self.content[:50]}...)"

class AgentMemory:
    """Agent记忆系统 - 存储思考、行动、观察历史"""
    
    def __init__(self, max_history: int = 50):
        self.max_history = max_history
        self.thoughts: List[Thought] = []
        self.actions: List[Dict[str, Any]] = []
        self.observations: List[Observation] = []
        self.task_history: List[Dict[str, Any]] = []
        
    def add_thought(self, thought: Thought):
        """添加思考记录"""
        self.thoughts.append(thought)
        if len(self.thoughts) > self.max_history:
            self.thoughts.pop(0)
            
    def add_action(self, action_name: str, args: Dict[str, Any]):
        """添加行动记录"""
        self.actions.append({"name": action_name, "args": args, "timestamp": datetime.now()})
        if len(self.actions) > self.max_history:
            self.actions.pop(0)
            
    def add_observation(self, observation: Observation):
        """添加观察记录"""
        self.observations.append(observation)
        if len(self.observations) > self.max_history:
            self.observations.pop(0)
            
    def add_task(self, task: Dict[str, Any], result: Dict[str, Any]):
        """添加任务记录"""
        self.task_history.append({
            "task": task,
            "result": result,
            "timestamp": datetime.now().isoformat()
        })
        if len(self.task_history) > self.max_history:
            self.task_history.pop(0)
            
    def get_recent_context(self, n: int = 3) -> str:
        """获取最近的上下文，用于推理"""
        context_lines = []
        
        # 最近的思考
        for t in self.thoughts[-n:]:
            context_lines.append(f"[THOUGHT-{t.reasoning_type}] {t.content}")
            
        # 最近的行动
        for a in self.actions[-n:]:
            context_lines.append(f"[ACTION] {a['name']} with {a['args']}")
            
        # 最近的观察
        for o in self.observations[-n:]:
            status = "✓" if o.success else "✗"
            context_lines.append(f"[OBSERVATION {status}] {o.content}")
            
        return "\n".join(context_lines) if context_lines else "（暂无历史记录）"
        
    def get_last_observation(self) -> Optional[Observation]:
        """获取最后一次观察"""
        return self.observations[-1] if self.observations else None
        
    def clear_episode(self):
        """清空当前episode的记忆"""
        self.thoughts = []
        self.actions = []
        self.observations = []

# ======================
# ROS2接口
# ======================

class ROS2Interface:
    """ROS2接口类，用于与ROS2系统进行交互"""

    def __init__(self):
        """初始化ROS2接口"""
        self.battery_level = 100.0  # 初始电池电量
        self.last_position = None  # 记录最后一个位置
        self.pre_position = None  # 任务执行前的位置
        self.initial_position = None  # 初始位置（只在第一次位置回调时记录）
        self.position_subscribed = False  # 是否已订阅位置信息
        self.battery_subscribed = False  # 是否已订阅电池电量信息
        self.battery_subscription = None  # 电池电量订阅对象
        self.position_subscription = None  # 位置订阅对象
        self.node = None  # ROS2节点
        self.initialized = False  # ROS2是否已初始化
        self.ros2_thread = None  # ROS2处理线程
        self.ros2_thread_running = False  # ROS2线程是否运行

        # 并发服务调用支持
        self.service_clients = {}  # 服务客户端缓存 {service_name: (client, callback_group)}
        self.mutually_exclusive_callback_group = None  # 互斥回调组
        self.reentrant_callback_group = None  # 可重入回调组

        # 异步服务调用结果存储
        self.service_call_results = {}  # {call_id: (future, event, result)}
        self.call_id_counter = 0  # 服务调用ID计数器
        self.service_call_lock = threading.Lock()  # 服务调用锁

        # 如果ROS2可用，初始化rclpy
        if ROS2_AVAILABLE:
            self._initialize_ros2()
    def set_laser_pointer(self, *args, **kwargs) -> Dict[str, Any]:
        """控制激光笔开关/查询状态 (jqr_ros_msgs版本，支持并发)

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

            # 使用异步服务调用（支持并发）
            result = self._call_ros2_service_async(
                "/set_laser_pointer",
                "jqr_ros_msgs/srv/LaserPointer",
                {"laser_pointer": laser_pointer_value},
                timeout=10.0
            )

            if not result.get("success"):
                error_msg = result.get("error_msg", "未知错误")
                return {
                    "success": False,
                    "description": error_msg
                }

            # 解析响应数据
            response_dict = result.get("response", {})
            result_number = response_dict.get("result_number", 0)
            result_msg = response_dict.get("result_msg", "")
            success = (result_number in [1, 2, 3])

            if success:
                logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 激光笔控制成功: {result_msg}")
                return {
                    "success": True,
                    "description": result_msg,
                    "result_number": result_number
                }
            else:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 激光笔控制失败: {result_msg}")
                return {
                    "success": False,
                    "description": result_msg,
                    "result_number": result_number
                }

        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置激光笔失败: {e}")
            return {
                "success": False,
                "description": f"设置激光笔失败: {str(e)}"
            }
    
    def get_laser_pointer_state(self) -> Dict[str, Any]:
        """获取激光笔状态（支持并发）

        Returns:
            Dict[str, Any]: 激光笔状态信息
        """
        try:
            # 使用异步服务调用（支持并发）
            result = self._call_ros2_service_async(
                "/get_laser_pointer_state",
                "jqr_ros_msgs/srv/LaserPointerState",
                {},
                timeout=10.0
            )

            if not result.get("success"):
                error_msg = result.get("error_msg", "未知错误")
                return {
                    "success": False,
                    "description": error_msg
                }

            # 解析响应数据
            response_dict = result.get("response", {})
            laser_pointer_state = response_dict.get("laser_pointer_state", False)
            result_number = response_dict.get("result_number", 0)
            result_msg = response_dict.get("result_msg", "")
            success = (result_number == 1)

            if success:
                logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取激光笔状态成功: state={laser_pointer_state}")
                return {
                    "success": True,
                    "laser_pointer_state": laser_pointer_state,
                    "description": result_msg,
                    "result_number": result_number
                }
            else:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取激光笔状态失败: {result_msg}")
                return {
                    "success": False,
                    "laser_pointer_state": laser_pointer_state,
                    "description": f"获取失败: {result_msg}",
                    "result_number": result_number
                }

        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取激光笔状态失败: {e}")
            return {
                "success": False,
                "description": f"获取激光笔状态失败: {str(e)}"
            }

    def start_battery_monitoring(self) -> bool:
        """开始电池电量监控（订阅模式）"""
        global battery_level

        if hasattr(self, 'battery_subscribed') and self.battery_subscribed:
            # logger.warning("电池电量监控已在运行")
            return True

        try:
            # 检查ROS2和节点是否可用
            if not ROS2_AVAILABLE:
                logger.warning("ROS2不可用，无法启动电池电量监控")
                return False

            if not self.initialized or not self.node:
                logger.warning("ROS2未初始化或节点不存在，无法启动电池电量监控")
                return False

            # 检查jqr_ros_msgs是否可用
            if not jqr_ros_msgs:
                logger.warning("jqr_ros_msgs模块不可用，无法创建电池电量订阅者")
                return False

            # 使用主节点创建电池电量订阅
            self.battery_subscription = self.node.create_subscription(
                BatteryLevel,
                '/battery_level',  # 电池电量话题
                battery_callback,
                10  # 队列大小
            )
            self.battery_subscribed = True
            return True

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
                logger.warning("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] rclpy模块不可用，跳过初始化")
                self.initialized = False
                return False

            # # 检查是否已经初始化
            # try:
            #     # 尝试获取rclpy状态来判断是否已初始化
            #     # Pylance 可能不认识 get_instance，但在某些rclpy版本中存在
            #     if hasattr(rclpy, 'get_instance'):
            #         instance = rclpy.get_instance()  # type: ignore
            #         if instance is not None:
            #             # logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] rclpy已经初始化")
            #             # 如果rclpy已初始化但没有节点，创建节点
            #             if self.node is None:
            #                 self.node = rclpy.create_node('smart_robot_agent_ros2')
            #                 # logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 节点创建成功: smart_robot_agent_ros2")
            #             # 创建回调组
            #             self._create_callback_groups()
            #             self.initialized = True
            #             # 启动ROS2处理线程
            #             self._start_ros2_spin_thread()
            #             return True
            #     else:
            #         # 备用检查方法
            #         logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 检查rclpy初始化状态")
            # except Exception as check_error:
            #     logger.debug(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 检查初始化状态时出错: {check_error}")
            #     # 未初始化，进行初始化
            #     pass

            # 初始化rclpy
            rclpy.init()
            # logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] rclpy初始化成功")

            # 创建节点
            self.node = rclpy.create_node('smart_robot_agent_ros2')
            # logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 节点创建成功: smart_robot_agent_ros2")

            # 创建回调组
            self._create_callback_groups()

            self.initialized = True
            # 启动ROS2处理线程
            self._start_ros2_spin_thread()
            return True

        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 初始化失败: {e}")
            self.initialized = False
            return False
    
    def _start_ros2_spin_thread(self):
        """启动ROS2独立处理线程"""
        if self.ros2_thread is not None and self.ros2_thread.is_alive():
            logger.warning("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] ROS2处理线程已在运行")
            return
            
        self.ros2_thread_running = True
        self.ros2_thread = threading.Thread(target=self._ros2_spin_worker, daemon=True)
        self.ros2_thread.start()
        # logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 独立处理线程已启动")
    
    def _create_callback_groups(self):
        """创建回调组以支持并发服务调用"""
        try:
            from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

            # 创建互斥回调组（串行执行，用于需要互斥的操作）
            self.mutually_exclusive_callback_group = MutuallyExclusiveCallbackGroup()
            # logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 互斥回调组创建成功")

            # 创建可重入回调组（并发执行，用于支持并发的服务调用）
            self.reentrant_callback_group = ReentrantCallbackGroup()
            # logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 可重入回调组创建成功")

        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 创建回调组失败: {e}")
            self.mutually_exclusive_callback_group = None
            self.reentrant_callback_group = None

    def _ros2_spin_worker(self):
        """ROS2独立处理线程工作函数"""
        global rclpy
        try:
            # logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 处理线程开始运行")
            while self.ros2_thread_running and rclpy and rclpy.ok() and self.node:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                # 短暂休眠避免CPU占用过高
                time.sleep(0.01)
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 处理线程出错: {e}")
        finally:
            logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 处理线程已退出")
    
    def stop_ros2_spin_thread(self):
        """停止ROS2处理线程"""
        try:
            self.ros2_thread_running = False
            if self.ros2_thread and self.ros2_thread.is_alive():
                self.ros2_thread.join(timeout=2.0)
            logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 处理线程已停止")
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 停止处理线程失败: {e}")

    def cleanup_ros2(self):
        """清理ROS2资源"""
        global rclpy
        try:
            # 停止处理线程
            self.stop_ros2_spin_thread()

            # 清理服务客户端缓存
            self.service_clients.clear()
            self.service_call_results.clear()

            if self.initialized and rclpy:
                if self.node:
                    self.node.destroy_node()
                    self.node = None
                rclpy.shutdown()
                self.initialized = False
                logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 资源已清理")
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
                    # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 服务 {service_name} 存在")
                    return True
            
            logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 服务 {service_name} 不存在")
            return False
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 检查服务存在性失败: {e}")
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
                    # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 动作 {action_name} 存在")
                    return True

            logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 动作 {action_name} 不存在")
            return False
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 检查动作存在性失败: {e}")
            return False

    def _get_or_create_service_client(self, service_name: str, service_type: str, use_concurrent: bool = True):
        """获取或创建服务客户端

        Args:
            service_name (str): 服务名称
            service_type (str): 服务类型字符串 (如 "jqr_ros_msgs/srv/RobotRise")
            use_concurrent (bool): 是否使用并发回调组

        Returns:
            服务客户端对象，如果创建失败则返回None
        """
        try:
            # 检查缓存中是否已有该客户端
            if service_name in self.service_clients:
                return self.service_clients[service_name][0]

            # 解析服务类型字符串
            # 格式: "package_name/srv/ServiceName"
            parts = service_type.split('/')
            if len(parts) != 3:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 无效的服务类型格式: {service_type}")
                return None

            package_name = parts[0]
            srv_name = parts[2]

            # 动态导入服务类型
            try:
                module = __import__(f'{package_name}.srv', fromlist=[srv_name])
                srv_class = getattr(module, srv_name)
            except (ImportError, AttributeError) as e:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 无法导入服务类型 {service_type}: {e}")
                return None

            # 选择回调组
            callback_group = self.reentrant_callback_group if use_concurrent else self.mutually_exclusive_callback_group

            # 创建服务客户端（如果node为None则返回None）
            if self.node is None:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 节点未初始化，无法创建服务客户端")
                return None

            # 创建服务客户端
            # Pylance 类型检查可能有误，callback_group 类型是兼容的
            client = self.node.create_client(
                srv_class,
                service_name,
                callback_group=callback_group  # type: ignore
            )

            # 缓存客户端
            self.service_clients[service_name] = (client, callback_group)
            # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 服务客户端已创建: {service_name}")

            return client

        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 创建服务客户端失败: {e}")
            return None

    def _call_ros2_service_async(self, service_name: str, service_type: str, request_data: dict, timeout: float = 10.0) -> Dict[str, Any]:
        """异步调用ROS2服务（支持并发）

        Args:
            service_name (str): 服务名称
            service_type (str): 服务类型
            request_data (dict): 请求数据（字典格式）
            timeout (float): 超时时间（秒）

        Returns:
            Dict[str, Any]: 服务响应结果
        """
        # 首先检查服务是否存在
        if not self._check_ros2_service_exists(service_name):
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 服务 {service_name} 不存在，无法调用")
            return {
                "success": False,
                "error_msg": f"服务 {service_name} 不存在或调用失败"
            }

        try:
            # 获取或创建服务客户端（使用可重入回调组支持并发）
            client = self._get_or_create_service_client(service_name, service_type, use_concurrent=True)
            if not client:
                return {
                    "success": False,
                    "error_msg": f"无法创建服务客户端: {service_name}"
                }

            # 等待服务可用
            if not client.wait_for_service(timeout_sec=timeout):
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 服务 {service_name} 未在 {timeout} 秒内变为可用")
                return {
                    "success": False,
                    "error_msg": f"服务 {service_name} 未在 {timeout} 秒内变为可用"
                }

            # 创建请求对象
            # Pylance 类型检查可能有误，srv_type.Request 在运行时存在
            if hasattr(client, 'srv_type'):
                request_type = client.srv_type.Request  # type: ignore
                if request_type is None:
                    logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 无法获取服务类型")
                    return {
                        "success": False,
                        "error_msg": "无法获取服务类型"
                    }
                request = request_type()  # type: ignore
            else:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 客户端没有 srv_type 属性")
                return {
                    "success": False,
                    "error_msg": "客户端没有 srv_type 属性"
                }
            for key, value in request_data.items():
                if hasattr(request, key):
                    setattr(request, key, value)
                else:
                    logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 请求类型没有属性: {key}")

            # 同步调用服务（由于回调组是可重入的，多个服务调用可以并发执行）
            # 注意：这里使用同步调用但配合可重入回调组，ROS2会在后台处理多个服务请求
            future = client.call_async(request)

            # 等待结果
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout:
                    # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 服务调用超时: {service_name}")
                    return {
                        "success": False,
                        "error_msg": f"服务调用超时: {service_name}"
                    }
                time.sleep(0.01)

            response = future.result()

            # 将响应转换为字典
            response_dict = {}
            if hasattr(response, 'get_fields_and_field_types'):
                # Pylance 可能无法识别 get_fields_and_field_types，运行时它是正确的
                for field_name in response.get_fields_and_field_types():  # type: ignore
                    value = getattr(response, field_name)
                    # 处理std_msgs类型
                    if hasattr(value, 'data'):
                        response_dict[field_name] = value.data
                    else:
                        response_dict[field_name] = value
            else:
                # 如果无法获取字段，尝试直接转换为字典
                response_dict = vars(response) if hasattr(response, '__dict__') else {}

            # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 异步服务调用成功: {service_name}, 响应: {response_dict}")
            return {
                "success": True,
                "response": response_dict
            }

        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 异步服务调用失败: {e}")
            return {
                "success": False,
                "error_msg": f"服务调用失败: {str(e)}"
            }
    
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
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 服务 {service_name} 不存在，无法调用")
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
            # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 执行命令: {cmd}")
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 执行命令: {cmd}")
            
            
            # 使用subprocess而不是os.popen来获得更好的控制
            # 设置ROS环境变量
            import os
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = os.environ.get('ROS_DOMAIN_ID', '0')
            
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10, env=env)
            
            if result.returncode != 0:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 服务调用命令执行失败，返回码: {result.returncode}")
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] stderr: {result.stderr}")
                return None
            
            response = result.stdout.strip()
            # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 服务调用结果: {response}")            
            
            # 检查结果是否为空
            if not response:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 服务 {service_name} 返回空响应")
                return None
                
            return response
        except subprocess.TimeoutExpired:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 服务调用超时: {service_name}")
            return None
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 服务调用失败: {e}")
            return None    
    
    def subscribe_robot_position(self) -> bool:
        """订阅机器人位置信息
        
        Returns:
            bool: 订阅是否成功
        """
        try:
            if not ROS2_AVAILABLE or not self.initialized or not self.node:
                logger.warning("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] ROS2不可用或未初始化，无法订阅位置信息")
                return False
            
            if not geometry_msgs:
                logger.error("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] geometry_msgs不可用，无法订阅位置话题")
                return False
                
            # 使用主节点创建位置订阅，订阅的回调由主spin循环处理
            self.position_subscription = self.node.create_subscription(
                geometry_msgs.PoseStamped,
                '/tracked_pose',  # 假设SLAM发布的话题名为 /tracked_pose
                self._position_callback,
                10
            )
            
            self.position_subscribed = True
            # logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 已使用主节点订阅机器人位置话题: /tracked_pose")
            return True
            
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 订阅位置信息失败: {e}")
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
            
            # 记录初始位置（只在第一次回调时记录）
            if self.initial_position is None:
                self.initial_position = position
                logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 已记录初始位置: ({position['position']['x']:.2f}, {position['position']['y']:.2f})")
            # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 收到位置更新: ({position['position']['x']:.2f}, {position['position']['y']:.2f})")
            
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
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 位置回调处理失败: {e}")
    
    def record_current_position(self) -> bool:
        """记录当前位置
        
        Returns:
            bool: 记录是否成功
        """
        if self.last_position:
            self.pre_position = self.last_position
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 已记录当前位置: {self.pre_position['position']}")
            return True
        else:
            logger.warning("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 没有可用的位置信息")
            return False
    
    def get_last_position(self) -> Optional[Dict[str, Any]]:
        """获取最后记录的位置
        
        Returns:
            Optional[Dict[str, Any]]: 位置信息，如果没有则返回None
        """
        return self.pre_position
    
    def get_initial_position(self) -> Optional[Dict[str, Any]]:
        """获取初始位置
        
        Returns:
            Optional[Dict[str, Any]]: 初始位置信息，如果没有则返回None
        """
        return self.initial_position
    
    def navigate_to_position(self, position: Dict[str, Any]) -> Dict[str, Any]:
        """导航到指定位置

        注意：/navigate_to_pose 是一个action server，不是service

        Args:
            position (Dict[str, Any]): 目标位置信息，格式包含 position 和 orientation

        Returns:
            Dict[str, Any]: 导航结果
        """
        try:
            if not position or 'position' not in position:
                return {
                    "success": False,
                    "error_msg": "无效的位置信息"
                }

            # 检查导航action是否可用
            if not self._check_ros2_action_exists("/navigate_to_pose"):
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 导航action /navigate_to_pose 不可用")
                return {
                    "success": False,
                    "error_msg": "导航action /navigate_to_pose 不可用"
                }

            # 调用导航action
            # 构造NavigateToPose goal，完全复用position的数据结构
            goal_data = {
                "pose": {
                    "header": {
                        "stamp": {"sec": 0, "nanosec": 0},
                        "frame_id": "map"
                    },
                    "pose": {
                        "position": {
                            "x": position['position'].get('x', 0.0),
                            "y": position['position'].get('y', 0.0),
                            "z": position['position'].get('z', 0.0)
                        },
                        "orientation": {
                            "x": position.get('orientation', {}).get('x', 0.0),
                            "y": position.get('orientation', {}).get('y', 0.0),
                            "z": position.get('orientation', {}).get('z', 0.0),
                            "w": position.get('orientation', {}).get('w', 1.0)
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
                # 解析action响应结果
                # 检查是否包含成功状态
                if "SUCCEEDED" in response:
                    return {
                        "success": True,
                        "error_msg": ""
                    }
                else:
                    # 提取错误信息
                    error_msg = "导航失败"
                    if "ABORTED" in response:
                        error_msg = "导航被中止"
                    elif "CANCELED" in response:
                        error_msg = "导航被取消"
                    elif "REJECTED" in response:
                        error_msg = "导航目标被拒绝"
                    
                    # 尝试从响应中提取更多细节
                    import re
                    result_match = re.search(r'Result:\s*(\w+)', response)
                    if result_match:
                        error_msg = f"导航失败: {result_match.group(1)}"
                    
                    return {
                        "success": False,
                        "error_msg": error_msg
                    }
            else:
                return {
                    "success": False,
                    "error_msg": "导航action调用失败"
                }
                
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 导航到位置失败: {e}")
            return {
                "success": False,
                "error_msg": f"导航失败: {str(e)}"
            }
        
    # ======================
    # 机器人运动控制相关接口
    # ======================
    
    def get_move_mode(self) -> Dict[str, Any]:
        """获取运动模式（支持并发）

        Returns:
            Dict[str, Any]: 运动模式信息
        """
        try:
            # 使用异步服务调用（支持并发）
            result = self._call_ros2_service_async(
                "/get_move_mode",
                "jqr_ros_msgs/srv/MoveMode",
                {},
                timeout=10.0
            )

            if not result.get("success"):
                error_msg = result.get("error_msg", "未知错误")
                return {
                    "success": False,
                    "move_mode": -1,
                    "linear_vel": 0.0,
                    "description": error_msg
                }

            # 解析响应数据
            response_dict = result.get("response", {})
            move_mode = response_dict.get("move_mode", -1)
            linear_vel = response_dict.get("linear_vel", 0.0)
            result_number = response_dict.get("result_number", 1)
            result_msg = response_dict.get("result_msg", "")

            success = (result_number == 1)

            if success:
                logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取运动模式成功: mode={move_mode}, vel={linear_vel}")
                return {
                    "success": True,
                    "move_mode": move_mode,
                    "linear_vel": linear_vel,
                    "description": result_msg,
                    "result_number": result_number
                }
            else:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取运动模式失败: {result_msg}")
                return {
                    "success": False,
                    "move_mode": move_mode,
                    "linear_vel": linear_vel,
                    "description": f"获取失败: {result_msg}",
                    "result_number": result_number
                }

        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取运动模式失败: {e}")
            return {
                "success": False,
                "move_mode": -1,
                "linear_vel": 0.0,
                "description": f"获取运动模式失败: {str(e)}"
            }
    def set_robot_rise_jqr(self, rise: bool, duration: int = 0) -> Dict[str, Any]:
        """控制机器人升降 (jqr_ros_msgs版本，支持并发)

        Args:
            rise (bool): 升降状态 (True: 上升, False: 下降)
            duration (int): 执行时间（单位0.1s），缺省表示希望以最快的速度执行

        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            # 构造请求数据
            request_data = {"robot_rise": rise}
            if duration > 0:
                # Pylance 可能误报类型错误，duration 确实是 int 类型
                request_data["duration"] = duration  # type: ignore

            # 使用异步服务调用（支持并发）
            result = self._call_ros2_service_async(
                "/set_robot_rise",
                "jqr_ros_msgs/srv/RobotRise",
                request_data,
                timeout=10.0
            )

            if not result.get("success"):
                err_msg = result.get("error_msg", "未知错误")
                return {
                    "success": False,
                    "err_msg": err_msg
                }

            # 解析响应数据
            response_dict = result.get("response", {})
            result_number = response_dict.get("result_number", 0)
            result_msg = response_dict.get("result_msg", "")

            success = (result_number == 1)

            if success:
                # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置机器人升降成功: 上升={rise}, duration={duration}")
                return {"success": True, "err_msg": ""}
            else:
                # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置机器人升降失败: {result_msg}")
                return {
                    "success": False,
                    "err_msg": result_msg
                }

        except Exception as e:
            # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置机器人升降失败: {e}")
            return {
                "success": False,
                "err_msg": f"设置机器人{'上升' if rise else '下降'}失败: {str(e)}"
            }
        
    def get_robot_rise_state(self) -> Dict[str, Any]:
        """获取机器人升降状态（支持并发）

        Returns:
            Dict[str, Any]: 升降状态信息
        """
        try:
            # 使用异步服务调用（支持并发）
            result = self._call_ros2_service_async(
                "/get_robot_rise",
                "jqr_ros_msgs/srv/RobotRiseState",
                {},
                timeout=10.0
            )

            if not result.get("success"):
                error_msg = result.get("error_msg", "未知错误")
                return {
                    "success": False,
                    "state": False,
                    "description": error_msg
                }

            # 解析响应数据
            response_dict = result.get("response", {})
            robot_rise_state = response_dict.get("robot_rise_state", False)
            result_number = response_dict.get("result_number", 0)
            result_msg = response_dict.get("result_msg", "")

            success = (result_number == 1)

            if success:
                # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取机器人升降状态成功: state={robot_rise_state}")
                return {
                    "success": True,
                    "state": robot_rise_state,
                    "description": result_msg,
                    "result_number": result_number
                }
            else:
                # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取机器人升降状态失败: {result_msg}")
                return {
                    "success": False,
                    "state": robot_rise_state,
                    "description": f"获取失败: {result_msg}",
                    "result_number": result_number
                }

        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取机器人升降状态失败: {e}")
            return {
                "success": False,
                "state": False,
                "description": f"获取机器人升降状态失败: {str(e)}"
            }
        
    # ======================
    # 机器人俯仰控制相关接口
    # ======================
    
    def set_robot_tilt_jqr(self, angle: float, duration: int = 0) -> Dict[str, Any]:
        """控制机器人俯仰 (jqr_ros_msgs版本，支持并发)

        Args:
            angle (float): 俯仰角度
            duration (int): 执行时间（单位0.1s），缺省表示希望以最快的速度执行

        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            # 构造请求数据
            request_data = {"robot_tilt": angle}
            if duration > 0:
                request_data["duration"] = duration

            # 使用异步服务调用（支持并发）
            result = self._call_ros2_service_async(
                "/set_robot_tilt",
                "jqr_ros_msgs/srv/RobotTilt",
                request_data,
                timeout=10.0
            )

            if not result.get("success"):
                error_msg = result.get("error_msg", "未知错误")
                return {
                    "success": False,
                    "angle": angle,
                    "description": error_msg
                }

            # 解析响应数据
            response_dict = result.get("response", {})
            result_number = response_dict.get("result_number", 0)
            result_msg = response_dict.get("result_msg", "")

            success = (result_number == 1)

            if success:
                # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置机器人俯仰角度成功: angle={angle}")
                return {
                    "success": True,
                    "angle": angle,
                    "description": result_msg,
                    "result_number": result_number
                }
            else:
                # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置机器人俯仰角度失败: {result_msg}")
                return {
                    "success": False,
                    "angle": angle,
                    "description": f"设置失败: {result_msg}",
                    "result_number": result_number
                }

        except Exception as e:
            # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置机器人俯仰角度失败: {e}")
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
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 动作 {action_name} 不存在，无法调用")
            return None
            
        try:
            # 构造ROS2动作调用命令
            cmd = f"ros2 action send_goal {action_name} {action_type} '{goal_data}'"
            # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 执行命令: {cmd}")
            
            # 使用subprocess而不是os.popen来获得更好的控制
            # 设置ROS环境变量
            import os
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = os.environ.get('ROS_DOMAIN_ID', '0')
            
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=30, env=env)
            
            if result.returncode != 0:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 动作调用命令执行失败，返回码: {result.returncode}")
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] stderr: {result.stderr}")
                return None
            
            response = result.stdout.strip()
            # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 动作调用结果: {response}")
            
            # 检查结果是否为空
            if not response:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 动作 {action_name} 返回空响应")
                return None
                
            return response
        except subprocess.TimeoutExpired:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 动作调用超时: {action_name}")
            return None
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 动作调用失败: {e}")
            return None
            
    # ======================
    # 药箱控制相关接口
    # ======================
    
    def set_medicine_box_switch(self, switch: bool, speed_stage: int) -> Dict[str, Any]:
        """控制药箱开关（支持并发）

        Args:
            switch (bool): 药箱开关状态 (True: 打开, False: 关闭)
            speed_stage (int): 速度档位 (1: 慢档, 2: 快档)

        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            # 使用异步服务调用（支持并发）
            result = self._call_ros2_service_async(
                "/set_medicine_box_switch",
                "jqr_ros_msgs/srv/MedicineBoxSwitch",
                {
                    "medicine_box_switch": switch,
                    "speed_stage": speed_stage
                },
                timeout=10.0
            )

            if not result.get("success"):
                error_msg = result.get("error_msg", "未知错误")
                return {
                    "type": "set_medicine_box_switch",
                    "success": False,
                    "error_msg": error_msg
                }

            # 解析响应数据
            response_dict = result.get("response", {})
            result_number = response_dict.get("result_number", 0)
            result_msg = response_dict.get("result_msg", "")

            success = (result_number == 1)

            if success:
                # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置药箱开关成功: switch={switch}, speed={speed_stage}")
                return {
                    "type": "set_medicine_box_switch",
                    "success": True
                }
            else:
                # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置药箱开关失败: {result_msg}")
                return {
                    "type": "set_medicine_box_switch",
                    "success": False,
                    "error_msg": result_msg or "服务异常"
                }

        except Exception as e:
            # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置药箱开关失败: {e}")
            return {
                "type": "set_medicine_box_switch",
                "success": False,
                "error_msg": f"设置药箱{'打开' if switch else '关闭'}失败: {str(e)}"
            }
    
    def get_medicine_box_state(self) -> Dict[str, Any]:
        """获取药箱状态（支持并发）

        Returns:
            Dict[str, Any]: 药箱状态信息
        """
        try:
            # 使用异步服务调用（支持并发）
            result = self._call_ros2_service_async(
                "/get_medicine_box_state",
                "jqr_ros_msgs/srv/MedicineBoxState",
                {},
                timeout=10.0
            )

            if not result.get("success"):
                error_msg = result.get("error_msg", "未知错误")
                return {
                    "success": False,
                    "state": False,
                    "description": error_msg
                }

            # 解析响应数据
            response_dict = result.get("response", {})
            medicine_box_state = response_dict.get("medicine_box_switch_state", False)
            result_number = response_dict.get("result_number", 0)
            result_msg = response_dict.get("result_msg", "")

            success = (result_number == 1)

            if success:
                # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取药箱状态成功: state={medicine_box_state}")
                return {
                    "success": True,
                    "state": medicine_box_state,
                    "description": result_msg,
                    "result_number": result_number
                }
            else:
                # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取药箱状态失败: {result_msg}")
                return {
                    "success": False,
                    "state": medicine_box_state,
                    "description": f"获取失败: {result_msg}",
                    "result_number": result_number
                }

        except Exception as e:
            # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取药箱状态失败: {e}")
            return {
                "success": False,
                "state": False,
                "description": f"获取药箱状态失败: {str(e)}"
            }

    def set_rgb_light_strip(self, brightness_set: Optional[int] = None, rgb_switch: Optional[bool] = None, color: Optional[str] = None, is_incremental: bool = False) -> Dict[str, Any]:
        """控制RGB灯带开关、颜色与亮度
        
        Args:
            rgb_switch (bool): RGB灯开关 (True: 开启, False: 关闭)，可选
            brightness (int): 亮度 0-255，可选
            color (str): 颜色名称 (red/yellow/blue/green等)，可选
            red (int): 红色分量 0~255，已弃用，请使用color参数
            green (int): 绿色分量 0~255，已弃用，请使用color参数
            blue (int): 蓝色分量 0~255，已弃用，请使用color参数
            is_incremental (bool): 是否增量调节 (True: 增量式, False: 非增量式)，默认False
            
        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            # 构造请求数据
            request_parts = []
                        
            # 添加必填参数
            if rgb_switch is not None:
                request_parts.append(f'"rgb_switch": {str(rgb_switch).lower()}')
            
            # 添加可选参数
            if is_incremental is not None:
                request_parts.append(f'"is_incremental": {str(is_incremental).lower()}')
            
            if brightness_set is not None:
                request_parts.append(f'"brightness_set": {brightness_set}')
            
            if color is not None:
                request_parts.append(f'"color": "{color}"')
            
            # 如果没有任何参数，返回错误
            if not request_parts:
                return {
                    "success": False,
                    "description": "请至少提供一个参数：rgb_switch, brightness, color"
                }
            
            request_data = '{' + ', '.join(request_parts) + '}'
            
            response = self._call_ros2_service(
                "/rgb_brightness_color_set",
                "jqr_ros_msgs/srv/RgbLightStrip",
                request_data
            )
            if response is None:
                result = {
                    "success": False,
                    "description": "服务 /rgb_brightness_color_set 不存在或调用失败"
                }
                # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置RGB灯带失败: {result}")
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
                    # if success:
                    #     logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置RGB灯带成功: {result}")
                    # else:
                    #     logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置RGB灯带失败: {result}")
                    return result
                except Exception as e:
                    logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] RGB灯带响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置RGB灯带失败: {e}")
            return {
                "success": False,
                "description": f"设置RGB灯带失败: {str(e)}"
            }

    def get_rgb_light_strip_state(self) -> Dict[str, Any]:
        """获取RGB灯带状态
        
        根据新的服务接口 rgb_state 获取RGB灯的开关、颜色和亮度状态
        
        Returns:
            Dict[str, Any]: 灯带状态信息
            - rgb_switch (bool): RGB灯开关状态
            - brightness_value (int): 亮度值 0-255
            - color (str): 颜色名称 (red/yellow/blue/green等)
            - success (bool): 是否获取成功
            - description (str): 描述信息
            - result_number (int): 结果码 (0=失败, 1=成功)
        """
        try:
            response = self._call_ros2_service(
                "/rgb_state",
                "jqr_ros_msgs/srv/RgbLightStripState",
                "{}"
            )
            if response is None:
                result = {
                    "success": False,
                    "description": "服务 /rgb_state 不存在或调用失败"
                }
                # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取RGB灯带状态失败: {result}")
                return result
            else:
                if not response or not response.strip():
                    return {
                        "success": False,
                        "description": "RGB灯带状态服务返回空响应"
                    }
                try:
                    response_data = parse_ros2_response(response)
                    rgb_switch = response_data.get("rgb_switch", False)
                    brightness_value = response_data.get("brightness_value", 0)
                    color = response_data.get("color", "")
                    result_number = response_data.get("result_number", 0)
                    result_msg = response_data.get("result_msg", "")
                    success = (result_number == 1)
                    result = {
                        "success": success,
                        "rgb_switch": rgb_switch,
                        "brightness_value": brightness_value,
                        "color": color,
                        "description": result_msg if success else f"获取失败: {result_msg}",
                        "result_number": result_number
                    }
                    # if success:
                    #     logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取RGB灯带状态成功: {result}")
                    # else:
                    #     logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取RGB灯带状态失败: {result}")
                    return result
                except Exception as e:
                    logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] RGB灯带状态响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取RGB灯带状态失败: {e}")
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
                # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取机器人俯仰状态失败: {result}")
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
                    # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 机器人俯仰状态服务返回空响应")
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
                        logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取机器人俯仰状态成功: {result}")
                    else:
                        logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取机器人俯仰状态失败: {result}")
                    
                    return result
                    
                except Exception as e:
                    # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 机器人俯仰状态响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "angle": 0.0,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取机器人俯仰状态失败: {e}")
            return {
                "success": False,
                "angle": 0.0,
                "description": f"获取机器人俯仰状态失败: {str(e)}"
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
                logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取机身升降状态: {result}")
                return result
            else:
                # 如果服务调用失败，返回默认值
                result = {
                    "success": False,
                }
                # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取机身升降状态(默认值): {result}")
                return result
        except Exception as e:
            # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取机身升降状态失败: {e}")
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
                # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置屏幕俯仰角度失败: {result}")
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
                    # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 屏幕俯仰服务返回空响应")
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
                        logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置屏幕俯仰角度成功: {result}")
                    else:
                        logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置屏幕俯仰角度失败: {result}")
                    return result
                    
                except (json.JSONDecodeError, KeyError) as e:
                    # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置屏幕俯仰角度响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "angle": angle,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设置屏幕俯仰角度失败: {e}")
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
                # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取屏幕俯仰状态失败: {result}")
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
                    # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 屏幕俯仰状态服务返回空响应")
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
                    
                    # if success:
                    #     logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取屏幕俯仰状态成功: {result}")
                    # else:
                    #     logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取屏幕俯仰状态失败: {result}")
                    
                    return result
                    
                except Exception as e:
                    # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 屏幕俯仰状态响应解析失败: {e}, 原始响应: {response}")
                    return {
                        "success": False,
                        "angle": 0.0,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 获取屏幕俯仰状态失败: {e}")
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
        # logger.info(f"[BATTERY] 收到电池电量更新: {battery_level}%")
        
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
            # logger.info(f'电池电量已通过USB发送: {battery_level:.1f}%')
        else:
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}]USB管理器不可用,无法发送电池电量信息") 
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
                # logger.info("USB串口连接成功")
                # 添加消息回调
                self.serial_manager.add_callback(self._handle_received_message)
                # 开始接收数据
                self.serial_manager.start_receiving()
                # logger.info("USB串口通信已启动，直接双向通信")
                return True
            else:
                logger.error("USB串口连接失败")
                return False
        except Exception as e:
            logger.error(f"初始化USB串口失败: {e}")
            return False
    
    def _handle_received_message(self, message: Dict[Any, Any]):
        """处理接收到的消息 - 使用线程池实现真正的并发"""
        try:
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 接收到USB消息: {message}")

            # 将消息转发给agent处理
            if self.agent and hasattr(self.agent, 'handle_client_message'):
                # 使用独立线程处理每个消息，实现真正的并发
                # 每个消息都在独立的线程中执行，互不阻塞
                thread = threading.Thread(
                    target=self._process_message_in_thread,
                    args=(message,),
                    daemon=True,
                    name=f"MessageHandler-{datetime.now().strftime('%H%M%S%f')}"
                )
                thread.start()
            else:
                logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] Agent或handle_client_message方法不可用")
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 处理USB消息失败: {e}")

    def _process_message_in_thread(self, message: Dict[str, Any]):
        """在独立线程中处理消息 - 使用线程独立的事件循环和WebSocket连接"""
        try:
            # 检查agent是否存在
            if not self.agent:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] Agent不可用，无法处理消息")
                return
            
            # 在线程中创建新的事件循环来运行异步任务
            # 每个线程有独立的事件循环和WebSocket连接，实现真正的并发
            import asyncio
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            try:
                # 运行异步任务
                loop.run_until_complete(self.agent._execute_task_concurrent(message))
            finally:
                # 清理线程本地的WebSocket连接
                if hasattr(self.agent, '_thread_local'):
                    thread_local = self.agent._thread_local
                    if hasattr(thread_local, 'websocket') and thread_local.websocket is not None:
                        try:
                            loop.run_until_complete(thread_local.websocket.close())
                        except Exception:
                            pass
                        thread_local.websocket = None
                
                # 关闭事件循环
                loop.close()
                
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 线程处理消息失败: {e}")
    
    async def send_message(self, message: Dict[Any, Any]) -> bool:
        """发送消息到客户端"""
        try:
            if not self.connected:
                logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] USB串口未连接，无法发送消息")
                return False
            
            success = self.serial_manager.send_message(message)
            if success:
                logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 已发送USB消息: {message}")
            else:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 发送USB消息失败: {message}")
            return success
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 发送USB消息异常: {e}")
            return False    

    
    def cleanup(self):
        """清理资源"""
        try:
            self.serial_manager.stop_receiving()
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] USB串口资源已清理")
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
    """智能机器人Agent - USB串口版本 (集成ReAct框架)"""
    
    def __init__(self):
        self.ros2_interface = ROS2Interface()

        # 创建USB串口通信管理器
        self.usb_manager = USBCoordinateManager(self)

        # 任务中断标志
        self._task_interrupted = False

        # 事件循环引用（用于从其他线程调度任务）
        self.event_loop: Optional[asyncio.AbstractEventLoop] = None
        # 线程本地存储，用于隔离WebSocket连接
        self._thread_local = threading.local()

        # 本地模型连接相关
        self.local_model_websocket = None
        self.local_model_connected = False
        # self.local_model_uri = "ws://localhost:8769"
        self.local_model_uri = "ws://192.168.31.180:8000/ws/navigate"
        # self.local_model_uri = "ws://192.168.8.229:8000/ws/navigate"
        # 任务执行状态跟踪
        self.active_navigation_tasks = set()  # 正在执行的导航任务ID集合
        self.task_execution_lock = asyncio.Lock()  # 任务执行锁
        
        # 本地模型连接锁
        self.local_model_lock = asyncio.Lock()
        
        # 消息队列用于处理USB接收的消息
        self.message_queue = queue.Queue()
        
        # 退出控制标志
        self._running = False
        
        # ======================
        # ReAct框架组件
        # ======================
        self.memory = AgentMemory(max_history=50)  # Agent记忆系统
        self.react_enabled = True  # 是否启用ReAct模式
        self.max_react_iterations = 8  # 最大思考-行动循环次数
        self.current_react_task = None  # 当前ReAct任务
        
        # 已知的任务类型列表（可直接执行，无需LLM）
        self.known_task_types = {
            "find_object", "go_to_object", "go_find_person", "follow_person",
            "back_to_last_position", "stop_follow", "stop_navigate", "stop_move",
            "get_move_mode", "get_medicine_box_state", "set_medicine_box_switch",
            "get_robot_rise_state", "set_robot_rise_jqr",
            "get_robot_tilt_state", "set_robot_tilt_jqr",
            "get_screen_tilt_state", "set_screen_tilt_jqr",
            "set_laser_pointer", "get_laser_pointer_state",
            "set_rgb_light_strip", "get_rgb_light_strip_state"
        }
    
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
                    logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 电池电量监控已启动")
                    
                else:
                    logger.warning("电池电量监控启动失败")
                
                # 启动位置订阅
                position_success = self.ros2_interface.subscribe_robot_position()
                if position_success:
                    logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 机器人位置订阅已启动")
                else:
                    logger.warning("机器人位置订阅启动失败")
            
            # 初始化USB串口通信
            usb_connected = await self.usb_manager.initialize()
            if not usb_connected:
                logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] USB串口连接失败，无法继续初始化Agent")
                return False            
            
            # 启动消息处理循环
            self._running = True
            asyncio.create_task(self._message_processor())
            
            # logger.info("SmartRobotAgent初始化完成")
            return True
        except Exception as e:
            logger.error(f"初始化SmartRobotAgent失败: {e}")
            return False
    
    async def _message_processor(self):
        """消息处理循环 - 现在是空函数，消息直接通过 create_task 处理"""
        # 不再需要队列处理循环，所有消息通过 _handle_received_message 直接异步处理
        pass
    
    async def handle_client_message(self, message: Dict[Any, Any]):
        """处理来自客户端消息
        
        智能Agent处理流程:
        1. 简单已知任务：直接执行
        2. 自然语言/未知任务：发给LLM分析并获取标准格式任务
        3. LLM返回JSON: {"type": "...", "params": {...}}
        4. Agent执行任务或直接返回答案
        """
        try:
            # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 收到客户端消息: {message}")
            
            # 重置任务状态
            self.memory.clear_episode()
            
            # 1. type类型任务（最高优先级）
            if isinstance(message, dict) and "type" in message:
                task_to_execute = message.copy()
                task_type = task_to_execute.get("type", "")
                task_params = task_to_execute.get("params", {})
                
                # 检查是否为已知任务类型
                if task_type in self.known_task_types:
                    # 已知任务类型，后台并发执行，不等待完成
                    # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 已知任务类型 '{task_type}'，后台并发执行")
                    # 创建后台任务执行，不等待完成
                    asyncio.create_task(self._execute_task_async(task_to_execute))
                    # 立即返回，不等待任务完成
                    return
                else:
                    # 未知任务类型，发给LLM处理
                    logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 未知任务类型 '{task_type}'，发送给LLM分析")
                    user_prompt = f"执行任务: {task_type}，参数: {task_params}"
                    llm_result = await self.analyze_with_llm(user_prompt, task_type)
                    result = llm_result
                    await self.send_response_to_client(result)
                    return
                return
            
            # 2. 自然语言任务（不含type字段）
            user_prompt = None
            
            # 情况A: 消息本身就是字符串（自然语言内容）
            if isinstance(message, str):
                user_prompt = message
                logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 收到自然语言字符串任务: {user_prompt}")
            
            # 情况B: 其他字典格式（不含type，提取第一个字符串值作为user_prompt）
            elif isinstance(message, dict):
                for key, value in message.items():
                    if isinstance(value, str) and len(value.strip()) > 0:
                        user_prompt = value
                        logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 从字段 '{key}' 中提取自然语言任务: {user_prompt}")
                        break
            
            # 发送自然语言任务给LLM分析
            if user_prompt:
                logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 发送自然语言任务给LLM分析")
                llm_result = await self.analyze_with_llm(user_prompt, "natural_language")
                await self.send_response_to_client(llm_result)
                return
        
        except Exception as e:
            # logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 处理客户端消息失败: {e}", exc_info=True)
            error_response = {
                "type": message.get("type", "unknown") if isinstance(message, dict) else "unknown",
                "success": False,
                "error_msg": str(e),
            }
            await self.send_response_to_client(error_response)
    
    async def send_response_to_client(self, response: Dict[Any, Any]):
        """发送响应给客户端"""
        try:
            # 发送到串口
            success = await self.usb_manager.send_message(response)
            # if success:
            #     logger.info(f"响应已发送到客户端: {response.get('type', 'unknown')}")
            # else:
            #     logger.error(f"发送响应失败: {response}")
        except Exception as e:
            logger.error(f"发送响应失败: {e}")

    async def _execute_task_async(self, task: Dict[str, Any]):
        """后台异步执行任务（并发执行）

        Args:
            task (Dict[str, Any]): 任务字典
        """
        task_type = None
        try:
            task_type = task.get("type") if task else None
            # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 开始后台执行任务: {task_type}")

            # 执行任务
            result = await self.execute_task(task)

            # 记录到记忆
            self.memory.add_task(task, result)

            # 发送响应到客户端
            await self.send_response_to_client(result)
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 任务执行完成: {task_type}")
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 后台任务执行异常: {task_type}, 错误: {e}")

    async def _execute_task_concurrent(self, message: Dict[str, Any]):
        """并发执行消息处理（直接执行任务，不经过handle_client_message）
        
        这个方法绕过 handle_client_message，直接处理消息并执行任务，
        避免消息在事件循环中排队等待
        """
        try:
            # logger.info(f"[CONCURRENT_EXECUTE] 开始并发处理消息: {message}")

            # 重置任务状态
            self.memory.clear_episode()

            # 只处理已知任务类型
            if isinstance(message, dict) and "type" in message:
                task_type = message.get("type", "")
                
                # 检查是否为已知任务类型
                if task_type in self.known_task_types:
                    # 已知任务类型，直接执行
                    # logger.info(f"[CONCURRENT_EXECUTE] 已知任务类型 '{task_type}'，直接执行")
                    result = await self.execute_task(message)
                    # 记录到记忆
                    self.memory.add_task(message, result)
                    # 发送响应
                    await self.send_response_to_client(result)
                else:
                    # 未知任务类型，调用 handle_client_message
                    # logger.info(f"[CONCURRENT_EXECUTE] 未知任务类型 '{task_type}'，使用常规处理")
                    await self.handle_client_message(message)

            # logger.info(f"[CONCURRENT_EXECUTE] 消息处理完成")
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 并发处理消息失败: {e}")

    # ======================
    # LLM智能分析核心方法
    # ======================

    async def analyze_with_llm(self, user_prompt: str, task_type: str) -> Dict[str, Any]:
        """使用LLM分析用户指令，返回标准格式任务
        
        Args:
            user_prompt (str): 用户指令/任务描述
            task_type (str): 任务类型标识
            
        Returns:
            Dict[str, Any]: 执行结果或回答
        """
        logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] ===== 开始LLM分析 =====")
        logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 任务类型: {task_type}")
        logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 用户指令: {user_prompt}")
        
        # 记录初始任务
        initial_task = {
            "type": task_type,
            "user_prompt": user_prompt
        }
        self.memory.add_task(initial_task, {})
        
        # 构造system_prompt（包含Agent已知能力）
        system_prompt = self._build_system_prompt()
        
        # 构造完整的提示词
        full_prompt = f"""{system_prompt}

用户指令: {user_prompt}

请分析用户指令，判断是执行任务还是交互问答，并严格按照以下JSON格式返回：
{{
    "type": "任务类型",
    "params": {{"参数名": "参数值"}}
}}

说明:
- 如果需要执行任务，type从上面可用工具中选择，params填写对应参数
- 如果是问答类请求（如问时间、天气、打招呼等），type使用"natural_response"，params中填写"response"字段作为回答内容
- 必须严格返回有效JSON格式，不要包含其他文字
"""
        
        # 调用LLM
        llm_response = await self._call_llm_for_analysis(full_prompt)
        logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] LLM响应: {llm_response}")
        
        # 解析LLM响应
        try:
            task_data = json.loads(llm_response)
            result_type = task_data.get("type", "")
            result_params = task_data.get("params", {})
            
            # 记录思考
            thought = AgentThought(
                content=f"分析用户指令，决定执行任务: {result_type}",
                reasoning_type="planning"
            )
            self.memory.add_thought(thought)
            
            # 判断任务类型
            if result_type == "natural_response":
                # 交互问答类，直接返回回答
                logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 交互问答类，直接返回回答")
                response_content = result_params.get("response", llm_response)
                return {
                    "type": task_type,
                    "success": True,
                    "result": response_content,
                    "description": response_content
                }
            elif result_type in self.known_task_types:
                # 已知任务类型，执行任务
                logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 执行已知任务: {result_type}")
                task_to_execute = {
                    "type": result_type,
                    "params": result_params
                }
                result = await self.execute_task(task_to_execute)
                
                # 记录行动和观察
                self.memory.add_action(result_type, result_params)
                observation = Observation(
                    content=result.get("result", result.get("description", str(result))),
                    source=f"action_{result_type}",
                    success=result.get("success", False)
                )
                self.memory.add_observation(observation)
                
                return result
            else:
                # 未知任务类型
                logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 未知任务类型: {result_type}")
                return {
                    "type": task_type,
                    "success": False,
                    "error_msg": f"未知任务类型: {result_type}"
                }
                
        except json.JSONDecodeError as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] LLM响应JSON解析失败: {e}, 原始响应: {llm_response}")
            # 如果解析失败，尝试直接作为自然语言回复
            return {
                "type": task_type,
                "success": True,
                "result": llm_response,
                "description": llm_response
            }
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 处理LLM响应失败: {e}")
            return {
                "type": task_type,
                "success": False,
                "error_msg": f"处理失败: {str(e)}"
            }

    def _build_system_prompt(self) -> str:
        """构造system_prompt，包含Agent已知能力
        
        Returns:
            str: system_prompt
        """
        available_tools = list(self.known_task_types)
        
        system_prompt = f"""你是一个智能机器人Agent的助手，负责分析用户指令并决定如何响应。

Agent已知的能力（可用工具）:
"""
        
        # 添加每个工具的说明
        tool_descriptions = {
            "find_object": "查找指定对象",
            "go_to_object": "导航到指定对象位置",
            "go_find_person": "去寻找指定的人",
            "follow_person": "跟随指定的人",
            "back_to_last_position": "返回到初始位置",
            "stop_follow": "停止跟随",
            "stop_navigate": "停止导航",
            "stop_move": "停止移动",
            "get_move_mode": "获取当前运动模式",
            "set_medicine_box_switch": "控制药箱开关",
            "get_medicine_box_state": "获取药箱状态",
            "get_robot_rise_state": "获取机器人升降状态",
            "set_robot_rise_jqr": "控制机器人升降",
            "get_robot_tilt_state": "获取机器人俯仰状态",
            "set_robot_tilt_jqr": "控制机器人俯仰角度",
            "get_screen_tilt_state": "获取屏幕俯仰状态",
            "set_screen_tilt_jqr": "控制屏幕俯仰",
            "set_laser_pointer": "控制激光笔开关",
            "get_laser_pointer_state": "获取激光笔状态",
            "set_rgb_light_strip": "设置RGB灯光",
            "get_rgb_light_strip_state": "获取RGB灯光状态"
        }
        
        for tool in available_tools:
            desc = tool_descriptions.get(tool, "未知工具")
            system_prompt += f"- {tool}: {desc}\n"
        
        system_prompt += """
其他说明:
- natural_response: 用于直接回答用户的问题或进行对话（如打招呼、问答、闲聊等）
- 如果用户指令不匹配上述任何工具，请使用natural_response直接回复
- 必须严格按照JSON格式返回，不要包含任何其他解释性文字
"""
        
        return system_prompt

    async def _call_llm_for_analysis(self, prompt: str) -> str:
        """调用LLM进行任务分析
        
        Args:
            prompt (str): 完整的提示词（包含system_prompt和user_prompt）
            
        Returns:
            str: LLM的JSON格式响应
        """
        try:
            # 构造LLM请求格式
            llm_request = {
                "type": "task_analysis",
                "system_prompt": prompt,
                "enable_react": False  # 不需要ReAct模式，直接返回JSON
            }
            
            # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 发送分析请求给LLM")
            response = await self.send_to_local_model(llm_request)
            
            # 提取LLM的响应内容
            if response and "result" in response:
                return response["result"]
            elif response and "content" in response:
                return response["content"]
            elif isinstance(response, str):
                return response
            else:
                logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] LLM响应格式异常: {response}")
                # 返回默认的natural_response
                return json.dumps({"type": "natural_response", "params": {"response": "抱歉，我无法理解您的指令"}})
                
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 调用LLM失败: {e}")
            # 返回默认的natural_response
            return json.dumps({"type": "natural_response", "params": {"response": f"分析失败: {str(e)}"}})
    
    async def execute_task(self, task: Dict[str, Any]) -> Dict[str, Any]:
        """执行单个任务
        
        Args:
            task (Dict[str, Any]): 任务字典，包含任务类型和参数
            
        Returns:
            Dict[str, Any]: 任务执行结果
        """
        task_type = task.get("type")
        task_params = task.get("params", {})
        
        # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 执行任务类型: {task_type}, 参数: {task_params}")
        
        if not task_type:
            return {"type": task_type or "unknown", "success": False, "error_msg": "任务类型为空"}
        
        # 检查任务类型并发控制
        success, error_msg = self.usb_manager.serial_manager.acquire_task_type_lock(task_type)
        if not success:
            # logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] {error_msg}")
            return {
                "type": task_type,
                "success": False,
                "error_msg": error_msg
            }

        try:
            # 直接使用params中的参数，通过_execute_task_by_type执行
            result = await self._execute_task_by_type(task_type, task_params)

            # 确保返回结果包含type字段
            if "type" not in result:
                result["type"] = task_type

            # 直接返回字典结果
            return result
        except Exception as e:
            logger.error(f"执行任务时出错: {e}")
            return {
                "type": task_type,
                "success": False,
                "error_msg": f"执行任务时出错: {str(e)}"
            }
        finally:
            # 任务执行完成，释放任务类型锁
            self.usb_manager.serial_manager.release_task_type_lock(task_type)

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
            return await self.go_to_object(**params)
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
    # async def find_object(self,obj_name: str) -> Dict[str, Any]:
    #     """
    #     查找物品的位置信息，按照ASM→DB→探索的优先级执行
        
    #     Args:
    #         obj_name (str): 物品名称
        
    #     Returns:
    #         Dict[str, Any]: 工具执行结果
    #     """
    #     logger.info(f"[FIND_OBJECT] 开始查找物品/人员: {obj_name}")
    #     try:
    #         # Step 1: ASM查询（最高优先级）
    #         asm_res = self.query_asm_object(obj_name)
    #         if asm_res:
    #             #打印asm_res
    #             print(asm_res)
    #             loc = asm_res["location"]
    #             logger.info(f"[FIND_OBJECT] 在ASM中找到 {obj_name} 位置: ({loc['x']}, {loc['y']})")
                
    #             # ASM找到：返回位置信息，询问用户是否需要导航
    #             result_msg = f"找到 {obj_name} 的位置：像素坐标 ({loc['x']}, {loc['y']})"
    #             logger.info(f"[FIND_OBJECT] {result_msg}")
                
    #             # 按照新格式返回结果，包含像素位置
    #             result_data = {
    #                 "type": "find_object",
    #                 "success": True,
    #                 "pixel_position": asm_res.get("pixel_position", []),  # 添加像素位置
    #                 "position_description": asm_res.get("object_description", "")  # 使用ASM中的描述
    #             }
                
    #             return result_data

    #         # Step 2: DB查询
    #         db_res = self.query_history_db(obj_name)
    #         if not db_res:
    #             # DB没有找到：返回失败结果
    #             result_data = {
    #                 "type": "find_object",
    #                 "success": False,
    #                 "pixel_position": None,
    #                 "position_description": None
    #             }
                
    #             return result_data

    #         logger.info(f"[FIND_OBJECT] 在DB中找到 {obj_name} 记录，时间: {db_res['last_show_time']}")
            
    #         # DB找到：直接反馈结果，不询问导航
    #         result_data = {
    #             "type": "find_object",
    #             "success": True,
    #             "pixel_position": [db_res["world_x"], db_res["world_y"]],
    #             "position_description": db_res["object_description"]
    #         }
            
    #         return result_data
    #     except Exception as e:
    #         result_data = {
    #             "type": "find_object",
    #             "success": False,
    #             "pixel_position": None,
    #             "position_description": None
    #         }
        
    #         return result_data
    
    async def go_to_object(self, obj_name: str, pixel_position: Optional[List[float]] = None) -> Dict[str, Any]:
        """导航到物体位置"""
        try:
            # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 开始导航到物体: {obj_name}")
            
            # 检查是否有新格式的tool和arguments
            model_data = {
                "type": "go_to_object",
                "user_prompt": f"去找{obj_name}",
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
            #         logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 收到中间信息: {cmd}")
            #         await self.usb_manager.send_message({"type": "go_to_object", "command": cmd})
            #         continue                
            #     if isinstance(response, dict) and "success" in response:
            #         success = response["success"]
            #         logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 收到最终结果: success={success}")
            #         if not success:
            #             result_msg["error_msg"] = response.get("error_msg", "导航失败")
            #         # 通过 USB 发给客户端
            #         result_msg["success"] = success
            #         await self.usb_manager.send_message(result_msg)
            #         final_sent = True
            #     await asyncio.sleep(0.2)
            # return result_msg    
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 导航到物体失败: {e}")
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
            # logger.info(f"[FOLLOW_PERSON] 开始跟随人员")
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
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 导航到物体失败: {e}")
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
            # logger.info(f"[GO_FIND_PERSON] 开始查找人员: {obj_name}")
            
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
                # logger.info(f"[STOP_MOVE] 检测到 {len(self.active_navigation_tasks)} 个活跃导航任务，发送停止命令")
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
                    # logger.info("[STOP_MOVE] 已清空活跃任务集合")
                        
                except Exception as e:
                    logger.warning(f"[STOP_MOVE] 发送停止命令到本地模型失败: {e}")
            else:
                logger.info("[STOP_MOVE] 当前没有活跃的导航任务")
            
            # 2. 在/cmd_vel话题上发一次0
            if ROS2_AVAILABLE:
                try:
                    # 使用ros2 topic publish命令发布速度为0的消息
                    cmd = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
                    # logger.info(f"[STOP_MOVE] 执行命令: {cmd}")
                    os.system(cmd)
                    # logger.info(f"[STOP_MOVE] 发布速度命令结果: {result}")
                    
                    # success_msg = "已停止机器人移动"
                    # logger.info(f"[STOP_MOVE] {success_msg}")
                    
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
            # logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] record_position_before_navigation 调用，self类型: {type(self)}")
            # 确保位置订阅已启动
            if hasattr(self, 'ros2_interface') and not self.ros2_interface.position_subscribed:
                # logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 启动位置订阅")
                self.ros2_interface.subscribe_robot_position()
            
            # 记录当前位置
            if hasattr(self, 'ros2_interface'):
                success = self.ros2_interface.record_current_position()
                # if success:
                #     logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 已在导航前记录当前位置")
                # else:
                #     logger.warning("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 无法记录当前位置，可能还没有位置信息")
                return success
            else:
                logger.warning("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] ROS2接口不可用")
                return False
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 记录位置失败: {e}")
            return False
    
    async def back_to_last_position(self) -> Dict[str, Any]:
        """
        返回到初始位置
        Returns:
            Dict[str, Any]: 返回导航结果
        """
        try:
            # logger.info("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 开始返回到初始位置")
            # 获取初始位置
            initial_position = self.ros2_interface.get_initial_position()
            if not initial_position:
                logger.warning("[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 没有记录的初始位置信息")
                return {
                    "success": False,
                    "error_msg": "没有记录的初始位置信息，无法返回"
                }
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 返回到初始位置: {initial_position['position']}")
            # 调用导航功能
            result = self.ros2_interface.navigate_to_position(initial_position)
            # 确保返回结果包含type字段
            result["type"] = "back_to_last_position"
            if result["success"]:
                logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 成功返回到初始位置")
            else:
                logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 返回初始位置失败: {result.get('error_msg', '未知错误')}")
            return result
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 返回初始位置时出错: {e}")
            return {
                "success": False,
                "error_msg": f"返回位置失败: {str(e)}"
            }
    
    # ============
    # 本地模型通信
    # ======================
    
    async def connect_to_local_model(self):
        """建立与本地模型的WebSocket连接（线程本地）"""
        import websockets
        
        # 获取线程本地存储
        if not hasattr(self, '_thread_local'):
            self._thread_local = threading.local()
        
        thread_local = self._thread_local
        
        # 清理可能存在的旧连接
        if hasattr(thread_local, 'websocket') and thread_local.websocket is not None:
            try:
                await thread_local.websocket.close()
            except Exception:
                pass
            thread_local.websocket = None
        
        # 尝试建立新连接
        try:
            connect_func = getattr(websockets, 'connect')
            thread_local.websocket = await connect_func(self.local_model_uri)
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 成功建立线程本地连接: {self.local_model_uri}")
            return True
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 建立线程本地连接失败: {e}")
            thread_local.websocket = None
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
        import websockets
        
        # 获取或创建线程本地的WebSocket连接
        thread_local = self._thread_local
        if not hasattr(thread_local, 'websocket') or thread_local.websocket is None:
            # 创建新的连接
            try:
                # 清理可能存在的旧连接
                if hasattr(thread_local, 'websocket') and thread_local.websocket is not None:
                    try:
                        await thread_local.websocket.close()
                    except Exception:
                        pass
                    thread_local.websocket = None
                
                # 建立新连接
                connect_func = getattr(websockets, 'connect')
                thread_local.websocket = await connect_func(self.local_model_uri)
                logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 成功建立线程本地连接: {self.local_model_uri}")
            except Exception as e:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 创建线程本地连接失败: {e}")
                return {"success": False, "error_msg": f"无法连接到本地模型服务器: {str(e)}"}
        
        websocket = thread_local.websocket
        
        try:
            # 发送数据
            message_str = json.dumps(model_data, ensure_ascii=False)
            await websocket.send(message_str)
            # logger.info(f"已发送到本地模型: {model_data}")
            
            intermediate_data = {
                "type": "",
                "command": ""
            }
            
            # 持续接收响应，直到收到最终结果
            final_response = None
            while self._running and websocket is not None:
                try:
                    response_str = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                    response_data = json.loads(response_str)
                    # logger.info(f"[LOCAL_MODEL] 收到响应: {response_data}")
                    
                    # 检查是否是最终结果（包含success字段或result字段）
                    if ("success" in response_data or "result" in response_data) and "command" not in response_data:
                        final_response = response_data
                        break
                    else:
                        # 中间信息，需要添加任务类型后转发给所有连接的客户端
                        task_type = model_data.get("type", "unknown")
                        intermediate_data["type"] = task_type
                        if "message" in response_data:
                            intermediate_data["command"] = response_data.get("message", "") 
                        else:
                            intermediate_data["command"] = response_data.get("command", "")
                        await self.usb_manager.send_message(intermediate_data)
                        # logger.info(f"[LOCAL_MODEL] 已转发中间信息给客户端: {intermediate_data}")
                except asyncio.TimeoutError:
                    # 超时检查运行状态
                    continue
                except Exception as e:
                    logger.error(f"接收本地模型响应时出错: {e}")
                    break
            
            return final_response if final_response else {"success": False, "error_msg": "未收到最终响应"}
            
        except Exception as e:
            logger.error(f"与本地模型通信失败: {e}")
            # 清理线程本地连接
            try:
                if hasattr(thread_local, 'websocket') and thread_local.websocket is not None:
                    await thread_local.websocket.close()
            except Exception:
                pass
            thread_local.websocket = None
            return {"success": False, "error_msg": f"本地模型通信失败: {str(e)}"}
    
    # ======================
    # ReAct框架核心方法
    # ======================
    
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
    # print("Smart Robot Agent is running...")
    # print(f"USB串口通信端口: {USB_SERIAL_PORT}@{USB_SERIAL_BAUDRATE}")
    # print("Type 'exit' to quit.")

    # 保存事件循环引用
    loop = asyncio.get_running_loop()

    # 初始化数据库
    init_database()

    # 修复ASM JSON文件
    fix_asm_json_format()

    # 创建智能机器人Agent
    global smart_robot_agent_instance
    agent = SmartRobotAgent()
    agent.event_loop = loop  # 保存事件循环引用
    smart_robot_agent_instance = agent
    # 初始化agent
    try:
        success = await agent.initialize()
        if not success:
            logger.error("SmartRobotAgent初始化失败，退出程序")
            return
        # logger.info("SmartRobotAgent启动成功")

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







