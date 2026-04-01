# -*- coding: utf-8 -*-
"""Smart Robot Agent with USB Serial Communication"""
import base64
import os
import json
import sqlite3
from datetime import datetime
import threading
import time
import asyncio
import logging
from contextlib import contextmanager
from typing import Optional, Dict, Any, List
import re
import queue
from enum import IntEnum

# 导入USB串口管理器（可选）
try:
    from usb_serial_manager import SerialManager
    SERIAL_AVAILABLE = True
except ImportError:
    SerialManager = None
    SERIAL_AVAILABLE = False
    logging.getLogger(__name__).info("usb_serial_manager不可用，串口功能已禁用")

# 导入WebSocket控制服务器
from websocket_control_server import WebSocketControlServer

# 导入配置
from config import config

# ======================
# 枚举定义
# ======================

class ResultCode(IntEnum):
    """ROS2服务调用结果码"""
    FAILURE = 0
    SUCCESS = 1
    PARTIAL = 2
    COMPLETE = 3

class MotorResultCode(IntEnum):
    """组合电机执行结果码"""
    SUCCESS = 101
    ABORTED = 102
    FAILED = 103
    REJECTED = 104

class MedicineBoxStatus(IntEnum):
    """药箱状态值 (实际协议中为 float，但值为整数)"""
    CLOSED = 0    # 关闭
    OPEN = 1      # 开启
    RUNNING = 2   # 运行中

class CallbackGroupType(IntEnum):
    """ROS2 回调组类型"""
    MUTUALLY_EXCLUSIVE = 0  # 默认互斥回调组
    REENTRANT = 1           # 可重入回调组
    FACE_RECOGNITION = 2    # 人脸识别专用回调组

# ======================
# 版本控制
# ======================
AGENT_VERSION = config.AGENT_VERSION  # 智能机器人Agent版本号

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s.%(msecs)03d] %(levelname)s - %(name)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
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

# ======================
# 机器人状态管理器（线程安全单例）
# ======================

class RobotStateManager:
    """机器人状态管理器（线程安全）"""
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self._initialized = True
        self._state_lock = threading.Lock()
        self._battery_level = 100.0
        self._battery_node = None
        self._battery_thread = None
        self._battery_thread_running = False
        self._robot_pose_node = None
        self._robot_pose_thread = None
        self._robot_pose_thread_running = False
        self._agent_instance = None

    @property
    def battery_level(self) -> float:
        with self._state_lock:
            return self._battery_level

    @battery_level.setter
    def battery_level(self, value: float):
        with self._state_lock:
            self._battery_level = value

    @property
    def agent_instance(self):
        return self._agent_instance

    @agent_instance.setter
    def agent_instance(self, value):
        self._agent_instance = value

    @property
    def battery_node(self):
        return self._battery_node

    @battery_node.setter
    def battery_node(self, value):
        self._battery_node = value

    @property
    def battery_thread(self):
        return self._battery_thread

    @battery_thread.setter
    def battery_thread(self, value):
        self._battery_thread = value

    @property
    def battery_thread_running(self) -> bool:
        return self._battery_thread_running

    @battery_thread_running.setter
    def battery_thread_running(self, value: bool):
        self._battery_thread_running = value

    @property
    def robot_pose_node(self):
        return self._robot_pose_node

    @robot_pose_node.setter
    def robot_pose_node(self, value):
        self._robot_pose_node = value

    @property
    def robot_pose_thread(self):
        return self._robot_pose_thread

    @robot_pose_thread.setter
    def robot_pose_thread(self, value):
        self._robot_pose_thread = value

    @property
    def robot_pose_thread_running(self) -> bool:
        return self._robot_pose_thread_running

    @robot_pose_thread_running.setter
    def robot_pose_thread_running(self, value: bool):
        self._robot_pose_thread_running = value


robot_state = RobotStateManager()
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
            LaserPointer, LaserPointerState,
            FaceDelete
        )
        jqr_ros_msgs = True
    except ImportError as e:
        logger.warning(f"jqr_ros_msgs 导入失败 (ImportError): {e}")
        logger.warning("请检查ROS2工作空间是否正确配置和source")
        jqr_ros_msgs = False
    except Exception as e:
        logger.error(f"jqr_ros_msgs 导入失败 (未知错误): {e}")
        jqr_ros_msgs = False
    ROS2_AVAILABLE = True
except ImportError as e:
    logger.warning(f"ROS2 rclpy 不可用: {e}")
    geometry_msgs = None
    jqr_ros_msgs = False

# 尝试导入cv2，如果不存在则忽略
try:
    import cv2
except ImportError:
    cv2 = None
    logger.warning("cv2模块未安装,视频处理功能将不可用")

# 导入subprocess用于系统调用
import subprocess

# ======================
# 配置
# ======================
ASM_JSON_PATH = config.ASM_JSON_PATH
VIDEO_BASE_DIR = config.VIDEO_BASE_DIR
DB_PATH = config.DB_PATH

os.makedirs(VIDEO_BASE_DIR, exist_ok=True)

# USB串口配置
USB_SERIAL_PORT = config.USB_SERIAL_PORT
USB_SERIAL_BAUDRATE = config.USB_SERIAL_BAUDRATE

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

@contextmanager
def get_db_connection(db_path: str = DB_PATH):
    """数据库连接上下文管理器，确保连接正确关闭"""
    conn = sqlite3.connect(db_path)
    try:
        yield conn
        conn.commit()
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()

def init_database():
    """初始化数据库"""
    try:
        with get_db_connection() as conn:
            cursor = conn.cursor()

            cursor.execute('''
                CREATE TABLE IF NOT EXISTS history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    command TEXT NOT NULL,
                    response TEXT NOT NULL
                )
            ''')
    except sqlite3.Error as e:
        logger.error(f"数据库初始化失败: {e}")

def save_to_history(command: str, response: str):
    """保存命令和响应到数据库"""
    try:
        with get_db_connection() as conn:
            cursor = conn.cursor()

            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            cursor.execute(
                'INSERT INTO history (timestamp, command, response) VALUES (?, ?, ?)',
                (timestamp, command, response)
            )
    except sqlite3.Error as e:
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
        self.robot_state_subscription = None  # 机器人状态订阅对象
        self.robot_state_monitoring_active = False  # 机器人状态监控是否激活标志
        self.motor_control_publisher = None  # 电机控制发布对象
        self.head_motor_control_publisher = None  # 头部电机控制发布对象
        self.combine_motor_control_publisher = None  # 组合电机控制发布对象
        self.combine_motor_result_subscription = None  # 组合电机控制结果订阅对象
        self.rgb_control_publisher = None  # RGB灯控制发布对象
        self.rgb_state_subscription = None  # RGB灯状态订阅对象
        self.rgb_monitoring_active = False  # RGB监控是否激活标志
        self.combine_motor_monitoring_active = False  # 组合电机监控是否激活标志
        self.combine_motor_result = {}  # 组合电机执行结果 {task_id: {"progress": 0-100, "status": 101/102/103}}
        self._motor_task_id_counter = 0  # 组合电机任务ID计数器（float32精度安全范围：1~16777215）
        self._last_motor_task_id = 0  # 上一次生成的task_id，用于去重
        self.robot_state = {
            "screen_tilt": 0.0,  # 屏幕俯仰角度
            "robot_tilt": 0.0,  # 机身俯仰角度
            "robot_rise": 0.0,  # 机身升降状态 (0.0=降下, 1.0=升起, 2.0=运行中)
            "medicine_box": 0.0,  # 药盒电机状态 (0.0=关闭, 1.0=开启, 2.0=运行中)
            "battery": 100.0  # 电池电量
        }
        self.rgb_state = {
            "rgb_switch": 0,  # RGB灯开关 (0=关闭, 1=开启)
            "rgb_mode": 0,  # RGB灯模式 (0-5)
            "rgb_speed": 0,  # RGB灯速度 (0-6)
            "is_incremental": 0,  # 是否增量式亮度调节 (0=非增量, 1=增量)
            "brightness": 0,  # RGB灯亮度 (0-255)
            "color": 0  # RGB灯颜色 (0-8)
        }
        self.rgb_state_lock = threading.Lock()  # RGB状态锁
        self.node = None  # ROS2节点
        self.initialized = False  # ROS2是否已初始化
        self.executor = None  # MultiThreadedExecutor
        self.ros2_thread = None  # ROS2处理线程
        self.ros2_thread_running = False  # ROS2线程是否运行

        # 并发服务调用支持
        self.service_clients = {}  # 服务客户端缓存 {service_name: (client, callback_group)}
        self.mutually_exclusive_callback_group = None  # 互斥回调组
        self.reentrant_callback_group = None  # 可重入回调组
        self.face_recognition_callback_group = None  # 人脸识别专用回调组

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
                CallbackGroupType.REENTRANT,
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
            success = (result_number in (ResultCode.SUCCESS, ResultCode.PARTIAL, ResultCode.COMPLETE))

            if success:
                logger.info(f"激光笔控制成功: {result_msg}")
                return {
                    "success": True,
                    "description": result_msg,
                    "result_number": result_number
                }
            else:
                logger.error(f"激光笔控制失败: {result_msg}")
                return {
                    "success": False,
                    "description": result_msg,
                    "result_number": result_number
                }

        except Exception as e:
            logger.error(f"设置激光笔失败: {e}")
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
                CallbackGroupType.REENTRANT,
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
            success = (result_number == ResultCode.SUCCESS)

            if success:
                logger.info(f"获取激光笔状态成功: state={laser_pointer_state}")
                return {
                    "success": True,
                    "laser_pointer_state": laser_pointer_state,
                    "description": result_msg,
                    "result_number": result_number
                }
            else:
                logger.error(f"获取激光笔状态失败: {result_msg}")
                return {
                    "success": False,
                    "laser_pointer_state": laser_pointer_state,
                    "description": f"获取失败: {result_msg}",
                    "result_number": result_number
                }

        except Exception as e:
            logger.error(f"获取激光笔状态失败: {e}")
            return {
                "success": False,
                "description": f"获取激光笔状态失败: {str(e)}"
            }

    def delete_person(self, person_id: str) -> Dict[str, Any]:
        """删除指定人脸人员

        Args:
            person_id (str): 要删除的人员ID（人员名称，如"爷爷"）

        Returns:
            Dict[str, Any]: 删除结果
        """
        try:
            # 使用异步服务调用
            result = self._call_ros2_service_async(
                "/delete_person",
                CallbackGroupType.REENTRANT,
                "jqr_ros_msgs/srv/FaceDelete",
                {"person_id": person_id},
                timeout=10.0
            )

            if not result.get("success"):
                error_msg = result.get("error_msg", "未知错误")
                return {
                    "success": False,
                    "error_msg": error_msg
                }

            # 解析响应数据
            response_dict = result.get("response", {})
            result_value = response_dict.get("result", False)
            err_msg = response_dict.get("err_msg", "")

            if result_value:
                logger.info(f"删除人员成功: {person_id}")
                return {
                    "success": True,
                    "obj_name": person_id,
                    "error_msg": ""
                }
            else:
                logger.error(f"删除人员失败: {person_id}, {err_msg}")
                return {
                    "success": False,
                    "obj_name": person_id,
                    "error_msg": err_msg or "删除失败"
                }

        except Exception as e:
            logger.error(f"删除人员失败: {e}")
            return {
                "success": False,
                "obj_name": person_id,
                "error_msg": f"删除人员失败: {str(e)}"
            }

    def start_battery_monitoring(self) -> bool:
        """开始电池电量监控（订阅模式，已弃用，使用 start_robot_state_monitoring）"""
        return self.start_robot_state_monitoring()
        
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

    def start_robot_state_monitoring(self) -> bool:
        """开始机器人状态监控（订阅 robot_state_update 和 rgb_state 话题）"""
        if hasattr(self, 'robot_state_subscribed') and self.robot_state_subscribed:
            logger.info("机器人状态监控已启动，跳过重复订阅")
            return True

        try:
            logger.info("正在启动机器人状态监控...")
            if not ROS2_AVAILABLE:
                logger.error("ROS2不可用，无法启动机器人状态监控")
                return False
            if not self.initialized:
                logger.error(f"ROS2未初始化，无法启动机器人状态监控。initialized={self.initialized}, node={'存在' if self.node else '不存在'}")
                return False
            if not self.node:
                logger.error("ROS2节点不存在，无法启动机器人状态监控")
                return False

            try:
                from std_msgs.msg import Float32MultiArray
                logger.info("std_msgs.msg.Float32MultiArray 导入成功")
            except ImportError:
                logger.error("std_msgs.msg.Float32MultiArray 不可用")
                return False

            self.robot_state_subscription = self.node.create_subscription(
                Float32MultiArray,
                '/robot_state_update',
                self._robot_state_callback,
                10
            )
            self.robot_state_monitoring_active = True
            self.robot_state_subscribed = True
            logger.info("机器人状态监控已启动，订阅话题: /robot_state_update")

            # 同时启动RGB状态监控
            self.start_rgb_state_monitoring()

            return True

        except Exception as e:
            logger.error(f"启动机器人状态监控失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def stop_robot_state_monitoring(self):
        """停止机器人状态监控"""
        try:
            # 先设置标志为False，避免回调继续执行
            self.robot_state_monitoring_active = False

            if hasattr(self, 'robot_state_subscription') and self.robot_state_subscription:
                self.robot_state_subscription.destroy()
                self.robot_state_subscription = None

            self.robot_state_subscribed = False
            logger.info("机器人状态监控已停止")
            return True

        except Exception as e:
            logger.error(f"停止机器人状态监控失败: {e}")
            return False

    def _robot_state_callback(self, msg):
        """机器人状态回调函数"""
        # 检查监控是否仍然激活，避免在订阅销毁后执行回调
        if not self.robot_state_monitoring_active:
            return

        try:
            data = msg.data
            if len(data) >= 5:
                self.robot_state["screen_tilt"] = float(data[0])
                self.robot_state["robot_tilt"] = float(data[1])
                self.robot_state["robot_rise"] = float(data[2])
                self.robot_state["medicine_box"] = float(data[3])
                self.robot_state["battery"] = float(data[4])
            else:
                logger.warning(f"robot_state_update 数据长度不足: {len(data)}，需要至少5个元素")
        except Exception as e:
            logger.error(f"机器人状态回调处理失败: {e}")
            import traceback
            traceback.print_exc()

    def _rgb_state_callback(self, msg):
        """RGB灯状态回调函数"""
        # 检查监控是否仍然激活，避免在订阅销毁后执行回调
        if not self.rgb_monitoring_active:
            return

        try:
            data = msg.data
            with self.rgb_state_lock:
                if len(data) >= 6:
                    self.rgb_state["rgb_switch"] = int(data[0])
                    self.rgb_state["rgb_mode"] = int(data[1])
                    self.rgb_state["rgb_speed"] = int(data[2])
                    self.rgb_state["is_incremental"] = int(data[3])
                    self.rgb_state["brightness"] = int(data[4])
                    self.rgb_state["color"] = int(data[5])
                else:
                    logger.warning(f"rgb_state 数据长度不足: {len(data)}，需要至少6个元素")
        except Exception as e:
            logger.error(f"RGB灯状态回调处理失败: {e}")
            import traceback
            traceback.print_exc()

    def start_rgb_state_monitoring(self):
        """启动RGB灯状态监控（订阅rgb_state话题）"""
        try:
            if not ROS2_AVAILABLE or not self.initialized or not self.node:
                logger.warning("ROS2不可用或未初始化，无法启动RGB状态监控")
                return False

            if self.rgb_state_subscription is not None:
                logger.warning("RGB状态监控已在运行")
                return False

            from std_msgs.msg import UInt8MultiArray

            self.rgb_state_subscription = self.node.create_subscription(
                UInt8MultiArray,
                '/rgb_state',
                self._rgb_state_callback,
                10
            )
            self.rgb_monitoring_active = True  # 设置监控激活标志
            logger.info("RGB灯状态监控已启动，订阅话题: /rgb_state")
            return True

        except Exception as e:
            logger.error(f"启动RGB状态监控失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def stop_rgb_state_monitoring(self):
        """停止RGB灯状态监控"""
        try:
            # 先设置标志为False，阻止回调执行
            self.rgb_monitoring_active = False

            # 销毁订阅
            if hasattr(self, 'rgb_state_subscription') and self.rgb_state_subscription:
                self.rgb_state_subscription.destroy()
                self.rgb_state_subscription = None

            # 短暂等待，确保所有回调都已处理完毕
            time.sleep(0.1)

            logger.info("RGB灯状态监控已停止")
            return True

        except Exception as e:
            logger.error(f"停止RGB灯状态监控失败: {e}")
            return False

    def _next_motor_task_id(self) -> int:
        """生成下一个组合电机任务ID（float32精度安全）

        使用当前时分秒 HHMMSS 作为task_id，最大值235959（6位），
        float32可精确表示。若同一秒内多次调用则自增+1避免重复。
        """
        from datetime import datetime
        now = datetime.now()
        task_id = now.hour * 10000 + now.minute * 100 + now.second
        if task_id <= self._last_motor_task_id:
            task_id = self._last_motor_task_id + 1
        self._last_motor_task_id = task_id
        return task_id

    def _combine_motor_result_callback(self, msg):
        """组合电机控制结果回调函数"""
        if not self.combine_motor_monitoring_active:
            return
        try:
            data = msg.data
            if len(data) >= 2:
                task_id = int(data[0])
                result = data[1]
                self.combine_motor_result[task_id] = {"result": result}
                logger.info(f"组合电机任务 {task_id} 结果: {result}")
        except Exception as e:
            logger.error(f"组合电机结果回调失败: {e}")

    def start_combine_motor_monitoring(self):
        """启动组合电机控制结果监控"""
        try:
            if not ROS2_AVAILABLE or not self.initialized or not self.node:
                return False
            if self.combine_motor_result_subscription is None:
                from std_msgs.msg import Float32MultiArray
                self.combine_motor_result_subscription = self.node.create_subscription(
                    Float32MultiArray,
                    '/combine_motor_control_result',
                    self._combine_motor_result_callback,
                    10
                )
                self.combine_motor_monitoring_active = True
                logger.info("组合电机控制结果监控已启动")
            return True
        except Exception as e:
            logger.error(f"启动组合电机监控失败: {e}")
            return False

    def stop_combine_motor_monitoring(self):
        """停止组合电机控制结果监控"""
        try:
            self.combine_motor_monitoring_active = False
            if self.combine_motor_result_subscription:
                self.combine_motor_result_subscription.destroy()
                self.combine_motor_result_subscription = None
            logger.info("组合电机控制结果监控已停止")
            return True
        except Exception as e:
            logger.error(f"停止组合电机监控失败: {e}")
            return False

    def get_robot_state(self) -> Dict[str, Any]:
        """获取机器人状态"""
        return self.robot_state.copy()

    def get_battery_level(self) -> float:
        """获取电池电量（从 robot_state 话题获取）"""
        return self.robot_state["battery"]

    def publish_motor_control(self, screen_tilt: float = 0.0, robot_tilt: float = 0.0,
                               robot_rise: float = 0.0, medicine_box: float = 0.0,
                               medicine_speed: float = 1.0) -> Dict[str, Any]:
        """发布电机控制指令到 motor_control 话题

        Args:
            screen_tilt (float): 屏幕俯仰角度
            robot_tilt (float): 机身俯仰角度
            robot_rise (float): 机身升降控制 (0.0=降下, 1.0=升起)
            medicine_box (float): 药盒电机控制 (0.0=关, 1.0=开)
            medicine_speed (float): 药盒电机速度 (0.0=慢档, 1.0=快档)

        Returns:
            Dict[str, Any]: 发布结果
        """
        try:
            if not ROS2_AVAILABLE or not self.initialized or not self.node:
                logger.warning("ROS2不可用或未初始化，无法发布电机控制")
                return {"success": False, "error_msg": "ROS2不可用或未初始化"}

            if self.motor_control_publisher is None:
                try:
                    from std_msgs.msg import Float32MultiArray
                    self.motor_control_publisher = self.node.create_publisher(
                        Float32MultiArray,
                        '/motor_control',
                        10
                    )
                except Exception as e:
                    logger.error(f"创建电机控制发布者失败: {e}")
                    return {"success": False, "error_msg": f"创建发布者失败: {str(e)}"}

            try:
                from std_msgs.msg import Float32MultiArray
                msg = Float32MultiArray()
                msg.data = [float(screen_tilt), float(robot_tilt), float(robot_rise),
                           float(medicine_box), float(medicine_speed)]
                self.motor_control_publisher.publish(msg)
                logger.info(f"电机控制指令已发布: 屏幕={screen_tilt:.1f}, 机身={robot_tilt:.1f}, 升降={robot_rise:.1f}, 药盒={medicine_box:.1f}, 速度={medicine_speed:.1f}")
                return {"success": True}
            except Exception as e:
                logger.error(f"发布电机控制指令失败: {e}")
                return {"success": False, "error_msg": f"发布失败: {str(e)}"}

        except Exception as e:
            logger.error(f"发布电机控制失败: {e}")
            return {"success": False, "error_msg": f"未知错误: {str(e)}"}

    def publish_head_motor_control(self, control_pitch: bool = False, pitch_angle: float = 0.0,
                                    control_yaw: bool = False, yaw_angle: float = 0.0) -> Dict[str, Any]:
        """发布头部电机控制指令到 head_motor_control 话题（新头部样机）

        Args:
            control_pitch (bool): 是否控制俯仰 (False=不控制, True=控制)
            pitch_angle (float): pitch角度（仅在control_pitch=True时有效）
            control_yaw (bool): 是否控制偏航 (False=不控制, True=控制)
            yaw_angle (float): yaw角度（仅在control_yaw=True时有效）

        Returns:
            Dict[str, Any]: 发布结果
        """
        try:
            if not ROS2_AVAILABLE or not self.initialized or not self.node:
                logger.warning("ROS2不可用或未初始化，无法发布头部电机控制")
                return {"success": False, "error_msg": "ROS2不可用或未初始化"}

            if self.head_motor_control_publisher is None:
                try:
                    from std_msgs.msg import Float32MultiArray
                    self.head_motor_control_publisher = self.node.create_publisher(
                        Float32MultiArray,
                        '/head_motor_control',
                        10
                    )
                except Exception as e:
                    logger.error(f"创建头部电机控制发布者失败: {e}")
                    return {"success": False, "error_msg": f"创建发布者失败: {str(e)}"}

            try:
                from std_msgs.msg import Float32MultiArray
                msg = Float32MultiArray()
                msg.data = [
                    1.0 if control_pitch else 0.0,
                    float(pitch_angle),
                    1.0 if control_yaw else 0.0,
                    float(yaw_angle)
                ]
                self.head_motor_control_publisher.publish(msg)
                logger.info(f"头部电机控制指令已发布: 控制俯仰={control_pitch}, pitch={pitch_angle:.1f}, 控制偏航={control_yaw}, yaw={yaw_angle:.1f}")
                return {"success": True}
            except Exception as e:
                logger.error(f"发布头部电机控制指令失败: {e}")
                return {"success": False, "error_msg": f"发布失败: {str(e)}"}

        except Exception as e:
            logger.error(f"发布头部电机控制失败: {e}")
            return {"success": False, "error_msg": f"未知错误: {str(e)}"}

    def publish_combine_motor_control(self, task_id: float, control_pitch: bool = False, pitch_angle: float = 0.0,
                                       control_yaw: bool = False, yaw_angle: float = 0.0,
                                       control_chassis_move: bool = False, chassis_offset: float = 0.0,
                                       control_chassis_rotate: bool = False, chassis_rotation: float = 0.0,
                                       speed_level: int = 0) -> Dict[str, Any]:
        """发布组合电机控制指令到 combine_motor_control 话题

        Args:
            task_id (float): 任务ID，确保一个工作周期内id唯一
            control_pitch (bool): 是否控制俯仰
            pitch_angle (float): pitch角的目标角度，单位：弧度
            control_yaw (bool): 是否控制偏航
            yaw_angle (float): yaw角的目标角度，单位：弧度
            control_chassis_move (bool): 是否控制底盘位移
            chassis_offset (float): 底盘位置偏移量，正值前进，负值后退，单位：米
            control_chassis_rotate (bool): 是否控制底盘旋转
            chassis_rotation (float): 底盘旋转偏移量，正值逆时针，负值顺时针，单位：弧度
            speed_level (int): 执行档位，0=低速，1=中速，2=快速

        Returns:
            Dict[str, Any]: 发布结果
        """
        try:
            if not ROS2_AVAILABLE or not self.initialized or not self.node:
                logger.warning("ROS2不可用或未初始化，无法发布组合电机控制")
                return {"success": False, "error_msg": "ROS2不可用或未初始化"}

            if self.combine_motor_control_publisher is None:
                try:
                    from std_msgs.msg import Float32MultiArray
                    self.combine_motor_control_publisher = self.node.create_publisher(
                        Float32MultiArray,
                        '/combine_motor_control',
                        10
                    )
                except Exception as e:
                    logger.error(f"创建组合电机控制发布者失败: {e}")
                    return {"success": False, "error_msg": f"创建发布者失败: {str(e)}"}

            try:
                from std_msgs.msg import Float32MultiArray
                msg = Float32MultiArray()
                msg.data = [
                    float(task_id),
                    1.0 if control_pitch else 0.0,
                    float(pitch_angle),
                    1.0 if control_yaw else 0.0,
                    float(yaw_angle),
                    1.0 if control_chassis_move else 0.0,
                    float(chassis_offset),
                    1.0 if control_chassis_rotate else 0.0,
                    float(chassis_rotation),
                    float(speed_level)
                ]
                self.combine_motor_control_publisher.publish(msg)
                logger.info(f"组合电机控制指令已发布: task_id={task_id}, pitch={control_pitch}/{pitch_angle:.2f}, yaw={control_yaw}/{yaw_angle:.2f}, move={control_chassis_move}/{chassis_offset:.2f}, rotate={control_chassis_rotate}/{chassis_rotation:.2f}, speed={speed_level}")
                return {"success": True}
            except Exception as e:
                logger.error(f"发布组合电机控制指令失败: {e}")
                return {"success": False, "error_msg": f"发布失败: {str(e)}"}

        except Exception as e:
            logger.error(f"发布组合电机控制失败: {e}")
            return {"success": False, "error_msg": f"未知错误: {str(e)}"}

    async def _wait_for_motor_result(self, task_id: int, timeout: float = 20.0) -> Dict[str, Any]:
        """等待组合电机执行结果

        Args:
            task_id: 任务ID
            timeout: 超时时间（秒）

        Returns:
            Dict[str, Any]: {"success": True/False, "result": 101/102/103, "error_msg": "..."}
        """
        import asyncio
        start_time = time.time()

        while time.time() - start_time < timeout:
            if task_id in self.combine_motor_result:
                result_value = self.combine_motor_result[task_id]["result"]
                if result_value == MotorResultCode.SUCCESS:
                    return {"success": True, "result": result_value}
                elif result_value == MotorResultCode.FAILED:
                    return {"success": False, "result": result_value, "error_msg": "电机执行失败"}
                elif result_value == MotorResultCode.ABORTED:
                    return {"success": False, "result": result_value, "error_msg": "电机执行中止"}
                elif result_value == MotorResultCode.REJECTED:
                    return {"success": False, "result": result_value, "error_msg": "电机拒绝执行"}
            await asyncio.sleep(0.1)

        return {"success": False, "error_msg": "等待电机反馈超时"}

    async def _execute_motor_step(self, task_id: float, control_pitch: bool = False, pitch_angle: float = 0.0,
                                   control_yaw: bool = False, yaw_angle: float = 0.0,
                                   control_chassis_move: bool = False, chassis_offset: float = 0.0,
                                   control_chassis_rotate: bool = False, chassis_rotation: float = 0.0,
                                   speed_level: int = 0, max_retries: int = 3) -> Dict[str, Any]:
        """执行单步电机控制并等待反馈，支持重试"""
        for retry in range(max_retries):
            # 清除旧的结果缓存，防止残留数据干扰
            self.combine_motor_result.pop(int(task_id), None)

            result = self.publish_combine_motor_control(
                task_id=task_id, control_pitch=control_pitch, pitch_angle=pitch_angle,
                control_yaw=control_yaw, yaw_angle=yaw_angle,
                control_chassis_move=control_chassis_move, chassis_offset=chassis_offset,
                control_chassis_rotate=control_chassis_rotate, chassis_rotation=chassis_rotation,
                speed_level=speed_level
            )
            if not result["success"]:
                return result

            wait_result = await self._wait_for_motor_result(int(task_id))
            if wait_result["success"]:
                return wait_result

            if retry < max_retries - 1:
                logger.warning(f"电机步骤执行失败，重试 {retry + 1}/{max_retries}")

        return {"success": False, "error_msg": f"电机步骤执行失败，已重试{max_retries}次"}

    async def user_position_tracking(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """用户移动位置时的视线跟踪
        
        Args:
            params: {
                "yaw_angle": float,  # 声源方向角度（弧度），255表示使用默认值
                "pitch_angle": float  # 俯仰角度（弧度），255表示使用默认值
            }
        """
        import math
        
        # 默认角度（弧度）
        DEFAULT_PITCH = math.radians(45)
        DEFAULT_YAW = math.radians(45)
        
        # 解析参数，255表示使用默认值
        yaw_angle = params.get("yaw_angle", 255)
        pitch_angle = params.get("pitch_angle", 255)
        
        if yaw_angle == 255:
            yaw_angle = DEFAULT_YAW
        if pitch_angle == 255:
            pitch_angle = DEFAULT_PITCH
            
        task_id = self._next_motor_task_id()
        return await self._execute_motor_step(
            task_id=task_id, control_pitch=True, pitch_angle=pitch_angle,
            control_yaw=True, yaw_angle=yaw_angle, speed_level=1
        )

    async def patrol_table_inspection(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """巡逻中停至桌子识别记忆物品"""
        import math
        import asyncio

        # 步骤1: 头部俯视
        task_id = self._next_motor_task_id()
        result = await self._execute_motor_step(
            task_id=task_id, control_pitch=True, pitch_angle=math.radians(-15), speed_level=0
        )
        if not result["success"]:
            return result

        # 步骤2: 头部左扫
        task_id = self._next_motor_task_id()
        result = await self._execute_motor_step(
            task_id=task_id, control_yaw=True, yaw_angle=math.radians(-45), speed_level=0
        )
        if not result["success"]:
            return result

        # 步骤3: 头部右扫
        task_id = self._next_motor_task_id()
        result = await self._execute_motor_step(
            task_id=task_id, control_yaw=True, yaw_angle=math.radians(45), speed_level=0
        )
        if not result["success"]:
            return result

        # 步骤4: 头部回正
        task_id = self._next_motor_task_id()
        return await self._execute_motor_step(
            task_id=task_id, control_pitch=True, pitch_angle=0.0,
            control_yaw=True, yaw_angle=0.0, speed_level=1
        )

    async def wake_head_range(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """声源在头部转角范围内
        
        Args:
            params: {
                "yaw_angle": float,  # 声源方向角度（弧度），255表示使用默认值
                "pitch_angle": float  # 俯仰角度（弧度），255表示使用默认值
            }
        """
        import math
        
        # 默认角度（弧度）
        DEFAULT_PITCH = math.radians(45)
        DEFAULT_YAW = math.radians(45)
        
        # 解析参数，255表示使用默认值
        yaw_angle = params.get("yaw_angle", 255)
        pitch_angle = params.get("pitch_angle", 255)
        
        if yaw_angle == 255:
            yaw_angle = DEFAULT_YAW
        if pitch_angle == 255:
            pitch_angle = DEFAULT_PITCH
            
        task_id = self._next_motor_task_id()
        return await self._execute_motor_step(
            task_id=task_id, control_pitch=True, pitch_angle=pitch_angle,
            control_yaw=True, yaw_angle=yaw_angle, speed_level=2
        )

    async def wake_beyond_head_range(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """声源超出头部转角极限
        
        Args:
            params: {
                "yaw_angle": float,  # 声源方向角度（弧度），255表示使用默认值
                "pitch_angle": float  # 俯仰角度（弧度），255表示使用默认值
            }
            
        根据声源方向计算底盘旋转角度：底盘旋转角度 = 声源yaw角度 - 头部偏航极限(90°)
        """
        import math

        # 默认角度（弧度）
        DEFAULT_PITCH = math.radians(45)
        DEFAULT_YAW = math.radians(90)  # 头部偏航极限
        HEAD_YAW_LIMIT = math.radians(90)  # 头部偏航极限
        
        # 解析参数，255表示使用默认值
        yaw_angle = params.get("yaw_angle", 255)
        pitch_angle = params.get("pitch_angle", 255)
        
        if yaw_angle == 255:
            yaw_angle = DEFAULT_YAW
            chassis_rotation = math.radians(90)  # 默认底盘旋转90°
        else:
            # 计算底盘旋转角度：声源方向 - 头部极限
            chassis_rotation = yaw_angle - HEAD_YAW_LIMIT
            
        if pitch_angle == 255:
            pitch_angle = DEFAULT_PITCH

        # 步骤1: 头部转至极限
        task_id = self._next_motor_task_id()
        result = await self._execute_motor_step(
            task_id=task_id, control_pitch=True, pitch_angle=pitch_angle,
            control_yaw=True, yaw_angle=HEAD_YAW_LIMIT, speed_level=2
        )
        if not result["success"]:
            return result

        # 步骤2: 底盘原地旋转
        task_id = self._next_motor_task_id()
        result = await self._execute_motor_step(
            task_id=task_id, control_chassis_rotate=True, chassis_rotation=chassis_rotation, speed_level=1
        )
        if not result["success"]:
            return result

        # 步骤3: 头部回正
        task_id = self._next_motor_task_id()
        return await self._execute_motor_step(
            task_id=task_id, control_yaw=True, yaw_angle=0.0, speed_level=2
        )

    async def wake_side_moving(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """行走中侧方被唤醒
        
        Args:
            params: {
                "yaw_angle": float  # 声源方向角度（弧度），255表示使用默认值
            }
            
        底盘旋转角度 = 声源yaw角度
        """
        import math

        # 默认角度（弧度）
        DEFAULT_YAW = math.radians(45)
        
        # 解析参数，255表示使用默认值
        yaw_angle = params.get("yaw_angle", 255)
        
        if yaw_angle == 255:
            yaw_angle = DEFAULT_YAW
            chassis_rotation = math.radians(45)  # 默认底盘旋转45°
        else:
            chassis_rotation = yaw_angle

        # 步骤1: 头部转向
        task_id = self._next_motor_task_id()
        result = await self._execute_motor_step(
            task_id=task_id, control_yaw=True, yaw_angle=yaw_angle, speed_level=2
        )
        if not result["success"]:
            return result

        # 步骤2: 底盘旋转
        task_id = self._next_motor_task_id()
        result = await self._execute_motor_step(
            task_id=task_id, control_chassis_rotate=True, chassis_rotation=chassis_rotation, speed_level=1
        )
        if not result["success"]:
            return result

        # 步骤3: 头部回正
        task_id = self._next_motor_task_id()
        return await self._execute_motor_step(
            task_id=task_id, control_yaw=True, yaw_angle=0.0, speed_level=2
        )

    async def wake_back_moving(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """行走中后方被唤醒并停止
        
        Args:
            params: {
                "yaw_angle": float  # 声源方向角度（弧度），255表示使用默认值
            }
            
        底盘旋转角度 = 声源yaw角度（后方约为180°）
        """
        import math

        # 默认角度（弧度）
        DEFAULT_YAW = math.radians(90)  # 头部偏航极限
        HEAD_YAW_LIMIT = math.radians(90)  # 头部偏航极限
        
        # 解析参数，255表示使用默认值
        yaw_angle = params.get("yaw_angle", 255)
        
        if yaw_angle == 255:
            yaw_angle = DEFAULT_YAW
            chassis_rotation = math.radians(180)  # 默认底盘旋转180°
        else:
            # 声源在后方，底盘旋转角度 = 声源方向
            chassis_rotation = yaw_angle

        # 步骤1: 头部转至极限
        task_id = self._next_motor_task_id()
        result = await self._execute_motor_step(
            task_id=task_id, control_yaw=True, yaw_angle=HEAD_YAW_LIMIT, speed_level=2
        )
        if not result["success"]:
            return result

        # 步骤2: 底盘原地旋转
        task_id = self._next_motor_task_id()
        result = await self._execute_motor_step(
            task_id=task_id, control_chassis_rotate=True, chassis_rotation=chassis_rotation, speed_level=1
        )
        if not result["success"]:
            return result

        # 步骤3: 头部回正
        task_id = self._next_motor_task_id()
        return await self._execute_motor_step(
            task_id=task_id, control_yaw=True, yaw_angle=0.0, speed_level=2
        )

    async def obstacle_avoidance_turn(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """绕行障碍物时的协同转向

        场景描述：机器人遇到障碍物需要绕行，路径需要向右转弯。

        Args:
            params: {
                "turn_angle": float,  # 转向角度（弧度），默认45°右转
                "head_speed": float,  # 头部转速（°/s），默认30°/s
            }

        动作流程：
        1. 头部提前向绕行方向（右侧）缓慢预转，引导视线（头部水平0°→45°）
        2. 底盘执行转向动作，配合头部完成路径调整
        3. 底盘绕行，头部"回正"

        速度特点：
        - 底盘以较快转动速度完成避障（speed_level=2）
        - 头部转动速度较慢（30°/s），保持视野平滑过渡，避免画面剧烈抖动
        """
        import math

        # 解析参数
        turn_angle = params.get("turn_angle", math.radians(45))  # 默认45°右转
        head_speed = params.get("head_speed", 30)  # 默认30°/s

        # 步骤1: 头部预转右侧45°（慢速引导视线）
        # 使用低速档位(speed_level=0)来实现慢速转动，模拟30°/s的效果
        task_id = self._next_motor_task_id()
        result = await self._execute_motor_step(
            task_id=task_id,
            control_yaw=True,
            yaw_angle=turn_angle,  # 头部右转45°
            speed_level=0  # 低速档位，保持视野平滑
        )
        if not result["success"]:
            return result

        # 步骤2: 底盘右转 + 头部回正（底盘快速避障）
        # 底盘和头部同时动作：底盘右转，头部回正
        task_id = self._next_motor_task_id()
        return await self._execute_motor_step(
            task_id=task_id,
            control_yaw=True,
            yaw_angle=0.0,  # 头部回正
            control_chassis_rotate=True,
            chassis_rotation=turn_angle,  # 底盘右转45°
            speed_level=2  # 快速档位，快速完成避障
        )

    def _initialize_ros2(self):
        """初始化ROS2"""
        global rclpy
        try:
            if not rclpy:
                logger.warning("rclpy模块不可用，跳过初始化")
                self.initialized = False
                return False

            # 初始化rclpy
            rclpy.init()

            # 创建节点
            self.node = rclpy.create_node('smart_robot_agent_ros2')

            # 创建回调组
            self._create_callback_groups()

            # 创建 MultiThreadedExecutor 支持多线程并发
            from rclpy.executors import MultiThreadedExecutor
            self.executor = MultiThreadedExecutor(num_threads=4)
            self.executor.add_node(self.node)

            self.initialized = True
            # 启动ROS2处理线程
            self._start_ros2_spin_thread()
            return True

        except Exception as e:
            logger.error(f"初始化失败: {e}")
            self.initialized = False
            return False
    
    def _start_ros2_spin_thread(self):
        """启动ROS2独立处理线程"""
        if self.ros2_thread is not None and self.ros2_thread.is_alive():
            logger.warning("ROS2处理线程已在运行")
            return
            
        self.ros2_thread_running = True
        self.ros2_thread = threading.Thread(target=self._ros2_spin_worker, daemon=True)
        self.ros2_thread.start()

    def _create_callback_groups(self):
        """创建回调组以支持并发服务调用"""
        try:
            from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

            # 创建互斥回调组（串行执行，用于需要互斥的操作）
            self.mutually_exclusive_callback_group = MutuallyExclusiveCallbackGroup()

            # 创建可重入回调组（并发执行，用于支持并发的服务调用）
            self.reentrant_callback_group = ReentrantCallbackGroup()

            # 创建独立的人脸识别回调组（用于耗时的人脸识别服务）
            self.face_recognition_callback_group = ReentrantCallbackGroup()

        except Exception as e:
            logger.error(f"创建回调组失败: {e}")
            self.mutually_exclusive_callback_group = None
            self.reentrant_callback_group = None
            self.face_recognition_callback_group = None

    def _ros2_spin_worker(self):
        """ROS2独立处理线程工作函数"""
        global rclpy
        try:
            logger.info("ROS2处理线程开始运行")
            spin_count = 0
            while self.ros2_thread_running and rclpy and rclpy.ok() and self.node:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                # 延长休眠时间，减少CPU占用（无任务时不需要频繁spin）
                time.sleep(0.05)
                spin_count += 1
                if spin_count % 200 == 0:  # 每200次输出一次心跳日志（约10秒一次）
                    logger.debug(f"ROS2 spin 线程运行中，已执行 {spin_count} 次")
        except Exception as e:
            logger.error(f"处理线程出错: {e}")
            import traceback
            traceback.print_exc()
        finally:
            logger.info("处理线程已退出")
    
    def stop_ros2_spin_thread(self):
        """停止ROS2处理线程"""
        try:
            self.ros2_thread_running = False
            if self.ros2_thread and self.ros2_thread.is_alive():
                self.ros2_thread.join(timeout=2.0)
            logger.info("处理线程已停止")
        except Exception as e:
            logger.error(f"停止处理线程失败: {e}")

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
                try:
                    rclpy.shutdown()
                except Exception as shutdown_error:
                    # 如果已经shutdown过，忽略错误
                    if "already called" in str(shutdown_error):
                        logger.debug("ROS2 context already shutdown")
                    else:
                        raise
                self.initialized = False
                logger.info("资源已清理")
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
                    return True
            
            logger.warning(f"服务 {service_name} 不存在")
            return False
        except Exception as e:
            logger.error(f"检查服务存在性失败: {e}")
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
                    return True

            logger.warning(f"动作 {action_name} 不存在")
            return False
        except Exception as e:
            logger.error(f"检查动作存在性失败: {e}")
            return False

    def _get_or_create_service_client(self, service_name: str, service_type: str, use_concurrent: int):
        """获取或创建服务客户端

        Args:
            service_name (str): 服务名称
            service_type (str): 服务类型字符串 (如 "jqr_ros_msgs/srv/RobotRise")
            use_concurrent (int): 是否使用并发回调组

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
                logger.error(f"无效的服务类型格式: {service_type}")
                return None

            package_name = parts[0]
            srv_name = parts[2]

            # 动态导入服务类型
            try:
                module = __import__(f'{package_name}.srv', fromlist=[srv_name])
                srv_class = getattr(module, srv_name)
            except (ImportError, AttributeError) as e:
                logger.error(f"无法导入服务类型 {service_type}: {e}")
                return None

            # 选择回调组
            if use_concurrent == CallbackGroupType.FACE_RECOGNITION:  # 人脸识别专用回调组
                callback_group = self.face_recognition_callback_group
            elif use_concurrent == CallbackGroupType.REENTRANT:  # 可重入回调组
                callback_group = self.reentrant_callback_group
            else:  # 默认互斥回调组
                callback_group = self.mutually_exclusive_callback_group

            # 创建服务客户端（如果node为None则返回None）
            if self.node is None:
                logger.error("节点未初始化，无法创建服务客户端")
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

            return client

        except Exception as e:
            logger.error(f"创建服务客户端失败: {e}")
            return None

    def _create_sensor_msgs_image(self, pil_image) -> Any:
        """创建 sensor_msgs/Image 对象从 PIL.Image

        Args:
            pil_image: PIL.Image 对象

        Returns:
            sensor_msgs.msg.Image 对象
        """
        try:
            # 导入必要的模块
            from sensor_msgs.msg import Image
            import numpy as np
            import cv2

            # 转换为 numpy 数组
            cv_img = np.array(pil_image)

            # 确定编码格式
            if pil_image.mode == 'RGB':
                encoding = 'rgb8'
            elif pil_image.mode == 'RGBA':
                encoding = 'rgba8'
            elif pil_image.mode == 'L':
                encoding = 'mono8'
            else:
                encoding = 'rgb8'  # 默认

            # 创建 Image 消息
            img_msg = Image()
            img_msg.height = cv_img.shape[0]
            img_msg.width = cv_img.shape[1]
            img_msg.encoding = encoding

            # 处理通道数
            if len(cv_img.shape) == 2:
                # 单通道
                img_msg.step = cv_img.shape[1]
                img_msg.data = cv_img.tobytes()
            else:
                # 多通道
                channels = cv_img.shape[2]
                img_msg.step = cv_img.shape[1] * channels
                img_msg.data = cv_img.tobytes()

            # 设置时间戳
            img_msg.header.stamp = self.node.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera"

            return img_msg

        except Exception as e:
            logger.error(f"创建 sensor_msgs/Image 失败: {e}")
            return None

    def _set_request_field_complex(self, request, field_name: str, value: Any) -> bool:
        """设置请求字段的复杂类型值

        Args:
            request: 请求对象
            field_name (str): 字段名称
            value: 字段值

        Returns:
            bool: 是否设置成功
        """
        try:
            # 检查字段是否存在
            if not hasattr(request, field_name):
                logger.warning(f"请求类型没有属性: {field_name}")
                return False

            # 获取字段类型
            field_value = getattr(request, field_name)

            # 如果是列表类型
            if isinstance(field_value, list):
                # 清空列表
                field_value.clear()

                # 处理每个元素
                if isinstance(value, list):
                    for item in value:
                        # 检查是否是 PIL.Image
                        if hasattr(item, 'mode'):  # PIL.Image 的特征
                            img_msg = self._create_sensor_msgs_image(item)
                            if img_msg:
                                field_value.append(img_msg)
                        elif isinstance(item, dict):
                            # 如果是字典，尝试直接赋值（用于简单类型）
                            field_value.append(item)
                        else:
                            # 其他类型，直接添加
                            field_value.append(item)
                else:
                    # 单个值，添加到列表
                    field_value.append(value)

                return True
            else:
                # 非列表类型，直接赋值
                setattr(request, field_name, value)
                return True

        except Exception as e:
            logger.error(f"设置请求字段失败: {field_name}, {e}")
            return False

    def _call_ros2_service_async(self, service_name: str, use_concurrent:int,service_type: str, request_data: dict, timeout: float = 10.0) -> Dict[str, Any]:
        """异步调用ROS2服务（支持并发）

        Args:
            service_name (str): 服务名称
            service_type (str): 服务类型
            request_data (dict): 请求数据（字典格式）
            timeout (float): 超时时间（秒）

        Returns:
            Dict[str, Any]: 服务响应结果
        """
        try:
            # 获取或创建服务客户端（使用可重入回调组支持并发）
            if(use_concurrent == CallbackGroupType.FACE_RECOGNITION):
                client = self._get_or_create_service_client(service_name, service_type, use_concurrent=CallbackGroupType.FACE_RECOGNITION)
            else:
                client = self._get_or_create_service_client(service_name, service_type, use_concurrent=CallbackGroupType.REENTRANT)
            if not client:
                return {
                    "success": False,
                    "error_msg": f"无法创建服务客户端: {service_name}"
                }

            # 等待服务可用
            t_wait_start = time.time()
            if not client.wait_for_service(timeout_sec=timeout):
                logger.error(f"服务 {service_name} 未在 {timeout} 秒内变为可用，耗时: {(time.time()-t_wait_start):.2f}s")
                return {
                    "success": False,
                    "error_msg": f"服务 {service_name} 未在 {timeout} 秒内变为可用"
                }
            logger.debug(f"服务 {service_name} 等待可用耗时: {(time.time()-t_wait_start):.2f}s")

            # 创建请求对象
            # Pylance 类型检查可能有误，srv_type.Request 在运行时存在
            if hasattr(client, 'srv_type'):
                request_type = client.srv_type.Request  # type: ignore
                if request_type is None:
                    logger.error("无法获取服务类型")
                    return {
                        "success": False,
                        "error_msg": "无法获取服务类型"
                    }
                request = request_type()  # type: ignore
            else:
                logger.error("客户端没有 srv_type 属性")
                return {
                    "success": False,
                    "error_msg": "客户端没有 srv_type 属性"
                }

            # 设置请求字段 - 支持复杂类型
            t_set_start = time.time()
            for key, value in request_data.items():
                self._set_request_field_complex(request, key, value)
            logger.debug(f"设置请求字段耗时: {(time.time()-t_set_start):.2f}s")

            # 同步调用服务（由于回调组是可重入的，多个服务调用可以并发执行）
            # 注意：这里使用同步调用但配合可重入回调组，ROS2会在后台处理多个服务请求
            t_call_start = time.time()
            future = client.call_async(request)
            logger.debug(f"call_async 耗时: {(time.time()-t_call_start):.2f}s")

            # 等待结果
            start_time = time.time()
            check_count = 0
            while not future.done():
                if time.time() - start_time > timeout:
                    logger.error(f"服务调用超时: {service_name}, 已检查 {check_count} 次")
                    return {
                        "success": False,
                        "error_msg": f"服务调用超时: {service_name}"
                    }
                time.sleep(0.01)
                check_count += 1
            logger.debug(f"等待响应完成，检查次数: {check_count}, 耗时: {(time.time()-start_time):.2f}s")

            response = future.result()

            # 将响应转换为字典
            response_dict = {}
            if hasattr(response, 'get_fields_and_field_types'):
                # Pylance 可能无法识别 get_fields_and_field_types，运行时它是正确的
                for field_name in response.get_fields_and_field_types():  # type: ignore
                    value = getattr(response, field_name)
                    # 处理数组类型（如 string[], CompressedImage[]）
                    if isinstance(value, list):
                        # 对于数组类型，将每个元素转换为字典
                        converted_list = []
                        for i, item in enumerate(value):
                            if hasattr(item, 'get_fields_and_field_types'):
                                # 复杂类型（如CompressedImage），转换为字典（优先检查）
                                item_dict = {}
                                for sub_field in item.get_fields_and_field_types():
                                    sub_value = getattr(item, sub_field)
                                    # 对于CompressedImage，直接使用原始值（包括data字段的bytearray）
                                    item_dict[sub_field] = sub_value
                                converted_list.append(item_dict)
                            elif hasattr(item, 'data'):
                                # std_msgs类型，提取data字段
                                logger.debug(f"转换数组元素[{i}]为std_msgs类型，提取data字段: type={type(item.data).__name__}")
                                converted_list.append(item.data)
                            else:
                                # 其他类型，直接添加
                                logger.debug(f"转换数组元素[{i}]: type={type(item).__name__}")
                                converted_list.append(item)
                        response_dict[field_name] = converted_list
                    # 处理std_msgs类型
                    elif hasattr(value, 'data'):
                        response_dict[field_name] = value.data
                    else:
                        response_dict[field_name] = value
            else:
                # 如果无法获取字段，尝试直接转换为字典
                response_dict = vars(response) if hasattr(response, '__dict__') else {}

            # 调试：记录响应的详细信息
            if 'rgb_images_compressed' in response_dict:
                logger.debug("rgb_images_compressed in response_dict")
                img_list = response_dict['rgb_images_compressed']
                logger.debug(f"rgb_images_compressed type: {type(img_list).__name__}, len: {len(img_list)}")
                if img_list:
                    logger.debug(f"rgb_images_compressed[0] type: {type(img_list[0]).__name__}")

            return {
                "success": True,
                "response": response_dict
            }

        except Exception as e:
            logger.error(f"异步服务调用失败: {e}")
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
            logger.error(f"服务 {service_name} 不存在，无法调用")
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
            logger.info(f"执行命令: {cmd}")
            
            
            # 使用subprocess而不是os.popen来获得更好的控制
            # 设置ROS环境变量
            import os
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = os.environ.get('ROS_DOMAIN_ID', '0')
            
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10, env=env)
            
            if result.returncode != 0:
                logger.error(f"服务调用命令执行失败，返回码: {result.returncode}")
                logger.error(f"stderr: {result.stderr}")
                return None
            
            response = result.stdout.strip()

            # 检查结果是否为空
            if not response:
                logger.error(f"服务 {service_name} 返回空响应")
                return None
                
            return response
        except subprocess.TimeoutExpired:
            logger.error(f"服务调用超时: {service_name}")
            return None
        except Exception as e:
            logger.error(f"服务调用失败: {e}")
            return None    
    
    def subscribe_robot_position(self) -> bool:
        """订阅机器人位置信息
        
        Returns:
            bool: 订阅是否成功
        """
        try:
            if not ROS2_AVAILABLE or not self.initialized or not self.node:
                logger.warning("ROS2不可用或未初始化，无法订阅位置信息")
                return False
            
            if not geometry_msgs:
                logger.error("geometry_msgs不可用，无法订阅位置话题")
                return False
                
            # 使用主节点创建位置订阅，订阅的回调由主spin循环处理
            self.position_subscription = self.node.create_subscription(
                geometry_msgs.PoseStamped,
                '/tracked_pose',  # 假设SLAM发布的话题名为 /tracked_pose
                self._position_callback,
                10
            )
            
            self.position_subscribed = True
            return True
            
        except Exception as e:
            logger.error(f"订阅位置信息失败: {e}")
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
                logger.info(f"已记录初始位置: ({position['position']['x']:.2f}, {position['position']['y']:.2f})")

        except Exception as e:
            logger.error(f"位置回调处理失败: {e}")
    
    def record_current_position(self) -> bool:
        """记录当前位置
        
        Returns:
            bool: 记录是否成功
        """
        if self.last_position:
            self.pre_position = self.last_position
            logger.info(f"已记录当前位置: {self.pre_position['position']}")
            return True
        else:
            logger.warning("没有可用的位置信息")
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
            logger.error(f"导航到位置失败: {e}")
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
                CallbackGroupType.REENTRANT,
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

            success = (result_number == ResultCode.SUCCESS)

            if success:
                logger.info(f"获取运动模式成功: mode={move_mode}, vel={linear_vel}")
                return {
                    "success": True,
                    "move_mode": move_mode,
                    "linear_vel": linear_vel,
                    "description": result_msg,
                    "result_number": result_number
                }
            else:
                logger.error(f"获取运动模式失败: {result_msg}")
                return {
                    "success": False,
                    "move_mode": move_mode,
                    "linear_vel": linear_vel,
                    "description": f"获取失败: {result_msg}",
                    "result_number": result_number
                }

        except Exception as e:
            logger.error(f"获取运动模式失败: {e}")
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
                CallbackGroupType.REENTRANT,
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

            success = (result_number == ResultCode.SUCCESS)

            if success:
                return {"success": True, "err_msg": ""}
            else:
                return {
                    "success": False,
                    "err_msg": result_msg
                }

        except Exception as e:
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
                CallbackGroupType.REENTRANT,
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

            success = (result_number == ResultCode.SUCCESS)

            if success:
                return {
                    "success": True,
                    "state": robot_rise_state,
                    "description": result_msg,
                    "result_number": result_number
                }
            else:
                return {
                    "success": False,
                    "state": robot_rise_state,
                    "description": f"获取失败: {result_msg}",
                    "result_number": result_number
                }

        except Exception as e:
            logger.error(f"获取机器人升降状态失败: {e}")
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
                CallbackGroupType.REENTRANT,
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

            success = (result_number == ResultCode.SUCCESS)

            if success:
                return {
                    "success": True,
                    "angle": angle,
                    "description": result_msg,
                    "result_number": result_number
                }
            else:
                return {
                    "success": False,
                    "angle": angle,
                    "description": f"设置失败: {result_msg}",
                    "result_number": result_number
                }

        except Exception as e:
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
            logger.error(f"动作 {action_name} 不存在，无法调用")
            return None
            
        try:
            # 构造ROS2动作调用命令
            cmd = f"ros2 action send_goal {action_name} {action_type} '{goal_data}'"

            # 使用subprocess而不是os.popen来获得更好的控制
            # 设置ROS环境变量
            import os
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = os.environ.get('ROS_DOMAIN_ID', '0')
            
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=120, env=env)
            
            if result.returncode != 0:
                logger.error(f"动作调用命令执行失败，返回码: {result.returncode}")
                logger.error(f"stderr: {result.stderr}")
                return None
            
            response = result.stdout.strip()

            # 检查结果是否为空
            if not response:
                logger.error(f"动作 {action_name} 返回空响应")
                return None
                
            return response
        except subprocess.TimeoutExpired:
            logger.error(f"动作调用超时: {action_name}")
            return None
        except Exception as e:
            logger.error(f"动作调用失败: {e}")
            return None
            
    # ======================
    # 药箱控制相关接口
    # ======================
    
    def set_medicine_box_switch(self, switch: bool, speed_stage: int) -> Dict[str, Any]:
        """控制药箱开关（使用话题控制）

        Args:
            switch (bool): 药箱开关状态 (True: 打开, False: 关闭)
            speed_stage (int): 速度档位 (1: 慢档, 2: 快档)

        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            medicine_box = 1.0 if switch else 0.0
            medicine_speed = 1.0 if speed_stage == 2 else 0.0

            result = self.publish_motor_control(
                screen_tilt=0.0,
                robot_tilt=0.0,
                robot_rise=0.0,
                medicine_box=medicine_box,
                medicine_speed=medicine_speed
            )

            if result.get("success"):
                return {
                    "type": "set_medicine_box_switch",
                    "success": True
                }
            else:
                return {
                    "type": "set_medicine_box_switch",
                    "success": False,
                    "error_msg": result.get("error_msg", "发布失败")
                }

        except Exception as e:
            return {
                "type": "set_medicine_box_switch",
                "success": False,
                "error_msg": f"设置药箱{'打开' if switch else '关闭'}失败: {str(e)}"
            }
    
    def get_medicine_box_state(self) -> Dict[str, Any]:
        """获取药箱状态（从 robot_state 话题获取）

        Returns:
            Dict[str, Any]: 药箱状态信息
        """
        try:
            medicine_box_value = self.robot_state["medicine_box"]

            # 映射状态值: 0.0=关闭, 1.0=开启, 2.0=运行中
            if medicine_box_value == MedicineBoxStatus.CLOSED:
                state = False
                state_desc = "关闭"
            elif medicine_box_value == MedicineBoxStatus.OPEN:
                state = True
                state_desc = "开启"
            elif medicine_box_value == MedicineBoxStatus.RUNNING:
                state = True
                state_desc = "运行中"
            else:
                state = False
                state_desc = "未知"

            return {
                "success": True,
                "state": state,
                "description": state_desc,
                "raw_value": medicine_box_value
            }

        except Exception as e:
            return {
                "success": False,
                "state": False,
                "description": f"获取药箱状态失败: {str(e)}"
            }

    def set_rgb_light_strip(self, brightness_set: Optional[int] = None, rgb_switch: Optional[bool] = None,
                             color: Optional[str] = None, is_incremental: bool = False,
                             rgb_mode: Optional[int] = None, rgb_speed: Optional[int] = None) -> Dict[str, Any]:
        """控制RGB灯带开关、颜色、亮度、模式和速度（通过话题发布）

        Args:
            rgb_switch (bool): RGB灯开关 (True: 开启, False: 关闭)，可选
            brightness_set (int): 亮度 0-255，可选
            color (str): 颜色名称 (red/green/blue/yellow/cyan/purple/white/warm_white)，可选
            is_incremental (bool): 是否增量调节 (True: 增量式, False: 非增量式)，默认False
            rgb_mode (int): 灯的模式 0-5，可选
              0=单色, 1=单色呼吸, 2=单色闪烁, 3=7色常亮循环, 4=7色呼吸循环, 5=7色闪烁循环
            rgb_speed (int): 控制速度 0-6档，可选（仅呼吸或闪烁模式生效）
              0=0.25s, 1=0.5s, 2=1s, 3=2.5s, 4=3.5s, 5=5s, 6=10s

        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            if not ROS2_AVAILABLE or not self.initialized or not self.node:
                return {
                    "success": False,
                    "error_msg": "ROS2不可用或未初始化"
                }

            # 确保发布者已创建
            if self.rgb_control_publisher is None:
                from std_msgs.msg import UInt8MultiArray
                self.rgb_control_publisher = self.node.create_publisher(
                    UInt8MultiArray,
                    '/rgb_control',
                    10
                )
                logger.info("RGB灯控制发布者已创建: /rgb_control")

            # 从当前状态获取默认值
            with self.rgb_state_lock:
                current_switch = self.rgb_state.get("rgb_switch", 0)
                current_mode = self.rgb_state.get("rgb_mode", 0)
                current_speed = self.rgb_state.get("rgb_speed", 0)
                current_brightness = self.rgb_state.get("brightness", 0)
                current_color = self.rgb_state.get("color", 0)

            # 构造控制数据
            data = [0, 0, 0, 0, 0, 0]  # [rgb_switch, rgb_mode, rgb_speed, is_incremental, brightness_set, color]

            # data[0]: 开关
            if rgb_switch is not None:
                data[0] = 1 if rgb_switch else 0
            else:
                data[0] = current_switch

            # data[1]: 模式
            if rgb_mode is not None:
                data[1] = rgb_mode
            else:
                data[1] = current_mode

            # data[2]: 速度
            if rgb_speed is not None:
                data[2] = rgb_speed
            else:
                data[2] = current_speed

            # data[3]: 是否增量调节
            data[3] = 1 if is_incremental else 0

            # data[4]: 亮度
            if brightness_set is not None:
                data[4] = brightness_set
            else:
                data[4] = current_brightness

            # data[5]: 颜色
            if color is not None:
                color_map = {
                    "red": 0,
                    "green": 1,
                    "blue": 2,
                    "yellow": 3,
                    "cyan": 4,
                    "purple": 5,
                    "white": 6,
                    "warm_white": 7,
                    "warm_white2": 8
                }
                data[5] = color_map.get(color.lower(), 0)
            else:
                data[5] = current_color

            # 发布控制消息
            from std_msgs.msg import UInt8MultiArray
            msg = UInt8MultiArray()
            msg.data = data
            self.rgb_control_publisher.publish(msg)

            logger.info(f"RGB灯控制指令已发布: 开关={data[0]}, 模式={data[1]}, 速度={data[2]}, "
                       f"增量={data[3]}, 亮度={data[4]}, 颜色={data[5]}")

            return {
                "success": True,
                "error_msg": ""
            }
        except Exception as e:
            logger.error(f"控制RGB灯失败: {e}")
            return {
                "success": False,
                "error_msg": f"控制RGB灯失败: {str(e)}"
            }

    def set_rgb(self, switch: Optional[bool] = None, mode: Optional[int] = None,
                color: Optional[str] = None) -> Dict[str, Any]:
        """设置RGB灯（简化的client接口，设置默认值）

        Args:
            switch (bool): 开关 (True: 开启, False: 关闭)，可选
            mode (int): 模式 (0=单色, 1=单色呼吸, 2=单色闪烁)，可选
            color (str): 颜色 (red/green/blue/yellow等)，可选

        Returns:
            Dict[str, Any]: 控制结果
        """
        # 设置默认值
        default_switch = True  # 默认开启
        default_mode = 0  # 默认单色模式
        default_color = "green"  # 默认绿色
        default_brightness = 100  # 默认亮度

        # 使用传入参数或默认值
        rgb_switch = switch if switch is not None else default_switch
        rgb_mode = mode if mode is not None else default_mode
        rgb_color = color if color is not None else default_color

        # 调用完整的 set_rgb_light_strip 方法
        return self.set_rgb_light_strip(
            rgb_switch=rgb_switch,
            rgb_mode=rgb_mode,
            color=rgb_color,
            brightness_set=default_brightness,
            rgb_speed=0,
            is_incremental=False
        )
    def get_rgb_light_strip_state(self) -> Dict[str, Any]:
        """获取RGB灯带状态（从rgb_state话题订阅获取）

        Returns:
            Dict[str, Any]: 灯带状态信息
            - rgb_switch (int): RGB灯开关状态 (0=关闭, 1=开启)
            - rgb_mode (int): 灯的模式 (0-5)
            - rgb_speed (int): 控制速度 (0-6)
            - is_incremental (int): 是否增量式亮度调节 (0=非增量, 1=增量)
            - brightness (int): 亮度值 0-255
            - color (int): 颜色 (0-8)
            - color_name (str): 颜色名称
            - success (bool): 是否获取成功
            - description (str): 描述信息
        """
        try:
            with self.rgb_state_lock:
                rgb_switch = self.rgb_state.get("rgb_switch", 0)
                rgb_mode = self.rgb_state.get("rgb_mode", 0)
                rgb_speed = self.rgb_state.get("rgb_speed", 0)
                is_incremental = self.rgb_state.get("is_incremental", 0)
                brightness = self.rgb_state.get("brightness", 0)
                color = self.rgb_state.get("color", 0)

            # 将颜色代码转换为名称
            color_names = {
                0: "red",
                1: "green",
                2: "blue",
                3: "yellow",
                4: "cyan",
                5: "purple",
                6: "white",
                7: "warm_white",
                8: "warm_white2"
            }
            color_name = color_names.get(color, "unknown")

            # 将模式代码转换为描述
            mode_descriptions = {
                0: "单色",
                1: "单色呼吸",
                2: "单色闪烁",
                3: "7色常亮循环",
                4: "7色呼吸循环",
                5: "7色闪烁循环"
            }
            mode_desc = mode_descriptions.get(rgb_mode, "未知")

            # 速度描述
            speed_descriptions = {
                0: "0.25s",
                1: "0.5s",
                2: "1s",
                3: "2.5s",
                4: "3.5s",
                5: "5s",
                6: "10s"
            }
            speed_desc = speed_descriptions.get(rgb_speed, f"{rgb_speed}")

            return {
                "success": True,
                "rgb_switch": rgb_switch,
                "rgb_mode": rgb_mode,
                "mode_description": mode_desc,
                "rgb_speed": rgb_speed,
                "speed_description": speed_desc,
                "is_incremental": is_incremental,
                "brightness": brightness,
                "color": color,
                "color_name": color_name,
            }

        except Exception as e:
            logger.error(f"获取RGB灯带状态失败: {e}")
            return {
                "success": False,
                "description": f"获取RGB灯带状态失败: {str(e)}"
            }

    def find_person(self, obj_name: str, user_prompt: str = "") -> Dict[str, Any]:
        """静态找人功能

        Args:
            obj_name (str): 人员名称/ID
            user_prompt (str): 用户原始指令，预留参数

        Returns:
            Dict[str, Any]: 找人结果
        """
        try:
            logger.info(f"开始找人: {obj_name}")

            # Step 1: 调用realsense_rgb_image服务获取4个相机的RGB图像
            camera_ids = ["cameraF", "cameraB", "cameraL", "cameraR"]

            # 构造请求数据 - 按照通信协议
            # Request: string[] camera_ids
            # Response: string[] camera_ids, sensor_msgs/CompressedImage[] rgb_images_compressed
            request_data = {
                "camera_ids": camera_ids
            }

            # 使用异步服务调用，直接返回Python对象，避免文本解析问题
            result = self._call_ros2_service_async(
                "/realsense_rgb_image",
                CallbackGroupType.FACE_RECOGNITION,
                "jqr_ros_msgs/srv/RealSenseRGBImage",
                request_data,
                timeout=10.0
            )

            if not result.get("success"):
                logger.error(f"相机图像数据获取异常: {result.get('error_msg', '未知错误')}")
                return {
                    "type": "find_person",
                    "success": False,
                    "obj_name": obj_name,
                    "result_msg": "相机图像数据获取异常"
                }

            # 解析响应 - 使用异步服务调用的响应，已经是字典格式
            try:
                response_data = result.get("response", {})
                returned_camera_ids = response_data.get("camera_ids", [])
                # rgb_images_compressed 是 sensor_msgs/CompressedImage[] 类型
                rgb_images_compressed = response_data.get("rgb_images_compressed", [])
                logger.info(f"获取到 {len(returned_camera_ids)} 个相机的图像数据")

                # 详细调试：输出响应数据的完整结构
                logger.debug(f"response_data keys: {list(response_data.keys())}")
                logger.debug(f"rgb_images_compressed type: {type(rgb_images_compressed).__name__}")
                logger.debug(f"rgb_images_compressed length: {len(rgb_images_compressed)}")
                if rgb_images_compressed:
                    logger.debug(f"rgb_images_compressed[0] type: {type(rgb_images_compressed[0]).__name__}")
                    logger.debug(f"rgb_images_compressed[0] has get: {hasattr(rgb_images_compressed[0], 'get')}")
                    logger.debug(f"rgb_images_compressed[0] has get_fields: {hasattr(rgb_images_compressed[0], 'get_fields_and_field_types')}")
                    if isinstance(rgb_images_compressed[0], dict):
                        logger.debug(f"rgb_images_compressed[0] keys: {list(rgb_images_compressed[0].keys())}")
                        if 'data' in rgb_images_compressed[0]:
                            logger.debug(f"rgb_images_compressed[0]['data'] type: {type(rgb_images_compressed[0]['data']).__name__}")

                if len(returned_camera_ids) != 4:
                    logger.warning(f"期望4个相机数据，实际获取到 {len(returned_camera_ids)} 个")

            except Exception as e:
                logger.error(f"解析相机图像响应失败: {e}")
                return {
                    "type": "find_person",
                    "success": False,
                    "obj_name": obj_name,
                    "result_msg": "相机图像数据解析失败"
                }

            # 将 CompressedImage 解码为 PIL.Image 列表 - 参考 ros2_data_source.py 的转换方法
            pil_images = []
            import numpy as np
            from PIL import Image
            import cv2

            for i, comp_img_msg in enumerate(rgb_images_compressed):
                try:
                    # CompressedImage 格式: {format: string, data: bytearray}
                    data = comp_img_msg.get("data", [])
                    img_format = comp_img_msg.get("format", "jpeg")

                    # 将压缩数据解码为 numpy 数组
                    np_arr = np.frombuffer(data, np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                    if cv_image is None:
                        logger.error(f"解码相机 {returned_camera_ids[i]} 的图像失败")
                        pil_images.append(None)
                        continue

                    # BGR 转 RGB (参考 ros2_data_source.py:180)
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                    pil_img = Image.fromarray(cv_image, mode='RGB')
                    pil_images.append(pil_img)
                    logger.info(f"成功解码相机 {returned_camera_ids[i]} 的图像")

                except Exception as e:
                    logger.error(f"解码相机 {returned_camera_ids[i]} 的图像失败: {e}")
                    pil_images.append(None)

            # 检查是否所有图像都成功解码
            if any(img is None for img in pil_images):
                logger.error("部分相机图像解码失败")
                return {
                    "type": "find_person",
                    "success": False,
                    "obj_name": obj_name,
                    "result_msg": "相机图像解码失败"
                }

            # Step 2: 调用人脸识别服务face_recognition
            logger.info("进行人脸识别...")
            t0 = time.time()

            # 构造人脸识别请求 - 直接使用 PIL.Image 列表，_call_ros2_service_async 会自动转换
            # Request格式: person_id (string), camera_ids (string[]), rgb_images (sensor_msgs/Image[])
            t1 = time.time()
            logger.info(f"构造请求耗时: {(t1-t0)*1000:.2f}ms")
            face_recognition_request = {
                "person_id": obj_name,
                "camera_ids": returned_camera_ids,
                "rgb_images": pil_images  # 直接传递 PIL.Image 列表
            }

            # 直接调用服务 - ROS2 spin 线程会正常处理回调，不会阻塞
            t2 = time.time()
            logger.info(f"准备服务调用耗时: {(t2-t1)*1000:.2f}ms")
            face_result = self._call_ros2_service_async(
                "/face_recognition",
                CallbackGroupType.FACE_RECOGNITION,
                "jqr_ros_msgs/srv/FaceRecognition",
                face_recognition_request,
                timeout=30.0  # 给足够长的时间
            )
            t3 = time.time()
            logger.info(f"人脸识别服务调用完成，总耗时: {(t3-t2)*1000:.2f}ms")

            if not face_result.get("success"):
                logger.error(f"人脸识别服务调用失败: {face_result.get('error_msg', '未知错误')}")
                return {
                    "type": "find_person",
                    "success": False,
                    "obj_name": obj_name,
                    "result_msg": "人脸识别服务调用失败"
                }

            # 解析人脸识别响应 - 使用异步服务返回的字典格式
            t4 = time.time()
            logger.info(f"解析响应耗时: {(t4-t3)*1000:.2f}ms")
            try:
                response_data = face_result.get("response", {})
                camera_id = response_data.get("camera_id", "")
                bbox = response_data.get("bbox", {})

                if camera_id:
                    # 找到目标人
                    result_msg = f"目标人{obj_name}在相机 {camera_id} 里找到"
                    logger.info(f"{result_msg}, bbox: {bbox}")

                    return {
                        "type": "find_person",
                        "success": True,
                        "obj_name": obj_name,
                        "result_msg": result_msg,
                    }
                else:
                    # 未找到目标人
                    result_msg = "未找到目标人"
                    logger.info(f"{result_msg}")

                    return {
                        "type": "find_person",
                        "success": False,
                        "obj_name": obj_name,
                        "result_msg": result_msg,
                    }

            except Exception as e:
                logger.error(f"解析人脸识别响应失败: {e}")
                return {
                    "type": "find_person",
                    "success": False,
                    "obj_name": obj_name,
                    "result_msg": "人脸识别响应解析失败"
                }

        except Exception as e:
            logger.error(f"找人过程异常: {e}")
            import traceback
            traceback.print_exc()
            return {
                "type": "find_person",
                "success": False,
                "obj_name": obj_name,
                "result_msg": f"找人过程异常: {str(e)}"
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
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
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
                    
                    success = (result_number == ResultCode.SUCCESS)
                    
                    result = {
                        "success": success,
                        "angle": robot_tilt_state,
                        "description": result_msg if success else f"获取失败: {result_msg}",
                        "result_number": result_number
                    }
                    
                    if success:
                        logger.info(f"获取机器人俯仰状态成功: {result}")
                    else:
                        logger.info(f"获取机器人俯仰状态失败: {result}")
                    
                    return result
                    
                except Exception as e:
                    return {
                        "success": False,
                        "angle": 0.0,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
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
                logger.info(f"获取机身升降状态: {result}")
                return result
            else:
                # 如果服务调用失败，返回默认值
                result = {
                    "success": False,
                }
                return result
        except Exception as e:
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
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
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
                    
                    success = (result_number == ResultCode.SUCCESS)
                    
                    result = {
                        "success": success,
                        "angle": angle,
                        "description": result_msg if success else f"设置失败: {result_msg}",
                        "result_number": result_number
                    }
                    
                    if success:
                        logger.info(f"设置屏幕俯仰角度成功: {result}")
                    else:
                        logger.info(f"设置屏幕俯仰角度失败: {result}")
                    return result
                    
                except (json.JSONDecodeError, KeyError) as e:
                    return {
                        "success": False,
                        "angle": angle,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
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
                return result
            else:
                # 检查响应是否为空或无效
                if not response or not response.strip():
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
                    
                    success = (result_number == ResultCode.SUCCESS)
                    
                    result = {
                        "success": success,
                        "angle": screen_tilt_state,
                        "description": result_msg if success else f"获取失败: {result_msg}",
                        "result_number": result_number
                    }

                    return result

                except Exception as e:
                    return {
                        "success": False,
                        "angle": 0.0,
                        "description": f"响应解析失败: {str(e)}"
                    }
        except Exception as e:
            return {
                "success": False,
                "angle": 0.0,
                "description": f"获取屏幕俯仰状态失败: {str(e)}"
            }

    # ======================
    # 头部电机控制相关接口（新头部样机）
    # ======================

    def set_head_motor_control(self, control_pitch: bool = False, pitch_angle: float = 0.0,
                                control_yaw: bool = False, yaw_angle: float = 0.0) -> Dict[str, Any]:
        """控制头部电机（新头部样机）

        Args:
            control_pitch (bool): 是否控制俯仰 (False=不控制, True=控制)
            pitch_angle (float): pitch角度（仅在control_pitch=True时有效）
            control_yaw (bool): 是否控制偏航 (False=不控制, True=控制)
            yaw_angle (float): yaw角度（仅在control_yaw=True时有效）

        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            # 调用ROS2Interface的发布方法
            result = self.publish_head_motor_control(
                control_pitch=control_pitch,
                pitch_angle=pitch_angle,
                control_yaw=control_yaw,
                yaw_angle=yaw_angle
            )

            if result.get("success"):
                logger.info(f"头部电机控制成功: pitch={pitch_angle if control_pitch else 'N/A'}, yaw={yaw_angle if control_yaw else 'N/A'}")
            else:
                logger.error(f"头部电机控制失败: {result.get('error_msg', '未知错误')}")

            return result

        except Exception as e:
            logger.error(f"头部电机控制失败: {e}")
            return {
                "success": False,
                "description": f"头部电机控制失败: {str(e)}"
            }

    # ======================
    # 组合电机控制接口（combine_motor_control）
    # ======================

    async def set_combine_motor_control(self, control_pitch: bool = False, pitch_angle: float = 0.0,
                                         control_yaw: bool = False, yaw_angle: float = 0.0,
                                         control_chassis_move: bool = False, chassis_offset: float = 0.0,
                                         control_chassis_rotate: bool = False, chassis_rotation: float = 0.0,
                                         speed_level: int = 0) -> Dict[str, Any]:
        """组合电机控制（通过WebSocket/USB调用，带反馈等待）

        Args:
            control_pitch (bool): 是否控制俯仰
            pitch_angle (float): pitch角的目标角度，单位：弧度
            control_yaw (bool): 是否控制偏航
            yaw_angle (float): yaw角的目标角度，单位：弧度
            control_chassis_move (bool): 是否控制底盘位移
            chassis_offset (float): 底盘位置偏移量，正值前进，负值后退，单位：米
            control_chassis_rotate (bool): 是否控制底盘旋转
            chassis_rotation (float): 底盘旋转偏移量，正值逆时针，负值顺时针，单位：弧度
            speed_level (int): 执行档位，0=低速，1=中速，2=快速

        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            # 启动组合电机监控（如果尚未启动）
            self.start_combine_motor_monitoring()

            task_id = self._next_motor_task_id()
            result = await self._execute_motor_step(
                task_id=task_id,
                control_pitch=control_pitch, pitch_angle=float(pitch_angle),
                control_yaw=control_yaw, yaw_angle=float(yaw_angle),
                control_chassis_move=control_chassis_move, chassis_offset=float(chassis_offset),
                control_chassis_rotate=control_chassis_rotate, chassis_rotation=float(chassis_rotation),
                speed_level=int(speed_level)
            )

            if result.get("success"):
                logger.info(f"组合电机控制成功: pitch={control_pitch}/{pitch_angle:.2f}, yaw={control_yaw}/{yaw_angle:.2f}, "
                            f"move={control_chassis_move}/{chassis_offset:.2f}, rotate={control_chassis_rotate}/{chassis_rotation:.2f}, speed={speed_level}")
            else:
                logger.error(f"组合电机控制失败: {result.get('error_msg', '未知错误')}")

            return result

        except Exception as e:
            logger.error(f"组合电机控制异常: {e}")
            return {
                "success": False,
                "error_msg": f"组合电机控制异常: {str(e)}"
            }

def battery_callback(msg):
    """电池电量回调函数 - 收到信息后立马通过USB串口发送

    Args:
        msg: 电池电量消息
    """
    try:
        # 更新电池电量
        robot_state.battery_level = msg.battery_power_state

        # 构造电池电量消息
        battery_message = {
            "type": "battery_update",
            "battery_level": robot_state.battery_level,
            "timestamp": int(time.time())
        }

        # 通过USB串口发送电池电量信息
        if robot_state.agent_instance and hasattr(robot_state.agent_instance, 'usb_manager'):
            try:
                # 尝试获取当前事件循环
                loop = asyncio.get_event_loop()
                if loop.is_running():
                    # 如果事件循环正在运行，创建任务
                    asyncio.create_task(robot_state.agent_instance.usb_manager.send_message(battery_message))
                else:
                    # 如果事件循环没有运行，使用run_until_complete
                    loop.run_until_complete(robot_state.agent_instance.usb_manager.send_message(battery_message))
            except RuntimeError:
                # 如果没有事件循环，创建一个新的
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(robot_state.agent_instance.usb_manager.send_message(battery_message))
        else:
            logger.info("USB管理器不可用,无法发送电池电量信息")
    except Exception as e:
        logger.error(f'处理电池电量回调时出错: {e}')

# 注释：不再需要单独的spin循环，使用主节点的spin


# ======================
# USB串口服务器管理器
# ======================

class USBCoordinateManager:
    """USB坐标管理器 - 串口可选，禁用时仅通过WebSocket通信"""

    def __init__(self, agent=None):
        self.agent = agent
        self.connected = False
        self.serial_enabled = config.USB_SERIAL_ENABLED and SERIAL_AVAILABLE

        if self.serial_enabled:
            self.serial_manager = SerialManager(port=USB_SERIAL_PORT, baudrate=USB_SERIAL_BAUDRATE)
        else:
            self.serial_manager = None
            logger.info("串口已禁用，仅通过WebSocket通信")
        

        
    async def initialize(self):
        """初始化USB串口连接（串口禁用时直接返回成功）"""
        if not self.serial_enabled:
            logger.info("串口已禁用，跳过USB初始化")
            return True
        try:
            # 连接到串口设备
            self.connected = await self.serial_manager.connect()
            if self.connected:
                # 添加消息回调
                self.serial_manager.add_callback(self._handle_received_message)
                # 开始接收数据
                self.serial_manager.start_receiving()
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
            logger.info(f"接收到USB消息: {message}")

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
                logger.warning("Agent或handle_client_message方法不可用")
        except Exception as e:
            logger.error(f"处理USB消息失败: {e}")

    def _process_message_in_thread(self, message: Dict[str, Any]):
        """在独立线程中处理消息 - 使用线程独立的事件循环和WebSocket连接"""
        try:
            # 检查agent是否存在
            if not self.agent:
                logger.error("Agent不可用，无法处理消息")
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
            logger.error(f"线程处理消息失败: {e}")
    
    async def send_message(self, message: Dict[Any, Any]) -> bool:
        """发送消息到客户端（串口禁用时通过WebSocket发送）"""
        if not self.serial_enabled:
            # 串口禁用，尝试通过WebSocket广播
            if self.agent and hasattr(self.agent, 'websocket_server'):
                try:
                    await self.agent.websocket_server.broadcast_message(message)
                    return True
                except Exception as e:
                    logger.warning(f"WebSocket广播失败: {e}")
            return False
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
            if self.serial_manager:
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
    """智能机器人Agent - USB串口版本 (集成ReAct框架)"""
    
    def __init__(self):
        self.ros2_interface = ROS2Interface()

        # 创建USB串口通信管理器
        self.usb_manager = USBCoordinateManager(self)

        # 创建WebSocket控制服务器（局域网控制接口）
        self.websocket_server = WebSocketControlServer(
            agent=self,
            host=config.WEBSOCKET_HOST,  # 监听所有网卡
            port=config.WEBSOCKET_PORT   # WebSocket端口
        )

        # 任务中断标志
        self._task_interrupted = False

        # 事件循环引用（用于从其他线程调度任务）
        self.event_loop: Optional[asyncio.AbstractEventLoop] = None
        # 线程本地存储，用于隔离WebSocket连接
        self._thread_local = threading.local()

        # 本地模型连接相关
        self.local_model_websocket = None
        self.local_model_connected = False
        self.local_model_uri = config.LOCAL_MODEL_URI
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
        self.memory = AgentMemory(max_history=config.AGENT_MEMORY_SIZE)  # Agent记忆系统
        self.react_enabled = True  # 是否启用ReAct模式
        self.max_react_iterations = config.MAX_REACT_ITERATIONS  # 最大思考-行动循环次数
        self.current_react_task = None  # 当前ReAct任务
        
        # 已知的任务类型列表（可直接执行，无需LLM）
        self.known_task_types = {
            "find_object", "find_person", "go_to_object", "go_find_person", "follow_person",
            "back_to_last_position", "go_to_door", "stop_follow", "stop_navigate", "stop_move",
            "get_move_mode", "get_medicine_box_state", "set_medicine_box_switch",
            "get_robot_rise_state", "set_robot_rise_jqr",
            "get_robot_tilt_state", "set_robot_tilt_jqr",
            "get_screen_tilt_state", "set_screen_tilt_jqr",
            "set_laser_pointer", "get_laser_pointer_state",
            "set_rgb", "get_rgb_light_strip_state", "delete_person",
            "set_head_motor_control", "set_combine_motor_control"
        }
    
    async def initialize(self):
        """初始化agent"""
        try:
            # 设置全局实例引用
            robot_state.agent_instance = self
            
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

                # 启动组合电机控制结果监控
                combine_motor_success = self.ros2_interface.start_combine_motor_monitoring()
                if combine_motor_success:
                    logger.info("组合电机控制结果监控已启动")
                else:
                    logger.warning("组合电机控制结果监控启动失败")
            
            # 初始化USB串口通信
            usb_connected = await self.usb_manager.initialize()
            if not usb_connected:
                logger.warning("USB串口连接失败，将仅通过WebSocket通信")

            # 启动WebSocket控制服务器
            websocket_started = self.websocket_server.start()
            if websocket_started:
                logger.info("WebSocket控制服务器启动成功")
            else:
                logger.warning("WebSocket控制服务器启动失败")
            
            # 启动消息处理循环
            self._running = True
            asyncio.create_task(self._message_processor())

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
                    # 创建后台任务执行，不等待完成
                    asyncio.create_task(self._execute_task_async(task_to_execute))
                    # 立即返回，不等待任务完成
                    return
                else:
                    # 未知任务类型，发给LLM处理
                    logger.info(f"未知任务类型 '{task_type}'，发送给LLM分析")
                    user_prompt = f"执行任务: {task_type}，参数: {task_params}"
                    llm_result = await self.analyze_with_llm(user_prompt, task_type)
                    result = llm_result
                    await self.send_response_to_client(result)
                    return
            
            # 2. 自然语言任务（不含type字段）
            user_prompt = None
            
            # 情况A: 消息本身就是字符串（自然语言内容）
            if isinstance(message, str):
                user_prompt = message
                logger.info(f"收到自然语言字符串任务: {user_prompt}")
            
            # 情况B: 其他字典格式（不含type，提取第一个字符串值作为user_prompt）
            elif isinstance(message, dict):
                for key, value in message.items():
                    if isinstance(value, str) and len(value.strip()) > 0:
                        user_prompt = value
                        logger.info(f"从字段 '{key}' 中提取自然语言任务: {user_prompt}")
                        break
            
            # 发送自然语言任务给LLM分析
            if user_prompt:
                logger.info("发送自然语言任务给LLM分析")
                llm_result = await self.analyze_with_llm(user_prompt, "talk")
                await self.send_response_to_client(llm_result)
                return
        
        except Exception as e:
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

            # 执行任务
            result = await self.execute_task(task)

            # 记录到记忆
            self.memory.add_task(task, result)

            # 发送响应到客户端
            await self.send_response_to_client(result)
            logger.info(f"任务执行完成: {task_type}")
        except Exception as e:
            logger.error(f"后台任务执行异常: {task_type}, 错误: {e}")

    async def _execute_task_concurrent(self, message: Dict[str, Any]):
        """并发执行消息处理（直接执行任务，不经过handle_client_message）
        
        这个方法绕过 handle_client_message，直接处理消息并执行任务，
        避免消息在事件循环中排队等待
        """
        try:
            # 重置任务状态
            self.memory.clear_episode()

            # 只处理已知任务类型
            if isinstance(message, dict) and "type" in message:
                task_type = message.get("type", "")

                # 检查是否为已知任务类型
                if task_type in self.known_task_types:
                    result = await self.execute_task(message)
                    # 记录到记忆
                    self.memory.add_task(message, result)
                    # 发送响应
                    await self.send_response_to_client(result)
                else:
                    # 未知任务类型，调用 handle_client_message
                    await self.handle_client_message(message)

        except Exception as e:
            logger.error(f"并发处理消息失败: {e}")

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
        logger.info("===== 开始LLM分析 =====")
        logger.info(f"任务类型: {task_type}")
        logger.info(f"用户指令: {user_prompt}")
        
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
- 如果是问答类请求（如问时间、天气、打招呼等），type使用"default"，params中填写"response"字段作为回答内容
- 必须严格返回有效JSON格式，不要包含其他文字
"""
        
        # 调用LLM
        llm_response = await self._call_llm_for_analysis(full_prompt)
        logger.info(f"LLM响应: {llm_response}")
        
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
            if result_type == "default":
                # 交互问答类，直接返回回答
                logger.info("交互问答类，直接返回回答")
                response_content = result_params.get("response", llm_response)
                return {
                    "type": task_type,
                    "success": True,
                    "result": response_content,
                    "description": response_content
                }
            elif result_type in self.known_task_types:
                # 已知任务类型，执行任务
                logger.info(f"执行已知任务: {result_type}")
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
                logger.warning(f"未知任务类型: {result_type}")
                return {
                    "type": task_type,
                    "success": False,
                    "error_msg": f"未知任务类型: {result_type}"
                }
                
        except json.JSONDecodeError as e:
            logger.error(f"LLM响应JSON解析失败: {e}, 原始响应: {llm_response}")
            # 如果解析失败，尝试直接作为自然语言回复
            return {
                "type": task_type,
                "success": True,
                "result": llm_response,
                "description": llm_response
            }
        except Exception as e:
            logger.error(f"处理LLM响应失败: {e}")
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

        system_prompt = f"""你是一个智能机器人Agent的助手，负责分析用户指令并决定如何响应，当用户指令中包含多个任务请求比如找多个物品时，创建任务列表并返回。下面是

Agent已知的能力（可用工具）:
"""

        # 添加每个工具的说明
        tool_descriptions = {
            "find_person": "静态查找指定人员",
            "go_to_object": "导航到指定对象位置",
            "go_find_person": "去寻找指定的人",
            "follow_person": "跟随指定的人",
            "back_to_last_position": "返回到初始位置",
            "go_to_door": "导航到门口位置",
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
            "set_rgb": "设置RGB灯",
            "get_rgb_light_strip_state": "获取RGB灯光状态",
            "delete_person": "删除指定人脸人员"
        }

        for tool in available_tools:
            desc = tool_descriptions.get(tool, "未知工具")
            system_prompt += f"- {tool}: {desc}\n"

        system_prompt += """
其他说明:
- default: 用于直接回答用户的问题或进行对话（如打招呼、问答、闲聊等）
- 如果用户指令不匹配上述任何工具，请使用default直接回复
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
            # 构造LLM请求格式 - 按照本地模型期望的messages数组格式
            messages = [
                {
                    "role": "system",
                    "content": [
                        {"type": "text", "text": "你是一个智能机器人任务分析助手，负责分析用户指令并返回标准JSON格式的响应。"},
                    ],
                },
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                    ],
                }
            ]

            llm_request = {
                "type": "talk",
                "message": json.dumps(messages),
            }

            response = await self.send_to_local_model(llm_request)
            
            # 提取LLM的响应内容
            if response and "result" in response:
                return response["result"]
            elif response and "content" in response:
                return response["content"]
            elif isinstance(response, str):
                return response
            else:
                logger.warning(f"LLM响应格式异常: {response}")
                # 返回默认的default
                return json.dumps({"type": "default", "params": {"response": "抱歉，我无法理解您的指令"}})
                
        except Exception as e:
            logger.error(f"调用LLM失败: {e}")
            # 返回默认的default
            return json.dumps({"type": "default", "params": {"response": f"分析失败: {str(e)}"}})

    async def _decompose_find_object_task(self, user_prompt: str) -> List[Dict[str, Any]]:
        """
        使用LLM拆解找物任务，判断是否需要执行多个go_to_object任务

        Args:
            user_prompt (str): 用户原始指令

        Returns:
            List[Dict[str, Any]]: 拆解后的任务列表，每个任务格式为 {"type": "go_to_object", "params": {...}}
        """
        system_prompt = f"""你是一个智能机器人任务拆解助手，负责分析用户的找物指令，判断需要查找多少个物体以及每个物体的名称,如果用户指令输入有对于物品的描述，拆解后物体名称需要包含它的描述。

任务规则:
1. 分析用户指令中提到的所有需要查找的物体
2. 如果涉及一个或多个物体，为每个物体创建一个go_to_object任务
3. go_to_object任务的params中需要包含obj_name字段

返回格式:
返回任务列表，格式如下:
[
    {{"type": "go_to_object", "params": {{"obj_name": "物体1名称"}}}},
    {{"type": "go_to_object", "params": {{"obj_name": "物体2名称"}}}}
]

注意:
- 严格按照JSON格式返回，不要包含任何解释性文字
- 每个任务都必须有type和params字段
- 如果用户的指令中指定了数量（如"找两个遥控器"），需要创建对应数量的任务
"""

        try:
            logger.info(f"开始任务拆解: {user_prompt}")

            # 构造消息数组格式（使用 OpenAI 兼容格式）
            messages = [
                {
                    "role": "system",
                    "content": system_prompt,
                },
                {
                    "role": "user",
                    "content": f"用户指令: {user_prompt}\n\n请分析并返回需要执行的任务列表（JSON格式）:",
                }
            ]

            # 使用 OpenAI 客户端调用本地模型
            completion = self.openai_client.chat.completions.create(
                model=self.openai_model,
                messages=messages,
            )
            llm_response = completion.choices[0].message.content

            logger.info(f"LLM响应: {llm_response}")

            # 解析LLM响应
            task_data = json.loads(llm_response)

            # 检查返回格式
            if isinstance(task_data, list):
                return task_data
            elif isinstance(task_data, dict):
                # 如果返回的是单个任务，包装成列表
                if "type" in task_data:
                    return [task_data]
                # 如果是natural_response或其他类型，返回空列表
                logger.info("LLM返回自然语言响应，无需拆解")
                return []
            else:
                logger.warning(f"LLM返回格式异常: {task_data}")
                return []

        except json.JSONDecodeError as e:
            logger.error(f"JSON解析失败: {e}, 原始响应: {llm_response}")
            return []
        except Exception as e:
            logger.error(f"任务拆解失败: {e}")
            import traceback
            traceback.print_exc()
            return []
    
    async def execute_task(self, task: Dict[str, Any]) -> Dict[str, Any]:
        """执行单个任务
        
        Args:
            task (Dict[str, Any]): 任务字典，包含任务类型和参数
            
        Returns:
            Dict[str, Any]: 任务执行结果
        """
        task_type = task.get("type")
        task_params = task.get("params", {})

        if not task_type:
            return {"type": task_type or "unknown", "success": False, "error_msg": "任务类型为空"}
        
        # 检查任务类型并发控制（仅串口模式下）
        has_lock = False
        if self.usb_manager.serial_manager:
            success, error_msg = self.usb_manager.serial_manager.acquire_task_type_lock(task_type)
            if not success:
                return {
                    "type": task_type,
                    "success": False,
                    "error_msg": error_msg
                }
            has_lock = True

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
            if has_lock:
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
            return await self.find_object(params.get("obj_name", ""), params.get("user_prompt", ""))
        elif task_type == "find_person":
            return self.ros2_interface.find_person(params.get("obj_name", ""), params.get("user_prompt", ""))
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
        elif task_type == "go_to_door":
                result = await self.go_to_door(**params)
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
        elif task_type == "delete_person" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.delete_person(params.get("obj_name", ""))
            result["type"] = task_type
            return result
        elif task_type == "set_rgb" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.set_rgb(**params)
            result["type"] = task_type
            return result
        elif task_type == "get_rgb_light_strip_state" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.get_rgb_light_strip_state()
            result["type"] = task_type
            return result
        # 头部电机控制（新头部样机）
        elif task_type == "set_head_motor_control" and hasattr(self, 'ros2_interface'):
            result = self.ros2_interface.set_head_motor_control(**params)
            result["type"] = task_type
            return result
        # 组合电机控制（combine_motor_control）
        elif task_type == "set_combine_motor_control" and hasattr(self, 'ros2_interface'):
            result = await self.ros2_interface.set_combine_motor_control(**params)
            result["type"] = task_type
            result.pop("result", None)
            return result
        # 交互场景
        elif task_type == "user_position_tracking":
            result = await self.ros2_interface.user_position_tracking(params)
            result["type"] = task_type
            result.pop("result", None)
            return result
        elif task_type == "patrol_table_inspection":
            result = await self.ros2_interface.patrol_table_inspection(params)
            result["type"] = task_type
            result.pop("result", None)
            return result
        elif task_type == "wake_head_range":
            result = await self.ros2_interface.wake_head_range(params)
            result["type"] = task_type
            result.pop("result", None)
            return result
        elif task_type == "wake_beyond_head_range":
            result = await self.ros2_interface.wake_beyond_head_range(params)
            result["type"] = task_type
            result.pop("result", None)
            return result
        elif task_type == "wake_side_moving":
            result = await self.ros2_interface.wake_side_moving(params)
            result["type"] = task_type
            result.pop("result", None)
            return result
        elif task_type == "wake_back_moving":
            result = await self.ros2_interface.wake_back_moving(params)
            result["type"] = task_type
            result.pop("result", None)
            return result
        elif task_type == "obstacle_avoidance_turn":
            result = await self.ros2_interface.obstacle_avoidance_turn(params)
            result["type"] = task_type
            result.pop("result", None)
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
            with get_db_connection() as conn:
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
            if row:
                logger.info(f"Found object {obj_name} with id {row[0]} at location ({row[2]}, {row[3]})")
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
                logger.info(f"Object {obj_name} not found in database")
        except sqlite3.Error as e:
            logger.error(f"[ERROR] DB query failed: {e}")
        return None
    async def find_object(self, obj_name: str, user_prompt: str = "") -> Dict[str, Any]:
        """
        查找物品的位置信息，按照ASM→DB→探索的优先级执行，并支持多任务拆解

        Args:
            obj_name (str): 物品名称
            user_prompt (str): 用户原始指令，用于任务拆解

        Returns:
            Dict[str, Any]: 工具执行结果
        """
        logger.info(f"开始查找物品/人员: {obj_name}, 用户指令: {user_prompt}")

        # 存储初始查询结果
        initial_find_result = None
        try:
            # Step 1: ASM查询（最高优先级）
            asm_res = self.query_asm_object(obj_name)
            if asm_res:
                # 打印asm_res
                print(asm_res)
                loc = asm_res["location"]
                logger.info(f"在ASM中找到 {obj_name} 位置: ({loc['x']}, {loc['y']})")

                # ASM找到：返回位置信息
                initial_find_result = {
                    "type": "find_object",
                    "success": True,
                    "pixel_position": asm_res.get("pixel_position", []),
                    "position_description": asm_res.get("object_description", "")
                }

            # Step 2: DB查询（如果ASM没有找到）
            if not initial_find_result:
                db_res = self.query_history_db(obj_name)
                if db_res:
                    logger.info(f"在DB中找到 {obj_name} 记录，时间: {db_res['last_show_time']}")
                    initial_find_result = {
                        "type": "find_object",
                        "success": True,
                        "pixel_position": [db_res["world_x"], db_res["world_y"]],
                        "position_description": db_res["object_description"]
                    }
                else:
                    # DB也没有找到：返回失败结果
                    initial_find_result = {
                        "type": "find_object",
                        "success": False,
                        "pixel_position": None,
                        "position_description": None
                    }

            # Step 3: 先发送找物结果给客户端
            if initial_find_result:
                await self.send_response_to_client(initial_find_result)
                logger.info("已发送找物结果给客户端")

            # Step 4: 使用LLM对user_prompt进行任务拆解并执行
            if user_prompt:
                logger.info(f"开始对用户指令进行任务拆解: {user_prompt}")
                task_list = await self._decompose_find_object_task(user_prompt)
                success_count = 0
                response = {}
                if task_list and len(task_list) > 0:
                    logger.info(f"LLM拆解出 {len(task_list)} 个子任务，开始执行")
                    # 依次执行所有go_to_object任务
                    for task in task_list:
                        if task.get("type") == "go_to_object":
                            task_params = task.get("params", {})
                            obj_name_sub = task_params.get("obj_name", obj_name)
                            logger.info(f"执行子任务: go_to_object {obj_name_sub}")
                            response = await self.go_to_object(obj_name_sub, task_params.get("pixel_position"))
                            if (response.get("success") == True):
                                success_count += 1
                            await self.send_response_to_client(response)
                    if success_count == len(task_list):
                        if initial_find_result:
                            initial_find_result["success"] = True
                            initial_find_result["position_description"] = response.get("error_msg", "")
                        logger.info("所有找物子任务执行成功")
                    else:
                        logger.info(f"找物子任务执行完成，成功 {success_count} 个，失败 {len(task_list) - success_count} 个")
                else:
                    logger.info("LLM未拆解出子任务")

        except Exception as e:
            logger.error(f"找物过程异常: {e}")
            initial_find_result = {
                "type": "find_object",
                "success": False,
                "pixel_position": None,
                "position_description": None,
                "error_msg": str(e)
            }
        # 如果没有user_prompt或没有执行子任务，返回initial_find_result
        return initial_find_result
    
    async def go_to_object(self, obj_name: str, pixel_position: Optional[List[float]] = None) -> Dict[str, Any]:
        """导航到物体位置"""
        try:
            # 构造符合导航服务期望的数据格式
            model_data = {
                "type": "go_to_object",
                "user_prompt": f"去找{obj_name}",
                "params": {
                    "obj_name": obj_name,
                    "pixel_position": pixel_position
                }
            }

            result_msg = {
                "type": "go_to_object",
                "success": False,
                "obj_name": obj_name,
                "error_msg": ""
            }

            response = await self.send_to_local_model(model_data)

            if response and response.get("error_msg") == "无法连接到本地模型服务器":
                result_msg["err_msg"] = "无法连接到本地模型服务器"
                return result_msg

            result_msg["success"] = response.get("success", False)
            result_msg["error_msg"] = response.get("error_msg", "")
            return result_msg

        except Exception as e:
            logger.error(f"导航到物体失败: {e}")
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
            model_data = {
                "type": "follow_person",
                "user_prompt": location_info or "跟随人员"
            }
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
        except Exception as e:
            logger.error(f"导航到物体失败: {e}")
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
            model_data = {
                "type": "go_to_person",
                "user_prompt": user_prompt,
                "person_id": obj_name
            }
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

                except Exception as e:
                    logger.warning(f"[STOP_MOVE] 发送停止命令到本地模型失败: {e}")
            else:
                logger.info("[STOP_MOVE] 当前没有活跃的导航任务")
            
            # 2. 在/cmd_vel话题上发一次0
            if ROS2_AVAILABLE:
                try:
                    # 使用ros2 topic publish命令发布速度为0的消息
                    cmd = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
                    os.system(cmd)

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
            # 确保位置订阅已启动
            if hasattr(self, 'ros2_interface') and not self.ros2_interface.position_subscribed:
                self.ros2_interface.subscribe_robot_position()

            # 记录当前位置
            if hasattr(self, 'ros2_interface'):
                success = self.ros2_interface.record_current_position()
                return success
            else:
                logger.warning("ROS2接口不可用")
                return False
        except Exception as e:
            logger.error(f"记录位置失败: {e}")
            return False
    
    async def back_to_last_position(self) -> Dict[str, Any]:
        """
        返回到初始位置
        Returns:
            Dict[str, Any]: 返回导航结果
        """
        try:
            # 获取初始位置
            initial_position = self.ros2_interface.get_initial_position()
            if not initial_position:
                logger.warning("没有记录的初始位置信息")
                return {
                    "success": False,
                    "error_msg": "没有记录的初始位置信息，无法返回"
                }
            logger.info(f"返回到初始位置: {initial_position['position']}")
            # 调用导航功能
            result = self.ros2_interface.navigate_to_position(initial_position)
            # 确保返回结果包含type字段
            result["type"] = "back_to_last_position"
            if result["success"]:
                logger.info("成功返回到初始位置")
            else:
                logger.info(f"返回初始位置失败: {result.get('error_msg', '未知错误')}")
            return result
        except Exception as e:
            logger.error(f"返回初始位置时出错: {e}")
            return {
                "success": False,
                "error_msg": f"返回位置失败: {str(e)}"
            }

    async def go_to_door(self) -> Dict[str, Any]:
        """
        导航到门口位置（从 position.txt 读取）
        Returns:
            Dict[str, Any]: 返回导航结果
        """
        try:
            logger.info("开始导航到门口位置")
            # 读取 position.txt 文件
            position_file = "/home/sunrise/welcome_position.txt"
            if not os.path.exists(position_file):
                logger.warning("position.txt 文件不存在")
                return {
                    "success": False,
                    "error_msg": f"position.txt 文件不存在"
                }

            with open(position_file, 'r', encoding='utf-8') as f:
                content = f.read().strip()

            # 解析位置数据（空格分隔格式: x y z qx qy qz qw）
            try:
                parts = content.split()
                if len(parts) != 7:
                    logger.error(f"position.txt 格式错误，需要6个浮点数，实际得到 {len(parts)} 个")
                    return {
                        "success": False,
                        "error_msg": f"position.txt 格式错误，需要7个浮点数 (x y z qx qy qz qw)"
                    }

                # 构造位置字典
                door_position = {
                    "position": {
                        "x": float(parts[0]),
                        "y": float(parts[1]),
                        "z": float(parts[2])
                    },
                    "orientation": {
                        "x": float(parts[3]),
                        "y": float(parts[4]),
                        "z": float(parts[5]),
                        "w": float(parts[6])  # 如果没有提供w，默认为1.0
                    }
                }

            except ValueError as e:
                logger.error(f"解析 position.txt 数值失败: {e}")
                return {
                    "success": False,
                    "error_msg": f"position.txt 数值格式错误: {str(e)}"
                }

            logger.info(f"门口位置: {door_position['position']}, 方向: {door_position['orientation']}")

            # 调用导航功能
            result = self.ros2_interface.navigate_to_position(door_position)
            # 确保返回结果包含type字段
            result["type"] = "go_to_door"

            if result["success"]:
                logger.info("成功导航到门口")
            else:
                logger.info(f"导航到门口失败: {result.get('error_msg', '未知错误')}")

            return result
        except Exception as e:
            logger.error(f"导航到门口时出错: {e}")
            return {
                "success": False,
                "error_msg": f"导航到门口失败: {str(e)}"
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
            logger.info(f"成功建立线程本地连接: {self.local_model_uri}")
            return True
        except Exception as e:
            logger.error(f"建立线程本地连接失败: {e}")
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
                
                # 建立新连接（禁用 ping keepalive，因为我们会持续接收数据）
                connect_func = getattr(websockets, 'connect')
                thread_local.websocket = await connect_func(
                    self.local_model_uri,
                    ping_interval=None,  # 禁用自动ping
                    ping_timeout=None,     # 禁用ping超时
                    close_timeout=10.0       # 关闭超时10秒
                )
                logger.info(f"成功建立线程本地连接: {self.local_model_uri}")
            except Exception as e:
                logger.error(f"创建线程本地连接失败: {e}")
                return {"success": False, "error_msg": f"无法连接到本地模型服务器: {str(e)}"}
        
        websocket = thread_local.websocket
        
        try:
            # 发送数据
            message_str = json.dumps(model_data, ensure_ascii=False)
            await websocket.send(message_str)

            intermediate_data = {
                "type": "",
                "command": ""
            }
            
            # 持续接收响应，直到收到最终结果
            final_response = None
            while self._running and websocket is not None:
                try:
                    response_str = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                    try:
                        response_data = json.loads(response_str)
                    except json.JSONDecodeError as e:
                        logger.error(f"JSON解析失败: {e}, 原始数据: {response_str}")
                        # 尝试将非JSON响应作为最终结果返回
                        final_response = {"success": False, "error_msg": f"本地模型返回非JSON数据: {response_str}"}
                        break

                    # 检查是否是最终结果（包含success字段或result字段）
                    if ("success" in response_data or "result" in response_data or "answer" in response_data) and "command" not in response_data:
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
                except asyncio.TimeoutError:
                    # 超时检查运行状态
                    continue
                except Exception as e:
                    error_str = str(e)
                    logger.error(f"接收本地模型响应时出错: {e}")

                    # 检查连接是否已关闭
                    if "connection" in error_str.lower() or "closed" in error_str.lower():
                        logger.warning("检测到WebSocket连接已关闭，清理连接并退出循环")
                        # 清理无效连接（关键：设置为None，避免下次复用）
                        if hasattr(thread_local, 'websocket') and thread_local.websocket is not None:
                            try:
                                await thread_local.websocket.close()
                            except Exception:
                                pass
                            thread_local.websocket = None  # 必须置None
                        # 退出循环
                        break

                    # 其他错误，检查是否有部分响应后退出
                    if final_response is None:
                        final_response = {"success": False, "error_msg": f"接收响应失败: {error_str}"}
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
    
    def get_websocket_stats(self) -> Dict[str, Any]:
        """获取WebSocket服务器统计信息
        
        Returns:
            Dict[str, Any]: WebSocket服务器状态
        """
        if hasattr(self, 'websocket_server'):
            return self.websocket_server.get_stats()
        return {
            "running": False,
            "error_msg": "WebSocket服务器未初始化"
        }
    
    def cleanup(self):
        """清理资源"""
        try:
            # 设置退出标志
            self._running = False
            
            # 停止WebSocket控制服务器
            if hasattr(self, 'websocket_server'):
                self.websocket_server.stop()
            
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
    print(f"Smart Robot Agent v{AGENT_VERSION} is running...")

    # 保存事件循环引用
    loop = asyncio.get_running_loop()

    # 初始化数据库
    init_database()

    # 修复ASM JSON文件
    fix_asm_json_format()

    # 创建智能机器人Agent
    agent = SmartRobotAgent()
    agent.event_loop = loop  # 保存事件循环引用
    robot_state.agent_instance = agent
    # 初始化agent
    try:
        success = await agent.initialize()
        if not success:
            logger.error("SmartRobotAgent初始化失败，退出程序")
            return

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


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"程序运行出错: {e}")


