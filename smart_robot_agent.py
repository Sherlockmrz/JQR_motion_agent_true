# -*- coding: utf-8 -*-
"""A simplified smart robot agent implementation without AgentScope dependencies."""

import os
import json
import sqlite3
from datetime import datetime
import threading
import time
import asyncio
from websockets.exceptions import ConnectionClosedError, ConnectionClosedOK
from websockets.server import serve
import logging
from typing import Optional, Dict, Any, List

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ROS2 可用性标志
ROS2_AVAILABLE = False
rclpy = None
Node = None
BatteryLevel = None

# 尝试导入rclpy，如果不存在则忽略
try:
    import rclpy
    from rclpy.node import Node
    from jqr_ros_msgs.msg import BatteryLevel
    ROS2_AVAILABLE = True
    logger.info("ROS2 rclpy 导入成功")
except ImportError as e:
    logger.warning(f"ROS2 rclpy 不可用: {e}")

# 尝试导入cv2，如果不存在则忽略
try:
    import cv2
except ImportError:
    cv2 = None
    logger.warning("cv2模块未安装，视频处理功能将不可用")

# ======================
# 配置
# ======================
ASM_JSON_PATH = "asm_data.json"
VIDEO_BASE_DIR = "/tmp/videos/"
DB_PATH = "/tmp/history.db"

os.makedirs(VIDEO_BASE_DIR, exist_ok=True)
os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)

# WebSocket配置
WEBSOCKET_HOST = "localhost"
WEBSOCKET_PORT = 8768

# ======================
# JSON修复函数
# ======================

def fix_asm_json_format():
    """修复ASM JSON文件格式 - 只修复格式问题，不修改数据内容"""
    if not os.path.exists(ASM_JSON_PATH):
        logger.warning(f"ASM JSON文件不存在: {ASM_JSON_PATH}")
        return False
        
    # 检查文件是否为空
    if os.path.getsize(ASM_JSON_PATH) == 0:
        logger.warning(f"ASM JSON文件为空: {ASM_JSON_PATH}")
        return False
        
    try:
        # 读取原始文件内容
        with open(ASM_JSON_PATH, 'r', encoding='utf-8') as f:
            original_content = f.read()
            
        # 检查是否已经是正确的JSON格式
        try:
            original_data = json.loads(original_content)
            logger.info("ASM JSON文件已经是正确的格式，无需修复")
            return True  # 格式正确，直接返回，不重写文件
        except json.JSONDecodeError as e:
            logger.info(f"ASM JSON文件格式不正确，需要修复: {e}")
            
        # 备份原始内容
        backup_content = original_content
        
        # 保守的格式修复 - 只修复最基础的格式问题
        fixed_content = original_content.strip()
        
        # 1. 确保以{开头
        if not fixed_content.startswith('{'):
            # 检查是否以"objects":开头
            if fixed_content.startswith('"objects":'):
                fixed_content = "{" + fixed_content
            else:
                # 保守处理：如果不知道如何修复，不修改内容
                logger.warning("无法确定如何修复文件格式，保持原样")
                return False
                
        # 2. 确保以}结尾
        if not fixed_content.endswith('}'):
            # 移除末尾的逗号
            if fixed_content.endswith(','):
                fixed_content = fixed_content[:-1] + "}"
            else:
                fixed_content = fixed_content + "}"
                
        # 3. 尝试解析修复后的内容
        try:
            data = json.loads(fixed_content)
            # 验证修复后的数据是否包含原始内容
            if "objects" in data and isinstance(data["objects"], list):
                # 只保存修复后的格式，不修改数据内容
                with open(ASM_JSON_PATH, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)
                logger.info(f"ASM JSON文件格式已修复，数据内容保持不变")
                return True
            else:
                logger.warning("修复后缺少objects数组，恢复原始内容")
                # 恢复原始内容
                with open(ASM_JSON_PATH, 'w', encoding='utf-8') as f:
                    f.write(backup_content)
                return False
                
        except json.JSONDecodeError as e:
            logger.error(f"修复后仍然无法解析: {e}")
            # 恢复原始内容
            with open(ASM_JSON_PATH, 'w', encoding='utf-8') as f:
                f.write(backup_content)
            return False
            
    except Exception as e:
        logger.error(f"处理ASM JSON文件时出错: {e}")
        return False

# ======================
# 工具函数：外部系统模拟
# ======================
def stop_following() -> bool:
    """模拟停止跟随"""
    print("[ROS2] Stopping follow")
    return True

def stop_navigation() -> bool:
    """模拟停止导航"""
    print("[ROS2] Stopping navigation")
    return True

def notify_navigation_model_stop_following():
    """通知导航模型停止跟随"""
    print("[NAV_MODEL] Stop-following signal sent")

# ======================
# 视频与图像处理
# ======================

def cut_video_6s(video_path: str, timestamp: str, output_dir: str) -> Optional[str]:
    """剪切视频前后3秒（总共6秒片段）"""
    try:
        # 使用ffmpeg剪切真实视频片段
        dt = datetime.fromisoformat(timestamp.replace("Z", "+00:00"))
        output_name = f"cut_6s_{dt.strftime('%Y%m%d_%H%M%S')}.mp4"
        output_path = os.path.join(output_dir, output_name)
        
        # 检查cv2是否可用
        if cv2 is None:
            logger.warning("cv2模块不可用，跳过视频时长获取")
            # 使用默认值
            video_duration = 100.0  # 默认100秒
        else:
            # 获取视频时长
            cap = cv2.VideoCapture(video_path)
            fps = cap.get(cv2.CAP_PROP_FPS)
            frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            video_duration = frame_count / fps if fps > 0 else 100.0  # 默认100秒
            cap.release()
        
        # 计算剪切时间（前后3秒）
        target_time = dt.timestamp()
        start_time = max(0, target_time - 3)  # 前3秒
        end_time = min(target_time + 3, video_duration)  # 后3秒，但不超过视频总长度
        
        # 确保开始时间小于结束时间
        if start_time >= end_time:
            start_time = max(0, end_time - 6)  # 最多回退6秒
        
        # 使用ffmpeg剪切视频
        cmd = f"ffmpeg -i '{video_path}' -ss {start_time} -to {end_time} -c copy '{output_path}' -y"
        logger.info(f"Executing ffmpeg command: {cmd}")
        result = os.system(cmd)
        logger.info(f"FFmpeg command result: {result}")
        
        if os.path.exists(output_path) and os.path.getsize(output_path) > 1000:
            logger.info(f"Video cut successfully: {output_path}")
            return output_path
        else:
            logger.error("Failed to cut video or video file too small")
            # 如果切片失败，返回原始视频路径
            return video_path
    except Exception as e:
        logger.error(f"[ERROR] Video cut failed: {e}")
        # 如果切片失败，返回原始视频路径
        return video_path

# ======================
# Qwen-VL API调用
# ======================

def call_qwen_vl_api_with_video(prompt: str, video_path: str) -> Optional[str]:
    """调用Qwen-VL API处理视频并返回结果"""
    try:
        # 检查视频文件是否存在
        if not os.path.exists(video_path):
            logger.error(f"视频文件不存在: {video_path}")
            return None
            
        # 这里应该是实际的API调用代码
        # 由于我们没有实际的API密钥和端点，暂时返回模拟结果
        logger.info(f"调用Qwen-VL API处理视频: {video_path}")
        logger.info(f"提示词: {prompt}")
        
        # 模拟API响应
        # 在实际实现中，这里会调用真实的Qwen-VL API
        simulated_responses = {
            "遥控器": "遥控器在电视柜上。",
            "钥匙": "钥匙在玄关柜子上。",
            "书本": "书本在书房桌子上。",
            "钱包": "钱包在卧室床头柜上。",
            "电视机": "电视机在客厅墙上。",
            "水杯": "水杯在厨房台面上。",
            "剪刀": "剪刀在工作台上。"
        }
        
        # 从prompt中提取物品名称
        for item in simulated_responses.keys():
            if item in prompt:
                response = simulated_responses[item]
                logger.info(f"模拟API响应: {response}")
                return response
                
        # 默认响应
        response = "物品在附近区域。"
        logger.info(f"默认API响应: {response}")
        return response
        
    except Exception as e:
        logger.error(f"调用Qwen-VL API时出错: {e}")
        return None

# ======================
# 数据查询
# ======================

def query_asm_object(obj_name: str) -> Optional[Dict[str, Any]]:
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
                # 使用新的格式字段
                world_x = obj.get("world_x")
                world_y = obj.get("world_y")
                if world_x is not None and world_y is not None:
                    location = {"x": world_x, "y": world_y}
                    return {
                        "location": location,
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

def query_history_db(obj_name: str) -> Optional[Dict[str, Any]]:
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

def get_video_path_by_time(timestamp: str) -> str:
    """根据时间获取视频路径"""
    # 使用真实家居视频文件
    real_video_path = "/home/jungong3/vln/as/home.mp4"
    if os.path.exists(real_video_path):
        return real_video_path
    # 如果真实视频文件不存在，回退到测试视频
    return os.path.join(VIDEO_BASE_DIR, "20251102.mp4")

# ======================
# Agent工具函数
# ======================

def find_object(obj_name: str) -> Dict[str, Any]:
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
        asm_res = query_asm_object(obj_name)
        if asm_res:
            loc = asm_res["location"]
            logger.info(f"[FIND_OBJECT] 在ASM中找到 {obj_name} 位置: ({loc['x']}, {loc['y']})")
            
            # ASM找到：返回位置信息，询问用户是否需要导航
            result_msg = f"找到 {obj_name} 的位置：地图坐标 ({loc['x']}, {loc['y']})"
            logger.info(f"[FIND_OBJECT] {result_msg}")
            
            # 按照新格式返回结果
            result_data = {
                "success": True,
                "world_position": [loc['x'], loc['y']],
                "position_description": "在沙发上"  # 修改为文字描述
            }
            
            return result_data

        # Step 2: DB查询
        db_res = query_history_db(obj_name)
        if not db_res:
            # DB没有找到：返回失败结果
            result_data = {
                "success": False,
                "world_position": None,
                "position_description": None
            }
            
            return result_data

        logger.info(f"[FIND_OBJECT] 在DB中找到 {obj_name} 记录，时间: {db_res['last_show_time']}")
        
        # DB找到：直接反馈结果，不询问导航
        result_data = {
            "success": True,
            "world_position": [db_res["world_x"], db_res["world_y"]],
            "position_description": db_res["object_description"]
        }
        
        return result_data
    except Exception as e:
        result_data = {
            "success": False,
            "world_position": None,
            "position_description": None
        }
        
        return result_data

async def explore_and_find_object(**kwargs) -> Dict[str, Any]:
    """
    探索并查找物品，通过WebSocket连接本地小模型服务
    
    Returns:
        Dict[str, Any]: 工具执行结果
    """
    # 处理参数格式:
    if "obj_name" in kwargs:
        obj_name = kwargs.get("obj_name", "")
    else:
        error_msg = "缺少必要参数obj_name"
        logger.error(f"[EXPLORE_AND_FIND_OBJECT] {error_msg}")
        return {
            "success": False,
            "object_name": "",
            "world_position": None,
            "position_description": error_msg
        }
    
    logger.info(f"[EXPLORE_AND_FIND_OBJECT] 开始探索查找物品: {obj_name}")

    try:
        # 获取全局SmartRobotAgent实例并校验
        global smart_robot_agent_instance
        if smart_robot_agent_instance is None:
            error_msg = "SmartRobotAgent实例未初始化"
            logger.error(f"[EXPLORE_AND_FIND_OBJECT] {error_msg}")
            return {
                "success": False,
                "object_name": obj_name,
                "world_position": None,
                "position_description": None
            }

        # 建立或确认与本地模型的连接
        connection_success = await smart_robot_agent_instance.connect_to_local_model()
        if not connection_success:
            logger.error("[EXPLORE_AND_FIND_OBJECT] 连接本地模型服务器失败")
            return {
                "success": False,
                "object_name": obj_name,
                "world_position": None,
                "position_description": None
            }

        # 构造发送给本地模型的数据（新协议格式）
        task_description = f"找到{obj_name}"
        logger.info(f"[EXPLORE_AND_FIND_OBJECT] 通过WebSocket连接本地小模型服务，任务描述: {task_description}")

        model_data = {
            "type": "explore_and_find_object",
            "user_prompt": task_description,
            "world_position": []  # 探索任务通常没有固定位置
        }

        # 使用通用的send_to_local_model方法
        task_id = f"explore_find_{obj_name}_{int(time.time())}"
        model_result = await smart_robot_agent_instance.send_to_local_model(model_data, task_id)
        
        if not model_result["success"]:
            return {
                "success": False,
                "error_msg": model_result["error_msg"]
            }
        
        model_response = model_result["response"]

        # 处理模型返回结果
        if model_response and model_response.get("success"):
            logger.info(f"[EXPLORE_AND_FIND_OBJECT] 本地模型成功返回结果 for {obj_name}")
            return {
                "success": True,
                "object_name": obj_name,
                "world_position": model_response.get("world_position"),
                "position_description": model_response.get("position_description") or f"找到{obj_name}"
            }
        else:
            err = model_response.get("error_msg") if isinstance(model_response, dict) else None
            logger.error(f"[EXPLORE_AND_FIND_OBJECT] 本地模型探索查找执行失败: {err}")
            return {
                "success": False,
                "object_name": obj_name,
                "world_position": None,
                "position_description": None
            }

    except Exception as e:
        logger.error(f"[EXPLORE_AND_FIND_OBJECT] 探索查找失败: {e}")
        return {
            "success": False,
            "object_name": obj_name,
            "world_position": None,
            "position_description": None
        }

async def go_to_object(world_position: Optional[List[float]] = None, location_info: Optional[str] = None, user_prompt: Optional[str] = None, **kwargs) -> Dict[str, Any]:
    """
    根据世界坐标导航到指定位置，通过WebSocket连接本地小模型服务
    
    Args:
        world_position (Optional[List[float]]): 世界坐标 [x, y]，可以为空
        location_info (Optional[str]): 位置信息描述（兼容旧参数）
        user_prompt (Optional[str]): 位置信息描述（新参数名）
        **kwargs: 其他额外参数
    
    Returns:
        Dict[str, Any]: 工具执行结果
    """
    # 统一处理location_info和user_prompt参数
    if user_prompt is not None:
        location_info = user_prompt
    elif not location_info:
        # 从kwargs中尝试获取其他可能的参数名
        location_info = kwargs.get('arguments', {}).get('user_prompt') if 'arguments' in kwargs else None
    
    logger.info(f"[GO_TO_OBJECT] 开始导航任务")
    if world_position:
        logger.info(f"[GO_TO_OBJECT] 目标坐标: {world_position}")
    if location_info:
        logger.info(f"[GO_TO_OBJECT] 位置信息: {location_info}")
    
    try:
        # 如果没有提供坐标，使用默认值或跳过坐标验证
        if world_position is None:
            world_position = [0.0, 0.0]  # 默认坐标
            logger.info(f"[GO_TO_OBJECT] 使用默认坐标: {world_position}")
        elif not isinstance(world_position, list) or len(world_position) < 2:
            logger.warning(f"[GO_TO_OBJECT] 无效的世界坐标格式: {world_position}，使用默认坐标")
            world_position = [0.0, 0.0]
        
        x, y = world_position[0], world_position[1]
        
        # 检查与本地模型的WebSocket连接状态
        logger.info(f"[GO_TO_OBJECT] 检查与本地模型的WebSocket连接状态")
        
        # 获取全局SmartRobotAgent实例
        global smart_robot_agent_instance
        if smart_robot_agent_instance is None:
            error_msg = "SmartRobotAgent实例未初始化"
            logger.error(f"[GO_TO_OBJECT] {error_msg}")
            
            result_data = {
                "success": False,
                "world_position": [x, y],
                "position_description": location_info or f"坐标 ({x}, {y})"
            }
            
            return result_data
        
        connection_success = await smart_robot_agent_instance.connect_to_local_model()
        
        # 如果连接不成功，返回任务失败
        if not connection_success:
            error_msg = "连接本地模型服务器失败"
            logger.error(f"[GO_TO_OBJECT] {error_msg}")
            
            result_data = {
                "success": False,
                "world_position": [x, y],
                "position_description": location_info or f"坐标 ({x}, {y})"
            }
            
            return result_data
        
        # 连接成功，构造发送给本地模型的数据（新协议格式）
        logger.info(f"[GO_TO_OBJECT] 通过WebSocket连接本地小模型服务，目标坐标: ({x}, {y})")
        
        model_data = {
            "type": "go_to_object",
            "user_prompt": location_info or f"去坐标({x}, {y})",
            "world_position": [x, y]
        }
        
        # 使用通用的send_to_local_model方法
        task_id = f"go_to_{x}_{y}_{int(time.time())}"
        model_result = await smart_robot_agent_instance.send_to_local_model(model_data, task_id)
        
        if not model_result["success"]:
            return {
                "success": False,
                "world_position": [x, y],
                "position_description": location_info or f"坐标 ({x}, {y})"
            }
        
        model_response = model_result["response"]
        
        # 检查响应格式并转换为统一格式
        if "result" in model_response:
            # 新协议格式
            model_response = {
                "success": model_response.get("result", False),
                "error_msg": model_response.get("error_msg", "")
            }
        # 旧协议格式直接使用
        # 检查连接状态并返回相应结果
        if model_response["success"]:
            success_msg = f"已通过本地模型成功导航到坐标: ({x}, {y})"
            logger.info(f"[GO_TO_OBJECT] {success_msg}")
            
            result_data = {
                "success": True,
                "world_position": [x, y],
                "position_description": location_info or f"坐标 ({x}, {y})"
            }
            
            return result_data
        else:
            error_msg = f"本地模型导航执行失败: {model_response['error_msg']}"
            logger.error(f"[GO_TO_OBJECT] {error_msg}")
            
            result_data = {
                "success": False,
                "world_position": [x, y],
                "position_description": location_info or f"坐标 ({x}, {y})"
            }
            
            return result_data
                
    except Exception as e:
        error_msg = f"导航失败: {str(e)}"
        logger.error(f"[GO_TO_OBJECT] {error_msg}")
        
        result_data = {
            "success": False,
            "world_position": world_position if 'world_position' in locals() else None,
            "position_description": location_info
        }
        
        return result_data

async def go_find_person(**kwargs) -> Dict[str, Any]:
    """
    查找指定人员，通过WebSocket连接本地小模型服务
    
    Args:
        person_id (str): 人员ID
        
    Returns:
        Dict[str, Any]: 工具执行结果
    """
    # 处理参数格式:
    if "person_id" in kwargs:
        person_id = kwargs.get("person_id", "")
    else:
        error_msg = "缺少必要参数person_id"
        logger.error(f"[GO_FIND_PERSON] {error_msg}")
        return {
            "success": False,
            "person_id": "",
            "world_position": None,
            "position_description": error_msg
        }
    
    logger.info(f"[GO_FIND_PERSON] 开始查找人员: {person_id}")

    try:
        # 获取全局SmartRobotAgent实例并校验
        global smart_robot_agent_instance
        if smart_robot_agent_instance is None:
            error_msg = "SmartRobotAgent实例未初始化"
            logger.error(f"[GO_FIND_PERSON] {error_msg}")
            return {
                "success": False,
                "person_id": person_id,
                "world_position": None,
                "position_description": None
            }

        # 建立或确认与本地模型的连接
        connection_success = await smart_robot_agent_instance.connect_to_local_model()
        if not connection_success:
            logger.error("[GO_FIND_PERSON] 连接本地模型服务器失败")
            return {
                "success": False,
                "person_id": person_id,
                "world_position": None,
                "position_description": None
            }

        # 构造发送给本地模型的数据
        task_description = f"去找这个人"
        logger.info(f"[GO_FIND_PERSON] 通过WebSocket连接本地小模型服务，任务描述: {task_description}")

        model_data = {
            "instruction": task_description,
            "person_id": person_id
        }

        # 使用通用的send_to_local_model方法
        task_id = f"find_person_{person_id}_{int(time.time())}"
        model_result = await smart_robot_agent_instance.send_to_local_model(model_data, task_id)
        
        if not model_result["success"]:
            return {
                "success": False,
                "person_id": person_id,
                "world_position": None,
                "position_description": None
            }
        
        model_response = model_result["response"]

        # 处理模型返回结果
        if model_response and model_response.get("success"):
            logger.info(f"[GO_FIND_PERSON] 本地模型成功返回结果 for {person_id}")
            return {
                "success": True,
                "person_id": person_id,
                "world_position": model_response.get("world_position"),
                "position_description": model_response.get("position_description") or f"找到{person_id}"
            }
        else:
            err = model_response.get("error_msg") if isinstance(model_response, dict) else None
            logger.error(f"[GO_FIND_PERSON] 本地模型查找执行失败: {err}")
            return {
                "success": False,
                "person_id": person_id,
                "world_position": None,
                "position_description": None
            }

    except Exception as e:
        logger.error(f"[GO_FIND_PERSON] 查找失败: {e}")
        return {
            "success": False,
            "person_id": person_id,
            "world_position": None,
            "position_description": None
        }

async def follow_person(location_info: Optional[str] = None, person_id: Optional[str] = None, user_prompt: Optional[str] = None, **kwargs) -> Dict[str, Any]:
    """
    跟随指定人员，通过WebSocket连接本地小模型服务
    
    Args:
        location_info (Optional[str]): 人员描述信息（兼容旧参数）
        person_id (Optional[str]): 人员ID
        user_prompt (Optional[str]): 人员描述信息（新参数名）
        **kwargs: 其他额外参数
    
    Returns:
        Dict[str, Any]: 工具执行结果
    """
    # 统一处理location_info和user_prompt参数
    if user_prompt is not None:
        location_info = user_prompt
    elif not location_info:
        # 从kwargs中尝试获取其他可能的参数名
        location_info = kwargs.get('arguments', {}).get('user_prompt') if 'arguments' in kwargs else None
        if not location_info:
            location_info = "跟随人员"  # 默认描述
    
    logger.info(f"[FOLLOW_PERSON] 开始跟随人员")
    if location_info:
        logger.info(f"[FOLLOW_PERSON] 人员描述: {location_info}")
    if person_id:
        logger.info(f"[FOLLOW_PERSON] 人员ID: {person_id}")
    
    try:
        # 获取全局SmartRobotAgent实例
        global smart_robot_agent_instance
        if smart_robot_agent_instance is None:
            error_msg = "SmartRobotAgent实例未初始化"
            logger.error(f"[FOLLOW_PERSON] {error_msg}")
            
            result_data = {
                "success": False,
                "result": error_msg
            }
            
            return result_data
        
        # 建立或确认与本地模型的连接
        connection_success = await smart_robot_agent_instance.connect_to_local_model()
        if not connection_success:
            error_msg = "连接本地模型服务器失败"
            logger.error(f"[FOLLOW_PERSON] {error_msg}")
            
            result_data = {
                "success": False,
                "result": error_msg
            }
            
            return result_data
            
        # 构造发送给本地模型的数据（新格式）
        logger.info(f"[FOLLOW_PERSON] 通过WebSocket连接本地小模型服务")
        
        # 确定使用user_prompt还是person_id
        if person_id:
            model_data = {
                "type": "follow_person",
                "person_id": person_id
            }
        elif location_info:
            model_data = {
                "type": "follow_person", 
                "user_prompt": location_info
            }
        else:
            # 默认跟随人员
            model_data = {
                "type": "follow_person",
                "user_prompt": "跟随人员"
            }
        
        # 使用通用的send_to_local_model方法
        task_id = f"follow_person_{person_id or location_info}_{int(time.time())}"
        model_result = await smart_robot_agent_instance.send_to_local_model(model_data, task_id)
        
        if not model_result["success"]:
            return {
                "success": False,
                "result": model_result["error_msg"]
            }
        
        model_response = model_result["response"]
        
        # 处理模型返回结果
        if model_response and model_response.get("result"):
            logger.info(f"[FOLLOW_PERSON] 本地模型成功返回结果")
            return {
                "success": True,
                "result": "跟随人员成功"
            }
        else:
            err = model_response.get("error_msg") if isinstance(model_response, dict) else None
            error_msg = f"本地模型跟随执行失败: {err}"
            logger.error(f"[FOLLOW_PERSON] {error_msg}")
            return {
                "success": False,
                "result": error_msg
            }
                
    except Exception as e:
        error_msg = f"跟随失败: {str(e)}"
        logger.error(f"[FOLLOW_PERSON] {error_msg}")
        
        result_data = {
            "success": False,
            "result": error_msg
        }
        
        return result_data

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
                "status": "success",
                "result": success_msg
            }
            
            return result_data
        else:
            error_msg = "停止跟随失败"
            
            result_data = {
                "status": "error",
                "result": error_msg
            }
            
            return result_data
    except Exception as e:
        error_msg = f"停止跟随失败: {str(e)}"
        
        result_data = {
            "status": "error",
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
                "status": "success",
                "result": success_msg
            }
            
            return result_data
        else:
            error_msg = "停止导航失败"
            
            result_data = {
                "status": "error",
                "result": error_msg
            }
            
            return result_data
    except Exception as e:
        error_msg = f"停止导航失败: {str(e)}"
        
        result_data = {
            "status": "error",
            "result": error_msg
        }
        
        return result_data

# ======================
# 初始化测试数据
# ======================

def init_test_data():
    """初始化测试数据"""
    # ASM - 检查文件是否存在，不存在则报错
    if not os.path.exists(ASM_JSON_PATH):
        logger.error(f"ASM数据文件不存在: {ASM_JSON_PATH}")
        raise FileNotFoundError(f"ASM数据文件不存在: {ASM_JSON_PATH}")
    else:
        logger.info("ASM数据文件存在，继续执行")

    # DB
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    
    # 删除现有的表（如果存在）
    cursor.execute("DROP TABLE IF EXISTS objects")
    
    # 重新创建表
    cursor.execute("""
        CREATE TABLE objects (
            id INTEGER PRIMARY KEY,
            name TEXT,
            world_x REAL,
            world_y REAL,
            last_show_time TEXT TIMESTAMP,
            exist_or_not INTEGER,
            object_description TEXT
        )
    """)
    
    # 插入更多测试数据，包括电视机和人员
    test_objects = [
        (1, "遥控器", 2.1, 1.5, "2025-11-02T09:30:00", 1, "在电视柜上"),
        (2, "钥匙", 0.5, 3.2, "2025-11-02T08:15:00", 1, "在玄关柜子上"),
        (3, "书本", 4.0, 1.0, "2025-11-02T07:45:00", 1, "在书房桌子上"),
        (4, "钱包", 1.8, 4.5, "2025-11-02T06:30:00", 1, "在卧室床头柜上"),
        (5, "电视机", 3.5, 2.0, "2025-11-02T11:00:00", 1, "在客厅墙上"),
        (6, "小明", 1.0, 1.0, "2025-11-02T10:30:00", 1, "在沙发上")
    ]
    
    for id, name, world_x, world_y, last_show_time, exist_or_not, object_description in test_objects:
        cursor.execute("""
            INSERT INTO objects (id, name, world_x, world_y, last_show_time, exist_or_not, object_description)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, (id, name, world_x, world_y, last_show_time, exist_or_not, object_description))
        logger.info(f"[INIT] Inserted object {name} with id {id} at location ({world_x}, {world_y})")
    
    conn.commit()
    conn.close()

# ======================
# WebSocket服务器
# ======================

class WebSocketServer:
    def __init__(self, agent=None):
        self.clients = set()
        self._reply_task: Optional[asyncio.Task] = None
        self.agent = agent
        
    async def register_client(self, websocket):
        """注册新的WebSocket客户端"""
        self.clients.add(websocket)
        logger.info(f"New client connected. Total clients: {len(self.clients)}")
        
    async def unregister_client(self, websocket):
        """注销WebSocket客户端"""
        self.clients.discard(websocket)
        logger.info(f"Client disconnected. Total clients: {len(self.clients)}")
        
    async def send_to_clients(self, message):
        """向所有连接的客户端发送消息"""
        if self.clients:
            await asyncio.gather(
                *[client.send(message) for client in self.clients],
                return_exceptions=True
            )
            
    async def handle_message(self, websocket, message):
        """处理来自客户端的消息"""
        try:
            # 解析JSON消息
            data = json.loads(message)
            
            # 检查是否是任务执行请求
            if "tasks" in data:
                tasks = data["tasks"]
                logger.info(f"收到 {len(tasks)} 个任务请求")
                
                # 执行任务
                results = []
                for task in tasks:
                    # 这里应该调用智能Agent来处理任务
                    # 暂时使用简单的任务执行器
                    result = await self.execute_task(task)
                    results.append(result)
                
                # 发送响应给客户端
                response = {
                    "type": "task_results",
                    "results": results
                }
                response_str = json.dumps(response, ensure_ascii=False)
                await websocket.send(response_str)
                
                # 广播给其他客户端（不包括当前客户端）
                other_clients = [client for client in self.clients if client != websocket]
                if other_clients:
                    await asyncio.gather(
                        *[client.send(response_str) for client in other_clients],
                        return_exceptions=True
                    )
            else:
                # 其他类型的消息
                response = {
                    "type": "error",
                    "message": "未知的消息格式，请发送包含tasks字段的任务请求"
                }
                await websocket.send(json.dumps(response, ensure_ascii=False))
                
        except (ConnectionClosedError, ConnectionClosedOK):
            # 连接已关闭，静默处理不报错
            logger.info("客户端连接已关闭，跳过响应发送")
            return
        except json.JSONDecodeError:
            try:
                await websocket.send(json.dumps({"type": "error", "message": "Invalid JSON format"}, ensure_ascii=False))
            except (ConnectionClosedError, ConnectionClosedOK):
                logger.info("WebSocket连接已关闭，无法发送JSON解码错误响应")
            except Exception as send_error:
                logger.warning(f"Failed to send JSON decode error response: {send_error}")
        except Exception as e:
            logger.error(f"Error handling message: {e}")
            try:
                await websocket.send(json.dumps({"type": "error", "message": str(e)}, ensure_ascii=False))
            except (ConnectionClosedError, ConnectionClosedOK):
                logger.info("WebSocket连接已关闭，无法发送错误响应")
            except Exception as send_error:
                logger.warning(f"Failed to send error response: {send_error}")
                
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
    
    async def _execute_task_by_type(self, task_type: str, params: Dict[str, Any]) -> Dict[str, Any]:
        """根据任务类型执行相应的工具函数"""
        if task_type == "find_object":
            return find_object(**params)
        elif task_type == "explore_and_find_object":
            return await explore_and_find_object(**params)
        elif task_type == "go_to_object":
            return await go_to_object(**params)
        elif task_type == "go_find_person":
            return await go_find_person(**params)
        elif task_type == "follow_person":
            return await follow_person(**params)
        elif task_type == "stop_follow":
            return stop_follow()
        elif task_type == "stop_navigate":
            return stop_navigate()
        elif task_type == "stop_move" and self.agent:
            return await self.agent.stop_move()
        # ROS2接口任务类型
        elif task_type == "get_move_mode" and self.agent:
            return self.agent.ros2_interface.get_move_mode()
        elif task_type == "set_move_mode" and self.agent:
            return self.agent.ros2_interface.set_move_mode(**params)
        elif task_type == "get_medicine_box_state" and self.agent:
            return self.agent.ros2_interface.get_medicine_box_state()
        elif task_type == "set_medicine_box_switch" and self.agent:
            return self.agent.ros2_interface.set_medicine_box_switch(**params)
        elif task_type == "get_robot_rise_state" and self.agent:
            return self.agent.ros2_interface.get_robot_rise_state()
        elif task_type == "set_robot_rise_jqr" and self.agent:
            return self.agent.ros2_interface.set_robot_rise_jqr(**params)
        elif task_type == "get_robot_tilt_state" and self.agent:
            return self.agent.ros2_interface.get_robot_tilt_state()
        elif task_type == "set_robot_tilt_jqr" and self.agent:
            return self.agent.ros2_interface.set_robot_tilt_jqr(**params)
        elif task_type == "get_screen_tilt_state" and self.agent:
            return self.agent.ros2_interface.get_screen_tilt_state()
        elif task_type == "set_screen_tilt_jqr" and self.agent:
            return self.agent.ros2_interface.set_screen_tilt_jqr(**params)
        else:
            return {
                "status": "error",
                "result": f"未知的任务类型: {task_type}"
            }
            
    async def websocket_handler(self, websocket):
        """WebSocket处理函数"""
        # 注册客户端
        await self.register_client(websocket)
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        except (ConnectionClosedError, ConnectionClosedOK) as e:
            # 区分正常关闭和异常关闭
            if hasattr(e, 'code') and e.code == 1000:
                logger.info(f"WebSocket连接正常关闭: {e}")
            else:
                logger.warning(f"WebSocket连接异常关闭: {e}")
        except Exception as e:
            logger.error(f"处理WebSocket消息时发生未预期错误: {e}")
        finally:
            # 注销客户端
            await self.unregister_client(websocket)

# ======================
# 电池订阅相关函数
# ======================

# 全局变量存储电池电量和相关状态
battery_level = 100.0
battery_node = None
battery_thread = None
battery_thread_running = False
websocket_server_ref = None

# 全局SmartRobotAgent实例
smart_robot_agent_instance = None

def battery_callback(msg):
    """电池电量回调函数 - 收到信息后立马发给client
    
    Args:
        msg: BatteryLevel消息
    """
    global battery_level, websocket_server_ref
    
    try:
        # 更新电池电量
        battery_level = msg.battery_power_state
        logger.info(f"[BATTERY] 收到电池电量更新: {battery_level}%")
        
        # 立马发送电池电量信息到所有连接的客户端
        if websocket_server_ref and websocket_server_ref.clients:
            battery_data = {
                "type": "battery_level",
                "level": battery_level,
                "description": f"当前电池电量: {battery_level:.1f}%"
            }
            
            # 在事件循环中发送消息
            try:
                loop = asyncio.get_event_loop()
                if loop.is_running():
                    asyncio.run_coroutine_threadsafe(
                        websocket_server_ref.send_to_clients(
                            json.dumps(battery_data, ensure_ascii=False)
                        ),
                        loop
                    )
                else:
                    # 如果事件循环没有运行，直接运行
                    loop.run_until_complete(
                        websocket_server_ref.send_to_clients(
                            json.dumps(battery_data, ensure_ascii=False)
                        )
                    )
            except RuntimeError:
                # 如果没有运行的事件循环，创建新的
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(
                    websocket_server_ref.send_to_clients(
                        json.dumps(battery_data, ensure_ascii=False)
                    )
                )
                
        logger.info(f'电池电量更新: {battery_level:.1f}%')
    except Exception as e:
        logger.error(f'处理电池电量回调时出错: {e}')

def ros2_spin_loop():
    """ROS2 spin循环"""
    global battery_node, battery_thread_running
    
    if not ROS2_AVAILABLE:
        logger.error("ROS2不可用，无法启动ROS2 spin循环")
        return
        
    try:
        while rclpy is not None and rclpy.ok() and battery_thread_running and battery_node:
            rclpy.spin_once(battery_node, timeout_sec=0.1)
            time.sleep(0.01)  # 短暂休眠以避免CPU占用过高
    except Exception as e:
        logger.error(f"ROS2 spin循环出错: {e}")
    finally:
        if battery_node:
            battery_node.destroy_node()
            battery_node = None

# ======================
# ROS2接口实现
# ======================

class ROS2Interface:
    """ROS2接口类，用于与ROS2系统进行交互"""
    
    def __init__(self):
        """初始化ROS2接口"""
        self.battery_level = 100.0  # 初始电池电量
        
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
            cmd = f"ros2 service call {service_name} {service_type} '{request_data}'"
            logger.info(f"[ROS2] 执行命令: {cmd}")
            
            # 执行命令并获取输出
            result = os.popen(cmd).read().strip()
            logger.info(f"[ROS2] 服务调用结果: {result}")
            return result
        except Exception as e:
            logger.error(f"[ROS2] 服务调用失败: {e}")
            return None
            
    def _call_ros2_action(self, action_name: str, action_type: str, goal_data: str) -> Optional[str]:
        """调用ROS2动作
        
        Args:
            action_name (str): 动作名称
            action_type (str): 动作类型
            goal_data (str): 目标数据
            
        Returns:
            Optional[str]: 动作执行结果，如果调用失败则返回None
        """
        try:
            # 构造ROS2动作调用命令
            cmd = f"ros2 action send_goal {action_name} {action_type} '{goal_data}'"
            logger.info(f"[ROS2] 执行命令: {cmd}")
            
            # 执行命令并获取输出
            result = os.popen(cmd).read().strip()
            logger.info(f"[ROS2] 动作调用结果: {result}")
            return result
        except Exception as e:
            logger.error(f"[ROS2] 动作调用失败: {e}")
            return None
            
    # ======================
    # 药箱控制相关接口
    # ======================
    
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
            elif response:
                # 解析响应数据
                # 根据MedicineBoxState.srv定义，响应包含medicine_box_switch_state字段
                # 这里简化处理，实际应解析具体的响应内容
                result = {
                    "success": True,
                    "state": True,  # 实际应从响应中提取
                    "description": "药箱当前处于开启状态"
                }
                logger.info(f"[ROS2] 获取药箱状态: {result}")
                return result
            else:
                # 如果服务调用成功但返回空，返回默认值
                result = {
                    "success": True,
                    "state": False,
                    "description": "药箱当前处于关闭状态"
                }
                logger.info(f"[ROS2] 获取药箱状态(默认值): {result}")
                return result
        except Exception as e:
            logger.error(f"[ROS2] 获取药箱状态失败: {e}")
            return {
                "success": False,
                "state": False,
                "description": f"获取药箱状态失败: {str(e)}"
            }
        
    def set_medicine_box_switch(self, switch: bool, duration: int = 0, speed_stage: int = 1) -> Dict[str, Any]:
        """控制药箱开关
        
        Args:
            switch (bool): 开关状态 (True: 开启, False: 关闭)
            duration (int): 执行时间（单位0.1s），缺省表示希望以最快的速度执行
            speed_stage (int): 执行速度 1是慢档 2是快档
            
        Returns:
            Dict[str, Any]: 控制结果
        """
        try:
            # 构造请求数据，根据新的jqr_ros_msgs格式包含speed_stage参数
            if duration > 0:
                request_data = f"{{medicine_box_switch: {str(switch).lower()}, speed_stage: {speed_stage}}}"
            else:
                request_data = f"{{medicine_box_switch: {str(switch).lower()}, speed_stage: {speed_stage}}}"
                
            # 调用ROS2服务控制药箱开关
            response = self._call_ros2_service(
                "/set_medicine_box_switch",
                "jqr_ros_msgs/srv/MedicineBoxSwitch",
                request_data
            )
            
            if response is None:
                # 服务调用失败，可能是服务不存在
                result = {
                    "success": False,
                    "switch": switch,
                    "description": "服务 /set_medicine_box_switch 不存在或调用失败"
                }
                logger.error(f"[ROS2] 设置药箱开关失败: {result}")
                return result
            elif response:
                result = {
                    "success": True,
                    "switch": switch,
                    "description": f"药箱已{'开启' if switch else '关闭'}"
                }
                logger.info(f"[ROS2] 设置药箱开关: {result}")
                return result
            else:
                result = {
                    "success": False,
                    "switch": switch,
                    "description": f"设置药箱{'开启' if switch else '关闭'}失败"
                }
                logger.error(f"[ROS2] 设置药箱开关失败: {result}")
                return result
        except Exception as e:
            logger.error(f"[ROS2] 设置药箱开关失败: {e}")
            return {
                "success": False,
                "switch": switch,
                "description": f"设置药箱{'开启' if switch else '关闭'}失败: {str(e)}"
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
                "{}"
            )
            
            if response is None:
                # 服务调用失败，可能是服务不存在
                result = {
                    "success": False,
                    "mode": 0,
                    "linear_vel": 0.0,
                    "description": "服务 /get_move_mode 不存在或调用失败"
                }
                logger.error(f"[ROS2] 获取运动模式失败: {result}")
                return result
            elif response:
                # 解析响应数据
                # 根据MoveMode.srv定义，响应包含move_mode和linear_vel字段
                # 这里简化处理，实际应解析具体的响应内容
                result = {
                    "success": True,
                    "mode": 0,  # 实际应从响应中提取
                    "linear_vel": 0.0,  # 实际应从响应中提取
                    "description": "当前运动模式: 停止模式，线速度: 0.0 m/s"
                }
                logger.info(f"[ROS2] 获取运动模式: {result}")
                return result
            else:
                # 如果服务调用成功但返回空，返回默认值
                result = {
                    "success": True,
                    "mode": 0,
                    "linear_vel": 0.0,
                    "description": "当前运动模式: 停止模式，线速度: 0.0 m/s"
                }
                logger.info(f"[ROS2] 获取运动模式(默认值): {result}")
                return result
        except Exception as e:
            logger.error(f"[ROS2] 获取运动模式失败: {e}")
            return {
                "success": False,
                "mode": 0,
                "linear_vel": 0.0,
                "description": f"获取运动模式失败: {str(e)}"
            }
        
    def set_move_mode(self, mode: int, linear_vel: float = 0.0) -> Dict[str, Any]:
        """设置运动模式
        
        Args:
            mode (int): 运动模式
            linear_vel (float): 线速度
            
        Returns:
            Dict[str, Any]: 设置结果
        """
        try:
            # 构造请求数据
            request_data = f"{{move_mode: {mode}, linear_vel: {linear_vel}}}"
                
            # 调用ROS2服务设置运动模式
            response = self._call_ros2_service(
                "/set_move_mode",
                "jqr_ros_msgs/srv/MoveMode",
                request_data
            )
            
            if response is None:
                # 服务调用失败，可能是服务不存在
                result = {
                    "success": False,
                    "mode": mode,
                    "linear_vel": linear_vel,
                    "description": "服务 /set_move_mode 不存在或调用失败"
                }
                logger.error(f"[ROS2] 设置运动模式失败: {result}")
                return result
            elif response:
                result = {
                    "success": True,
                    "mode": mode,
                    "linear_vel": linear_vel,
                    "description": f"运动模式已设置为: {mode}，线速度: {linear_vel} m/s"
                }
                logger.info(f"[ROS2] 设置运动模式: {result}")
                return result
            else:
                result = {
                    "success": False,
                    "mode": mode,
                    "linear_vel": linear_vel,
                    "description": f"设置运动模式失败"
                }
                logger.error(f"[ROS2] 设置运动模式失败: {result}")
                return result
        except Exception as e:
            logger.error(f"[ROS2] 设置运动模式失败: {e}")
            return {
                "success": False,
                "mode": mode,
                "linear_vel": linear_vel,
                "description": f"设置运动模式失败: {str(e)}"
            }
        
    # ======================
    # 机器人升降控制相关接口
    # ======================
    
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
                request_data = f"{{robot_rise: {str(rise).lower()}, duration: {duration}}}"
            else:
                request_data = f"{{robot_rise: {str(rise).lower()}}}"
                
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
            elif response:
                result = {
                    "success": True,
                    "rise": rise,
                    "description": f"机器人已{'上升' if rise else '下降'}"
                }
                logger.info(f"[ROS2] 设置机器人升降: {result}")
                return result
            else:
                result = {
                    "success": False,
                    "rise": rise,
                    "description": f"设置机器人{'上升' if rise else '下降'}失败"
                }
                logger.error(f"[ROS2] 设置机器人升降失败: {result}")
                return result
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
                "/get_robot_rise_state",
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
            elif response:
                # 解析响应数据
                # 根据RobotRiseState.srv定义，响应包含robot_rise_state字段
                # 这里简化处理，实际应解析具体的响应内容
                result = {
                    "success": True,
                    "state": True,  # 实际应从响应中提取
                    "description": "机器人当前处于上升状态"
                }
                logger.info(f"[ROS2] 获取机器人升降状态: {result}")
                return result
            else:
                # 如果服务调用成功但返回空，返回默认值
                result = {
                    "success": True,
                    "state": False,
                    "description": "机器人当前处于下降状态"
                }
                logger.info(f"[ROS2] 获取机器人升降状态(默认值): {result}")
                return result
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
                request_data = f"{{robot_tilt: {angle}, duration: {duration}}}"
            else:
                request_data = f"{{robot_tilt: {angle}}}"
                
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
            elif response:
                result = {
                    "success": True,
                    "angle": angle,
                    "description": f"机器人俯仰角度已设置为 {angle} 度"
                }
                logger.info(f"[ROS2] 设置机器人俯仰角度: {result}")
                return result
            else:
                result = {
                    "success": False,
                    "angle": angle,
                    "description": f"设置机器人俯仰角度失败"
                }
                logger.error(f"[ROS2] 设置机器人俯仰角度失败: {result}")
                return result
        except Exception as e:
            logger.error(f"[ROS2] 设置机器人俯仰角度失败: {e}")
            return {
                "success": False,
                "angle": angle,
                "description": f"设置机器人俯仰角度失败: {str(e)}"
            }
        
    def get_robot_tilt_state(self) -> Dict[str, Any]:
        """获取机器人俯仰状态
        
        Returns:
            Dict[str, Any]: 俯仰状态信息
        """
        try:
            # 调用ROS2服务获取机器人俯仰状态
            response = self._call_ros2_service(
                "/get_robot_tilt_state",
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
            elif response:
                # 解析响应数据
                # 根据RobotTiltState.srv定义，响应包含robot_tilt_state字段
                # 这里简化处理，实际应解析具体的响应内容
                result = {
                    "success": True,
                    "angle": 0.0,  # 实际应从响应中提取
                    "description": "机器人俯仰角度为 0.0 度"
                }
                logger.info(f"[ROS2] 获取机器人俯仰状态: {result}")
                return result
            else:
                # 如果服务调用成功但返回空，返回默认值
                result = {
                    "success": True,
                    "angle": 0.0,
                    "description": "机器人俯仰角度为 0.0 度"
                }
                logger.info(f"[ROS2] 获取机器人俯仰状态(默认值): {result}")
                return result
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
                # 解析响应数据
                # 这里需要根据实际的服务响应格式进行解析
                result = {
                    "success": True,
                    "height": 0.0,  # 从响应中提取的实际值
                    "description": "机身升降高度为 0.0 米"
                }
                logger.info(f"[ROS2] 获取机身升降状态: {result}")
                return result
            else:
                # 如果服务调用失败，返回默认值
                result = {
                    "success": True,
                    "height": 0.0,
                    "description": "机身升降高度为 0.0 米"
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
                request_data = f"{{screen_tilt: {angle}, duration: {duration}}}"
            else:
                request_data = f"{{screen_tilt: {angle}}}"
                
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
            elif response:
                result = {
                    "success": True,
                    "angle": angle,
                    "description": f"屏幕俯仰角度已设置为 {angle} 度"
                }
                logger.info(f"[ROS2] 设置屏幕俯仰角度: {result}")
                return result
            else:
                result = {
                    "success": False,
                    "angle": angle,
                    "description": f"设置屏幕俯仰角度失败"
                }
                logger.error(f"[ROS2] 设置屏幕俯仰角度失败: {result}")
                return result
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
                "/get_screen_tilt_state",
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
            elif response:
                # 解析响应数据
                # 根据ScreenTiltState.srv定义，响应包含screen_tilt_state字段
                # 这里简化处理，实际应解析具体的响应内容
                result = {
                    "success": True,
                    "angle": 0.0,  # 实际应从响应中提取
                    "description": "屏幕俯仰角度为 0.0 度"
                }
                logger.info(f"[ROS2] 获取屏幕俯仰状态: {result}")
                return result
            else:
                # 如果服务调用成功但返回空，返回默认值
                result = {
                    "success": True,
                    "angle": 0.0,
                    "description": "屏幕俯仰角度为 0.0 度"
                }
                logger.info(f"[ROS2] 获取屏幕俯仰状态(默认值): {result}")
                return result
        except Exception as e:
            logger.error(f"[ROS2] 获取屏幕俯仰状态失败: {e}")
            return {
                "success": False,
                "angle": 0.0,
                "description": f"获取屏幕俯仰状态失败: {str(e)}"
            }
        
    def start_battery_monitoring(self, websocket_server: WebSocketServer):
        """开始电池电量监控（订阅模式）
        
        Args:
            websocket_server (WebSocketServer): WebSocket服务器实例
        """
        global battery_node, battery_thread, battery_thread_running, websocket_server_ref
        
        if battery_node is not None:
            logger.warning("电池电量监控已在运行")
            return True
            
        try:
            # 保存WebSocket服务器引用
            websocket_server_ref = websocket_server
            
            # 初始化rclpy（如果尚未初始化）
            # 在调用 rclpy.ok() 之前先确保 ROS2 可用且 rclpy 已成功导入
            if not ROS2_AVAILABLE or rclpy is None:
                logger.error("ROS2不可用或rclpy未导入，无法初始化rclpy")
                return False
            try:
                # 如果 rclpy 尚未初始化则初始化
                if not rclpy.ok():
                    rclpy.init()
            except Exception as e:
                logger.error(f"初始化rclpy失败: {e}")
                return False
            
            # 创建简单的节点
            if ROS2_AVAILABLE and Node is not None:
                battery_node = Node('battery_subscriber')
            else:
                logger.error("Node类不可用，无法创建ROS2节点")
                return False
            
            # 创建电池电量订阅者
            battery_node.create_subscription(
                BatteryLevel,
                '/battery_level',  # 电池电量话题
                battery_callback,
                10  # 队列大小
            )
            
            # 启动ROS2 spin循环
            battery_thread_running = True
            battery_thread = threading.Thread(
                target=ros2_spin_loop,
                daemon=True
            )
            battery_thread.start()
            
            logger.info("电池电量订阅监控已启动")
            return True
            
        except Exception as e:
            logger.error(f"启动电池电量监控失败: {e}")
            return False
        
    def stop_battery_monitoring(self):
        """停止电池电量监控"""
        global battery_node, battery_thread, battery_thread_running
        
        try:
            # 停止spin循环
            battery_thread_running = False
            
            if battery_node:
                # 销毁节点
                battery_node.destroy_node()
                battery_node = None
                
            if battery_thread and battery_thread.is_alive():
                battery_thread.join(timeout=2.0)
                
            # 如果没有其他节点在使用rclpy，则关闭rclpy
            if ROS2_AVAILABLE and rclpy and rclpy.ok():
                try:
                    rclpy.shutdown()
                except:
                    pass  # 忽略关闭时的错误
                    
            logger.info("电池电量监控已停止")
            return True
            
        except Exception as e:
            logger.error(f"停止电池电量监控失败: {e}")
            return False

# ======================
# 智能机器人Agent
# ======================

class SmartRobotAgent:
    """智能机器人Agent，具备自主决策和环境交互能力"""
    
    def __init__(self) -> None:
        """初始化智能机器人Agent"""
        # 初始化测试数据
        init_test_data()
        
        # 创建ROS2接口
        self.ros2_interface = ROS2Interface()
        
        # 创建WebSocket服务器
        self.websocket_server = WebSocketServer(self)
        
        # 任务中断标志
        self._task_interrupted = False
        
        # 本地模型WebSocket连接相关
        self.local_model_websocket = None
        self.local_model_connected = False
        self.local_model_uri = "ws://localhost:8769"
        # self.local_model_uri = "ws://192.168.50.144:8000/ws/navigate"
        
        # 任务执行状态跟踪
        self.active_navigation_tasks = set()  # 正在执行的导航任务ID集合
        self.task_execution_lock = asyncio.Lock()  # 任务执行锁
        
    async def send_to_local_model(self, model_data: Dict[str, Any], task_id: Optional[str] = None) -> Dict[str, Any]:
        """
        通用的发送数据到本地模型的方法
        
        Args:
            model_data (Dict[str, Any]): 要发送给本地模型的数据
            task_id (str, optional): 任务ID，用于跟踪任务状态
            
        Returns:
            Dict[str, Any]: 本地模型的响应结果
        """
        try:
            # 检查WebSocket连接是否存在
            if self.local_model_websocket is None:
                return {
                    "success": False,
                    "error_msg": "本地模型WebSocket连接未建立"
                }
            
            # 如果提供了任务ID，添加到活跃任务集合
            if task_id:
                async with self.task_execution_lock:
                    self.active_navigation_tasks.add(task_id)
                logger.info(f"[LOCAL_MODEL] 添加任务 {task_id} 到活跃任务集合")
            
            # 发送数据到本地模型
            await self.local_model_websocket.send(json.dumps(model_data, ensure_ascii=False))
            logger.info(f"[LOCAL_MODEL] 已发送数据: {model_data}")
            
            # 持续接收响应，直到收到最终结果（包含result字段）
            final_response = None
            while True:
                response_str = await self.local_model_websocket.recv()
                response_data = json.loads(response_str)
                logger.info(f"[LOCAL_MODEL] 收到响应: {response_data}")
                
                # 如果是中间信息（非最终结果），转发给所有连接的客户端
                if "result" not in response_data and "success" not in response_data:
                    await websocket_server_ref.send_to_clients(json.dumps(response_data, ensure_ascii=False))
                    logger.info(f"[LOCAL_MODEL] 已转发中间信息给客户端: {response_data}")
                # 检查是否是最终结果
                elif "result" in response_data or "success" in response_data:
                    final_response = response_data
                    break
            
            return {
                "success": True,
                "response": final_response
            }
            
        except Exception as e:
            logger.error(f"[LOCAL_MODEL] 与本地模型通信失败: {e}")
            return {
                "success": False,
                "error_msg": f"与本地模型通信失败: {str(e)}"
            }
        finally:
            # 如果提供了任务ID，从活跃任务集合中移除
            if task_id:
                async with self.task_execution_lock:
                    self.active_navigation_tasks.discard(task_id)
                logger.info(f"[LOCAL_MODEL] 从活跃任务集合中移除任务 {task_id}")
    
    def has_active_navigation_tasks(self) -> bool:
        """
        检查是否有正在执行的导航任务
        
        Returns:
            bool: 如果有活跃的导航任务返回True，否则返回False
        """
        return len(self.active_navigation_tasks) > 0
        
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
        
    async def start_websocket_server(self):
        """启动WebSocket服务器"""
        try:
            server = await serve(
                self.websocket_server.websocket_handler,
                WEBSOCKET_HOST,
                WEBSOCKET_PORT
            )
            logger.info(f"WebSocket server started on {WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
            return server
        except OSError as e:
            if "Address already in use" in str(e) or "address already in use" in str(e) or "地址被占用" in str(e):
                logger.error(f"Port {WEBSOCKET_PORT} is already in use. Please check if another instance is running.")
                raise OSError(f"Port {WEBSOCKET_PORT} is already in use. Please check if another instance is running.")
            else:
                raise e
        
    def interrupt(self) -> None:
        """中断当前任务执行"""
        self._task_interrupted = True
        logger.info("任务执行已被中断")
        
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
                
                if self.local_model_connected and self.local_model_websocket:
                    try:
                        # 发送停止命令到本地模型
                        stop_data = {
                            "type": "stop"
                        }
                        await self.local_model_websocket.send(json.dumps(stop_data, ensure_ascii=False))
                        logger.info("[STOP_MOVE] 已发送停止命令到本地模型")
                        
                        # 清空活跃任务集合
                        async with self.task_execution_lock:
                            self.active_navigation_tasks.clear()
                        logger.info("[STOP_MOVE] 已清空活跃任务集合")
                        
                    except Exception as e:
                        logger.warning(f"[STOP_MOVE] 发送停止命令到本地模型失败: {e}")
                else:
                    logger.warning("[STOP_MOVE] 有活跃任务但本地模型未连接，仅清空任务状态")
                    async with self.task_execution_lock:
                        self.active_navigation_tasks.clear()
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
                        "success": True,
                        "result": success_msg
                    }
                    return result_data
                    
                except Exception as e:
                    error_msg = f"发布速度命令失败: {str(e)}"
                    logger.error(f"[STOP_MOVE] {error_msg}")
                    
                    result_data = {
                        "success": False,
                        "result": error_msg
                    }
                    return result_data
            else:
                # ROS2不可用时的模拟处理
                logger.info("[STOP_MOVE] ROS2不可用，模拟停止移动")
                success_msg = "已模拟停止机器人移动"
                
                result_data = {
                    "success": True,
                    "result": success_msg
                }
                return result_data
                
        except Exception as e:
            error_msg = f"停止移动失败: {str(e)}"
            logger.error(f"[STOP_MOVE] {error_msg}")
            
            result_data = {
                "success": False,
                "result": error_msg
            }
            return result_data

# ======================
# 主程序
# ======================

async def main():
    """主程序入口"""
    print("Smart Robot Agent is running...")
    print(f"WebSocket server listening on {WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
    print("Type 'exit' to quit.")
    
    # 创建智能机器人Agent
    global smart_robot_agent_instance
    agent = SmartRobotAgent()
    smart_robot_agent_instance = agent
    
    # 启动电池电量监控
    agent.ros2_interface.start_battery_monitoring(agent.websocket_server)
    
    # 启动WebSocket服务器
    ws_server = await agent.start_websocket_server()
    
    # 等待服务器关闭
    await ws_server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())























































































