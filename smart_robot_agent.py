# -*- coding: utf-8 -*-
"""A smart robot agent implementation using AgentScope framework."""

import os
import json
import sqlite3
import numpy as np
from datetime import datetime
from typing import Optional, Dict, Any, List
import re
import asyncio
import websockets
import threading
import queue
import logging
import subprocess

import dashscope

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from agentscope.agent import ReActAgent
from agentscope.formatter import DashScopeChatFormatter
from agentscope.memory import InMemoryMemory
from agentscope.model import DashScopeChatModel
from agentscope.tool import Toolkit, ToolResponse
from agentscope.message import Msg, TextBlock, ImageBlock, AudioBlock

# ======================
# 配置
# ======================
dashscope.api_key = os.getenv("DASHSCOPE_API_KEY")
if not dashscope.api_key:
    raise EnvironmentError("DASHSCOPE_API_KEY not set!")

ASM_JSON_PATH = "asm_data.json"

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
VIDEO_BASE_DIR = "/tmp/videos/"
DB_PATH = "/tmp/history.db"
MEMORY_FILE = "/tmp/robot_memory.json"
os.makedirs(VIDEO_BASE_DIR, exist_ok=True)
os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
os.makedirs(os.path.dirname(MEMORY_FILE), exist_ok=True)

# ======================
# Memory管理器
# ======================

class MemoryManager:
    """机器人记忆管理器"""
    
    def __init__(self, memory_file: str = MEMORY_FILE):
        self.memory_file = memory_file
        self.memory_data = self._load_memory()
        self.conversation_history = []  # 对话历史记录
        
    def _load_memory(self) -> Dict[str, Any]:
        """加载记忆数据"""
        if os.path.exists(self.memory_file):
            try:
                with open(self.memory_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception as e:
                logger.error(f"加载记忆文件失败: {e}")
                return {}
        return {}
        
    def _save_memory(self):
        """保存记忆数据"""
        try:
            # 确保目录存在
            os.makedirs(os.path.dirname(self.memory_file), exist_ok=True)
            with open(self.memory_file, 'w', encoding='utf-8') as f:
                json.dump(self.memory_data, f, indent=2, ensure_ascii=False)
        except Exception as e:
            logger.error(f"保存记忆文件失败: {e}")
            
    def store_memory(self, key: str, value: Any):
        """存储记忆信息"""
        # 检查是否已存在相同内容的记忆
        if key in self.memory_data:
            existing_value = self.memory_data[key]["value"]
            # 如果内容相同，不重复存储
            if existing_value == value:
                logger.info(f"记忆信息已存在，跳过存储: {key}")
                return
        
        self.memory_data[key] = {
            "value": value,
            "timestamp": datetime.now().isoformat()
        }
        self._save_memory()
        # logger.info(f"存储记忆信息: {key} = {value}")
        
    def retrieve_memory(self, key: str, default: Any = None) -> Any:
        """
        根据给定的键检索记忆中的值
        
        Args:
            key (str): 要检索的记忆键名
            default (Any): 当键不存在时返回的默认值，默认为None
        
        Returns:
            Any: 与键关联的值，如果键不存在则返回默认值
        
        Raises:
            无显式抛出异常
        """
        if key in self.memory_data:
            value = self.memory_data[key]["value"]
            logger.info(f"检索记忆信息: {key} = {value}")
            return value
        return default
        
    def get_recent_memories(self, limit: int = 5) -> List[Dict[str, Any]]:
        """获取最近的记忆信息"""
        memories = []
        for key, data in self.memory_data.items():
            memories.append({
                "key": key,
                "value": data["value"],
                "timestamp": data["timestamp"]
            })
        # 按时间戳排序，最新的在前面
        memories.sort(key=lambda x: x["timestamp"], reverse=True)
        return memories[:limit]
        
    def clear_memory(self, key: Optional[str] = None):
        """清除记忆信息"""
        if key:
            if key in self.memory_data:
                del self.memory_data[key]
                logger.info(f"清除记忆信息: {key}")
        else:
            self.memory_data.clear()
            logger.info("清除所有记忆信息")
        self._save_memory()
        
    def add_conversation(self, role: str, content: str):
        """添加对话历史记录"""
        conversation_entry = {
            "role": role,
            "content": content,
            "timestamp": datetime.now().isoformat()
        }
        self.conversation_history.append(conversation_entry)
        logger.info(f"添加对话历史: {role} - {content}")
        
    def get_conversation_history(self, limit: int = 10) -> List[Dict[str, str]]:
        """获取对话历史记录"""
        # 返回最近的对话记录
        return self.conversation_history[-limit:] if self.conversation_history else []
        
    def clear_conversation_history(self):
        """清除对话历史记录"""
        self.conversation_history.clear()
        logger.info("清除对话历史记录")
        
    def find_relevant_memories(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """根据查询找到相关的记忆"""
        relevant_memories = []
        query_words = set(query.lower().split())
        
        for key, data in self.memory_data.items():
            # 检查记忆的键和值是否与查询相关
            key_words = set(key.lower().split())
            value_str = str(data["value"]).lower()
            value_words = set(value_str.split())
            
            # 计算相关性得分
            key_overlap = len(query_words.intersection(key_words))
            value_overlap = len(query_words.intersection(value_words))
            relevance_score = key_overlap * 2 + value_overlap  # 给键更高的权重
            
            if relevance_score > 0:
                relevant_memories.append({
                    "key": key,
                    "value": data["value"],
                    "timestamp": data["timestamp"],
                    "relevance_score": relevance_score
                })
        
        # 按相关性得分排序，返回最相关的记忆
        relevant_memories.sort(key=lambda x: x["relevance_score"], reverse=True)
        return relevant_memories[:limit]
        
    def get_memory_patterns(self) -> Dict[str, Any]:
        """获取记忆中的模式和趋势"""
        patterns = {
            "recent_actions": [],
            "frequently_requested": {},
            "common_sequences": []
        }
        
        # 分析最近的行动
        recent_memories = self.get_recent_memories(10)
        for memory in recent_memories:
            if "action" in memory["key"]:
                patterns["recent_actions"].append(memory)
        
        # 分析频繁请求的项目
        for key, data in self.memory_data.items():
            if key.startswith("object_location_") or key.startswith("person_location_"):
                obj_name = key.replace("object_location_", "").replace("person_location_", "")
                patterns["frequently_requested"][obj_name] = patterns["frequently_requested"].get(obj_name, 0) + 1
        
        # 按频率排序
        patterns["frequently_requested"] = dict(
            sorted(patterns["frequently_requested"].items(), key=lambda x: x[1], reverse=True)
        )
        
        return patterns
        
    def store_action_result(self, action: str, result: Any, success: bool):
        """存储行动结果"""
        action_key = f"action_{action}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        action_data = {
            "action": action,
            "result": result,
            "success": success,
            "timestamp": datetime.now().isoformat()
        }
        self.store_memory(action_key, action_data)
        
        # 如果行动成功，更新相关对象的位置记忆
        if success and isinstance(result, dict) and "location" in result:
            # 这里可以添加更新对象位置记忆的逻辑
            pass
        
        return action_key
        
    def extract_and_store_info(self, user_input: str, agent_response: str):
        """从对话中提取有效信息并存储到记忆中"""
        try:
            # 检查是否是找物品/跟随等实际任务，如果是则跳过复杂的信息提取
            task_keywords = ["找", "寻找", "剪刀", "手机", "遥控器", "钥匙", "钱包", "书本", "电视机", "水杯", "跟随", "导航"]
            
            # 如果是找物品或跟随任务，只进行简单的对话记录
            is_task_conversation = any(keyword in user_input for keyword in task_keywords)
            
            if is_task_conversation:
                # 对于任务对话，只记录对话历史，不进行复杂的信息提取
                conversation_pair = {
                    "user": user_input,
                    "agent": agent_response,
                    "timestamp": datetime.now().isoformat(),
                    "type": "task_conversation"
                }
                self.store_memory(f"conversation_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}", conversation_pair)
                logger.info("任务对话已记录，跳过复杂信息提取")
                return
            
            # 对于非任务对话，进行简化版的信息提取
            # 只提取明显的物品名称和人员名称
            import re
            
            # 提取物品名称（简单的关键词匹配）
            object_keywords = ["手机", "遥控器", "钥匙", "钱包", "书本", "电视机", "水杯", "剪刀"]
            found_objects = []
            for obj in object_keywords:
                if obj in user_input:
                    found_objects.append(obj)
            
            # 提取人员名称（简单的模式匹配）
            person_patterns = [
                r"我叫([^，。！？]+)",
                r"我(爸爸|妈妈|哥哥|姐姐|弟弟|妹妹|朋友)叫([^，。！？]+)",
                r"([^，。！？]+)是我(爸爸|妈妈|哥哥|姐姐|弟弟|妹妹|朋友)"
            ]
            found_persons = []
            
            for pattern in person_patterns:
                matches = re.findall(pattern, user_input)
                for match in matches:
                    if isinstance(match, tuple):
                        # 处理多组匹配的情况
                        for item in match:
                            if item and len(item.strip()) > 0:
                                found_persons.append(item.strip())
                    elif match and len(match.strip()) > 0:
                        found_persons.append(match.strip())
            
            # 存储提取到的物品信息
            for obj_name in found_objects:
                self.store_memory(f"object_interest_{obj_name}", {
                    "type": "object",
                    "name": obj_name,
                    "context": user_input,
                    "timestamp": datetime.now().isoformat()
                })
                logger.info(f"检测到物品兴趣: {obj_name}")
            
            # 存储提取到的人员信息
            for person_name in found_persons:
                self.store_memory(f"person_{person_name}", {
                    "type": "person",
                    "name": person_name,
                    "context": user_input,
                    "timestamp": datetime.now().isoformat()
                })
                logger.info(f"检测到人员: {person_name}")
            
            # 存储对话对到记忆中
            conversation_pair = {
                "user": user_input,
                "agent": agent_response,
                "timestamp": datetime.now().isoformat(),
                "type": "normal_conversation"
            }
            self.store_memory(f"conversation_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}", conversation_pair)
            
            logger.info("已从对话中提取并存储简化信息")
        except Exception as e:
            logger.error(f"提取和存储对话信息失败: {e}")

    def extract_user_preferences(self, user_input: str) -> Dict[str, Any]:
        """简化版用户偏好提取（已弃用复杂逻辑）"""
        return {}
        
    def extract_tasks_and_goals(self, user_input: str) -> Dict[str, Any]:
        """简化版任务和目标提取（已弃用复杂逻辑）"""
        return {}
        
    def analyze_conversation_topic(self, user_input: str, agent_response: str) -> Dict[str, Any]:
        """简化版对话主题分析（已弃用复杂逻辑）"""
        return {}

# 格式化函数
def format_conversation_history(conversation_history: List[Dict[str, str]]) -> str:
    """格式化对话历史"""
    if not conversation_history:
        return "暂无对话历史"
    
    formatted_history = []
    for entry in conversation_history:
        role = entry.get("role", "unknown")
        content = entry.get("content", "")
        timestamp = entry.get("timestamp", "")
        formatted_history.append(f"[{role}] {content}")
    
    return "\n".join(formatted_history)

def format_user_interests(memory_manager: MemoryManager) -> str:
    """格式化用户关心的物品和人员"""
    interests = []
    # 添加物品兴趣
    for key, data in memory_manager.memory_data.items():
        if key.startswith("object_interest_"):
            interest_data = data.get("value", {})
            item = interest_data.get("name", "未知物品")
            context = interest_data.get("context", "")
            interests.append(f"- {item}")
    
    # 添加人员信息
    persons = {}
    for key, data in memory_manager.memory_data.items():
        if key.startswith("person_") and not key.endswith(("_hobby_", "_preference_")) and not key.startswith("person_interest_"):
            person_data = data.get("value", {})
            person_name = person_data.get("name", "未知人员")
            if person_name not in persons:
                persons[person_name] = {
                    "relationship": person_data.get("relationship", "未知关系"),
                    "hobbies": [],
                    "preferences": []
                }
    
    # 收集人员的兴趣爱好
    for key, data in memory_manager.memory_data.items():
        if "_hobby_" in key and key.startswith("person_"):
            hobby_data = data.get("value", {})
            person_name = hobby_data.get("person", "未知人员")
            hobby = hobby_data.get("name", "未知爱好")
            if person_name in persons and hobby not in persons[person_name]["hobbies"]:
                persons[person_name]["hobbies"].append(hobby)
    
    # 收集人员的偏好
    for key, data in memory_manager.memory_data.items():
        if "_preference_" in key and key.startswith("person_"):
            preference_data = data.get("value", {})
            person_name = preference_data.get("person", "未知人员")
            preference = preference_data.get("name", "未知偏好")
            if person_name in persons and preference not in persons[person_name]["preferences"]:
                persons[person_name]["preferences"].append(preference)
    
    # 格式化人员信息
    for person_name, person_info in persons.items():
        relationship = person_info["relationship"]
        hobbies = person_info["hobbies"]
        preferences = person_info["preferences"]
        
        interests.append(f"- {person_name} (关系: {relationship})")
        if hobbies:
            interests.append(f"  兴趣爱好: {', '.join(hobbies)}")
        if preferences:
            interests.append(f"  偏好: {', '.join(preferences)}")
    
    if not interests:
        return "暂无用户关心的物品或人员"
    
    return "\n".join(interests)

def format_user_hobbies(memory_manager: MemoryManager) -> str:
    """格式化用户爱好"""
    hobbies = []
    for key, data in memory_manager.memory_data.items():
        if key.startswith("user_hobby_"):
            hobby_data = data.get("value", {})
            hobby = hobby_data.get("name", "未知爱好")
            hobbies.append(f"- {hobby}")
    
    if not hobbies:
        return "暂无用户爱好信息"
    
    return "\n".join(hobbies)

def format_user_preferences(memory_manager: MemoryManager) -> str:
    """格式化用户偏好"""
    preferences = []
    for key, data in memory_manager.memory_data.items():
        if key.startswith("user_preference_"):
            preference_data = data.get("value", {})
            name = preference_data.get("name", "未知偏好")
            context = preference_data.get("context", "")
            preferences.append(f"- {name}")
    
    if not preferences:
        return "暂无用户偏好信息"
    
    return "\n".join(preferences)

def format_user_names(memory_manager: MemoryManager) -> str:
    """格式化用户名"""
    names = []
    for key, data in memory_manager.memory_data.items():
        if key.startswith("user_name_"):
            name_data = data.get("value", {})
            name = name_data.get("name", "未知用户")
            names.append(f"- {name}")
    
    if not names:
        return "暂无用户名信息"
    
    return "\n".join(names)

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# WebSocket配置
WEBSOCKET_HOST = "localhost"
WEBSOCKET_PORT = 8767

# ======================
# 工具函数：外部系统模拟
# ======================

def navigate_to(x: float, y: float) -> bool:
    """模拟导航到指定坐标"""
    print(f"[ROS2] Navigating to ({x:.2f}, {y:.2f})")
    return True

def start_following(person_id: str) -> bool:
    """模拟开始跟随指定人员"""
    print(f"[ROS2] Following {person_id}")
    return True

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
        
        # 获取视频时长
        cap = cv2.VideoCapture(video_path)
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        video_duration = frame_count / fps if fps > 0 else 100.0  # 默认100秒
        cap.release()
        
        # 计算剪切时间（前后30秒）
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
            if obj.get("name") == obj_name:
                # 转换world_position为location格式
                world_pos = obj.get("world_position", [])
                if len(world_pos) >= 2:
                    location = {"x": world_pos[0], "y": world_pos[1]}
                    return {
                        "location": location,
                        "last_time": f"2025-11-02T10:00:00"  # 默认时间，实际应该从数据中获取
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
                id TEXT PRIMARY KEY,
                name TEXT,
                last_time TEXT,
                location TEXT
            )
        """)
        cursor.execute(
            "SELECT id, name, last_time, location FROM objects WHERE name = ? ORDER BY last_time DESC LIMIT 1",
            (obj_name,)
        )
        row = cursor.fetchone()
        conn.close()
        if row:
            logger.info(f"[DB] Found object {obj_name} with id {row[0]} at location {row[3]}")
            return {
                "id": row[0],
                "name": row[1],
                "last_time": row[2],
                "location": json.loads(row[3]) if row[3] else None
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
# DashScope API 调用
# ======================

def call_qwen_text_api(prompt: str, max_retries: int = 3) -> str:
    """纯文本模型（qwen-max）"""
    last_exception = None
    for attempt in range(max_retries):
        try:
            response = dashscope.Generation.call(
                model="qwen3-max",
                prompt=prompt,
                temperature=0.1,
                seed=12345
            )
            # 检查响应是否为字典格式（同步调用）
            if isinstance(response, dict):
                if response.get('status_code') == 200:
                    output = response.get('output', {})
                    if output and isinstance(output, dict):
                        # 优先使用text字段，如果没有则从choices中提取
                        text = output.get('text', '')
                        if not text:
                            choices = output.get('choices', [])
                            if choices and isinstance(choices, list) and len(choices) > 0:
                                message = choices[0].get('message', {})
                                if message and isinstance(message, dict):
                                    text = message.get('content', '')
                        if text:
                            text = text.strip()
                            # 尝试提取 JSON
                            match = re.search(r'\{.*\}', text, re.DOTALL)
                            if match:
                                return match.group(0)
                            else:
                                return text
                        else:
                            raise Exception(f"Empty text in response: {response}")
                    else:
                        raise Exception(f"Invalid output in response: {response}")
                else:
                    raise Exception(f"API error: {response}")
            else:
                # 处理其他可能的响应格式
                status_code = getattr(response, 'status_code', None)
                if status_code == 200:
                    output = getattr(response, 'output', None)
                    if output:
                        # 优先使用text属性，如果没有则从choices中提取
                        text = getattr(output, 'text', '')
                        if not text:
                            choices = getattr(output, 'choices', [])
                            if choices and isinstance(choices, list) and len(choices) > 0:
                                message = choices[0].get('message', {})
                                if message and isinstance(message, dict):
                                    text = message.get('content', '')
                        if text:
                            text = text.strip()
                            # 尝试提取 JSON
                            match = re.search(r'\{.*\}', text, re.DOTALL)
                            if match:
                                return match.group(0)
                            else:
                                return text
                        else:
                            raise Exception(f"Empty text in response: {response}")
                    else:
                        raise Exception(f"Missing 'output' in response: {response}")
                else:
                    raise Exception(f"API error with status code: {status_code}")
        except Exception as e:
            logger.error(f"[ERROR] Text API call failed (attempt {attempt + 1}/{max_retries}): {e}")
            last_exception = e
            if attempt == max_retries - 1:
                raise
            # 等待一段时间再重试
            import time
            time.sleep(2 ** attempt)  # 指数退避
    # 如果所有重试都失败，抛出最后一个异常
    if last_exception:
        raise last_exception
    # 默认返回空字符串（理论上不会执行到这里）
    return ""

def call_qwen_vl_api(prompt: str, max_retries: int = 3) -> str:
    """纯文本模型（qwen3-max）"""
    # 由于输入只有自然语言，没有图片，所以使用纯文本模型
    return call_qwen_text_api(prompt, max_retries)

def call_qwen_vl_api_with_video(prompt: str, video_path: str, max_retries: int = 3) -> str:
    """使用qwen3-vl-plus模型处理视频输入"""
    last_exception = None
    for attempt in range(max_retries):
        try:
            # 使用dashscope处理视频
            messages = [
                {
                    "role": "user",
                    "content": [
                        {"video": video_path},
                        {"text": prompt}
                    ]
                }
            ]
            
            response = dashscope.MultiModalConversation.call(
                model="qwen3-vl-plus",
                messages=messages,
                temperature=0.1,
                seed=12345
            )
            
            # 检查响应是否为字典格式
            if isinstance(response, dict):
                if response.get('status_code') == 200:
                    output = response.get('output', {})
                    if output and isinstance(output, dict):
                        # 从choices中提取文本内容
                        choices = output.get('choices', [])
                        if choices and isinstance(choices, list) and len(choices) > 0:
                            message = choices[0].get('message', {})
                            if message and isinstance(message, dict):
                                content = message.get('content', '')
                                if content:
                                    # 如果content是列表格式，提取其中的文本
                                    if isinstance(content, list):
                                        texts = []
                                        for item in content:
                                            if isinstance(item, dict) and item.get('text'):
                                                texts.append(item['text'])
                                        return ''.join(texts)
                                    else:
                                        # 如果是字符串格式，直接返回
                                        return str(content)
                                else:
                                    raise Exception(f"Empty content in response: {response}")
                        else:
                            raise Exception(f"Invalid choices in response: {response}")
                    else:
                        raise Exception(f"Invalid output in response: {response}")
                else:
                    raise Exception(f"API error: {response}")
            else:
                # 处理其他可能的响应格式
                status_code = getattr(response, 'status_code', None)
                if status_code == 200:
                    output = getattr(response, 'output', None)
                    if output:
                        # 从choices中提取文本内容
                        choices = getattr(output, 'choices', [])
                        if choices and isinstance(choices, list) and len(choices) > 0:
                            message = choices[0].get('message', {})
                            if message and isinstance(message, dict):
                                content = message.get('content', '')
                                if content:
                                    # 如果content是列表格式，提取其中的文本
                                    if isinstance(content, list):
                                        texts = []
                                        for item in content:
                                            if isinstance(item, dict) and item.get('text'):
                                                texts.append(item['text'])
                                        return ''.join(texts)
                                    else:
                                        # 如果是字符串格式，直接返回
                                        return str(content)
                                else:
                                    raise Exception(f"Empty content in response: {response}")
                        else:
                            raise Exception(f"Invalid choices in response: {response}")
                    else:
                        raise Exception(f"Missing 'output' in response: {response}")
                else:
                    raise Exception(f"API error with status code: {status_code}")
        except Exception as e:
            logger.error(f"[ERROR] VL API call with video failed (attempt {attempt + 1}/{max_retries}): {e}")
            last_exception = e
            if attempt == max_retries - 1:
                raise
            # 等待一段时间再重试
            import time
            time.sleep(2 ** attempt)  # 指数退避
    # 如果所有重试都失败，抛出最后一个异常
    if last_exception:
        raise last_exception
    # 默认返回空字符串（理论上不会执行到这里）
    return ""

# ======================
# Tools
# ======================

def find_object(obj_name: str) -> ToolResponse:
    """
    查找物品的位置信息，按照ASM→DB→探索的优先级执行
    
    Args:
        obj_name (str): 物品名称
    
    Returns:
        ToolResponse: 工具响应对象
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
            
            return ToolResponse(
                content=[{"type": "text", "text": result_msg}],
                metadata={
                    "status": "success", 
                    "result": result_msg, 
                    "location": loc,
                    "needs_navigation_confirmation": True,  # 只有ASM找到时才询问导航
                    "source": "asm"
                }
            )

        # Step 2: DB查询
        db_res = query_history_db(obj_name)
        if not db_res:
            # DB没有找到：询问用户是否要探索寻找
            error_msg = f"常规查找未找到物品/人员 {obj_name}"
            logger.warning(f"[FIND_OBJECT] {error_msg}")
            return ToolResponse(
                content=[{"type": "text", "text": error_msg}],
                metadata={
                    "status": "not_found", 
                    "result": error_msg,
                    "needs_exploration_confirmation": True,  # 询问是否要探索寻找
                    "source": "none"
                }
            )

        logger.info(f"[FIND_OBJECT] 在DB中找到 {obj_name} 记录，时间: {db_res['last_time']}")

        # Step 3: 视频剪切（前后3秒）
        last_time = db_res["last_time"]
        raw_video = get_video_path_by_time(last_time)
        cut_video_path = cut_video_6s(raw_video, last_time, VIDEO_BASE_DIR)
        if not cut_video_path:
            error_msg = "视频处理失败"
            logger.error(f"[FIND_OBJECT] {error_msg}")
            return ToolResponse(
                content=[{"type": "text", "text": error_msg}],
                metadata={
                    "status": "error", 
                    "result": error_msg,
                    "needs_exploration_confirmation": True,  # 视频处理失败也询问是否要探索
                    "source": "db"
                }
            )

        logger.info(f"[FIND_OBJECT] 视频剪切完成: {cut_video_path}")

        # Step 4: VL 推理（使用qwen3-vl-plus模型处理6秒视频片段）
        prompt = f"根据视频分析，用一句话描述 '{obj_name}' 的具体位置。只回答位置，不要其他内容。"
        logger.info(f"[FIND_OBJECT] 调用VL模型进行位置识别")
        try:
            desc = call_qwen_vl_api_with_video(prompt, cut_video_path)
            if desc:
                result_msg = f"找到 {obj_name} 位置: {desc}"
                logger.info(f"[FIND_OBJECT] {result_msg}")
                
                # DB找到：直接反馈结果，不询问导航
                return ToolResponse(
                    content=[{"type": "text", "text": result_msg}],
                    metadata={
                        "status": "success", 
                        "result": desc,
                        "needs_navigation_confirmation": False,  # DB找到不询问导航
                        "source": "db"
                    }
                )
            else:
                error_msg = "视觉推理未返回有效结果"
                logger.warning(f"[FIND_OBJECT] {error_msg}")
                return ToolResponse(
                    content=[{"type": "text", "text": error_msg}],
                    metadata={
                        "status": "error", 
                        "result": error_msg,
                        "needs_exploration_confirmation": True,  # 推理失败询问是否要探索
                        "source": "db"
                    }
                )
        except Exception as e:
            error_msg = "视觉推理失败"
            logger.error(f"[FIND_OBJECT] {error_msg}: {e}")
            return ToolResponse(
                content=[{"type": "text", "text": error_msg}],
                metadata={
                    "status": "error", 
                    "result": error_msg,
                    "needs_exploration_confirmation": True,  # 推理失败询问是否要探索
                    "source": "db"
                }
            )
    except Exception as e:
        error_msg = f"查找物品/人员失败: {str(e)}"
        logger.error(f"[FIND_OBJECT] {error_msg}")
        return ToolResponse(
            content=[{"type": "text", "text": error_msg}],
            metadata={"status": "error", "result": error_msg}
        )

def go_to_object(location_info: str) -> ToolResponse:
    """
    根据位置信息导航到物品位置，通过ROS2导航action发送目标点
    
    Args:
        location_info (str): 物品位置信息
    
    Returns:
        ToolResponse: 工具响应对象
    """
    logger.info(f"[GO_TO_OBJECT] 开始导航到位置: {location_info}")
    
    try:
        # 位置信息到坐标的映射（这里需要根据实际环境配置）
        location_to_coordinates = {
            "厨房抽屉里": (1.5, 2.0),
            "客厅沙发上": (3.0, 1.5),
            "卧室床头": (0.5, 3.0),
            "书房桌子上": (2.5, 4.0),
            "电视柜上": (4.0, 1.0),
            "玄关柜子上": (0.0, 0.0),
            "工作台上": (2.0, 3.5),
            "客厅茶几上": (3.5, 2.5),
            "卧室床头柜": (0.8, 2.8),
            "厨房台面上": (1.2, 1.8)
        }
        
        # 获取坐标
        if location_info in location_to_coordinates:
            x, y = location_to_coordinates[location_info]
            
            # 调用ROS2导航action
            logger.info(f"[GO_TO_OBJECT] 调用ROS2导航action，目标坐标: ({x}, {y})")
            
            # 这里应该调用实际的ROS2导航action
            # 暂时使用模拟的navigate_to函数
            result = navigate_to(x, y)
            
            if result:
                success_msg = f"已通过ROS2导航action成功导航到: {location_info} (坐标: {x}, {y})"
                logger.info(f"[GO_TO_OBJECT] {success_msg}")
                return ToolResponse(
                    content=[{"type": "text", "text": success_msg}],
                    metadata={"status": "success", "result": success_msg}
                )
            else:
                error_msg = f"ROS2导航action执行失败: {location_info}"
                logger.error(f"[GO_TO_OBJECT] {error_msg}")
                return ToolResponse(
                    content=[{"type": "text", "text": error_msg}],
                    metadata={"status": "error", "result": error_msg}
                )
        else:
            # 如果位置信息不在映射中，尝试解析坐标格式
            # 支持格式: "坐标(1.5, 2.0)" 或 "1.5, 2.0"
            import re
            coord_pattern = r"[\[\(]?\s*(\d+\.?\d*)\s*,\s*(\d+\.?\d*)\s*[\]\)]?"
            match = re.search(coord_pattern, location_info)
            
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                
                logger.info(f"[GO_TO_OBJECT] 解析到坐标: ({x}, {y})")
                
                # 调用ROS2导航action
                result = navigate_to(x, y)
                
                if result:
                    success_msg = f"已通过ROS2导航action成功导航到坐标: ({x}, {y})"
                    logger.info(f"[GO_TO_OBJECT] {success_msg}")
                    return ToolResponse(
                        content=[{"type": "text", "text": success_msg}],
                        metadata={"status": "success", "result": success_msg}
                    )
                else:
                    error_msg = f"ROS2导航action执行失败: 坐标({x}, {y})"
                    logger.error(f"[GO_TO_OBJECT] {error_msg}")
                    return ToolResponse(
                        content=[{"type": "text", "text": error_msg}],
                        metadata={"status": "error", "result": error_msg}
                    )
            else:
                error_msg = f"无法识别的位置信息格式: {location_info}"
                logger.error(f"[GO_TO_OBJECT] {error_msg}")
                return ToolResponse(
                    content=[{"type": "text", "text": error_msg}],
                    metadata={"status": "error", "result": error_msg}
                )
            
    except Exception as e:
        error_msg = f"导航失败: {str(e)}"
        logger.error(f"[GO_TO_OBJECT] {error_msg}")
        return ToolResponse(
            content=[{"type": "text", "text": error_msg}],
            metadata={"status": "error", "result": error_msg}
        )

def follow_person(location_info: str) -> ToolResponse:
    """
    
    跟随指定人员
    
    Args:
        location_info (str): 人员位置信息
    
    Returns:
        ToolResponse: 工具响应对象
    """
    logger.info(f"[FOLLOW_PERSON] 开始跟随人员，位置信息: {location_info}")
    try:
        # 提取人员名称（假设位置信息格式为"跟随 [name]"）
        name = "person"
        if "跟随" in location_info:
            parts = location_info.split(" ")
            if len(parts) > 1:
                name = parts[1]
        
        # 存储跟随信息到记忆中
        memory_manager.store_memory(f"following_person", {
            "name": name,
            "location": location_info,
            "timestamp": datetime.now().isoformat()
        })
        
        # 开始跟随
        logger.info(f"[FOLLOW_PERSON] 调用跟随模块跟随人员: {name}")
        result = start_following(f"id_{name}")
        if result:
            success_msg = f"开始跟随 {name}"
            logger.info(f"[FOLLOW_PERSON] {success_msg}")
            # 存储对话历史
            memory_manager.add_conversation("system", f"开始跟随 {name}")
            return ToolResponse(
                content=[{"type": "text", "text": success_msg}],
                metadata={"status": "success", "result": success_msg}
            )
        else:
            error_msg = "跟随失败"
            logger.error(f"[FOLLOW_PERSON] {error_msg}")
            memory_manager.add_conversation("system", f"跟随 {name} 失败")
            return ToolResponse(
                content=[{"type": "text", "text": error_msg}],
                metadata={"status": "error", "result": error_msg}
            )
    except Exception as e:
        error_msg = f"跟随失败: {str(e)}"
        logger.error(f"[FOLLOW_PERSON] {error_msg}")
        memory_manager.add_conversation("system", f"跟随失败: {str(e)}")
        return ToolResponse(
            content=[{"type": "text", "text": error_msg}],
            metadata={"status": "error", "result": error_msg}
        )

def stop_follow() -> ToolResponse:
    """
    停止跟随
    
    Returns:
        ToolResponse: 工具响应对象
    """
    try:
        # 获取当前跟随的人员信息
        following_info = memory_manager.retrieve_memory("following_person", {})
        following_name = following_info.get("name", "未知人员")
        
        result = stop_following()
        if result:
            notify_navigation_model_stop_following()
            success_msg = "已停止跟随"
            # 存储对话历史
            memory_manager.add_conversation("system", f"已停止跟随 {following_name}")
            # 清除跟随信息
            memory_manager.clear_memory("following_person")
            return ToolResponse(
                content=[{"type": "text", "text": success_msg}],
                metadata={"status": "success", "result": success_msg}
            )
        else:
            error_msg = "停止跟随失败"
            memory_manager.add_conversation("system", f"停止跟随 {following_name} 失败")
            return ToolResponse(
                content=[{"type": "text", "text": error_msg}],
                metadata={"status": "error", "result": error_msg}
            )
    except Exception as e:
        error_msg = f"停止跟随失败: {str(e)}"
        memory_manager.add_conversation("system", f"停止跟随失败: {str(e)}")
        return ToolResponse(
            content=[{"type": "text", "text": error_msg}],
            metadata={"status": "error", "result": error_msg}
        )

def stop_navigate() -> ToolResponse:
    """
    停止导航
    
    Returns:
        ToolResponse: 工具响应对象
    """
    try:
        result = stop_navigation()
        if result:
            success_msg = "已停止导航"
            # 存储对话历史
            memory_manager.add_conversation("system", success_msg)
            return ToolResponse(
                content=[{"type": "text", "text": success_msg}],
                metadata={"status": "success", "result": success_msg}
            )
        else:
            error_msg = "停止导航失败"
            memory_manager.add_conversation("system", error_msg)
            return ToolResponse(
                content=[{"type": "text", "text": error_msg}],
                metadata={"status": "error", "result": error_msg}
            )
    except Exception as e:
        error_msg = f"停止导航失败: {str(e)}"
        memory_manager.add_conversation("system", error_msg)
        return ToolResponse(
            content=[{"type": "text", "text": error_msg}],
            metadata={"status": "error", "result": error_msg}
        )

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
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS objects (
            id TEXT PRIMARY KEY,
            name TEXT,
            last_time TEXT,
            location TEXT
        )
    """)
    
    # 插入更多测试数据，包括电视机和人员
    test_objects = [
        ("obj_001", "遥控器", "2025-11-02T09:30:00", json.dumps({"x": 2.1, "y": 1.5})),
        ("obj_002", "钥匙", "2025-11-02T08:15:00", json.dumps({"x": 0.5, "y": 3.2})),
        ("obj_003", "书本", "2025-11-02T07:45:00", json.dumps({"x": 4.0, "y": 1.0})),
        ("obj_004", "钱包", "2025-11-02T06:30:00", json.dumps({"x": 1.8, "y": 4.5})),
        ("obj_005", "电视机", "2025-11-02T11:00:00", json.dumps({"x": 3.5, "y": 2.0})),
        ("obj_006", "小明", "2025-11-02T10:30:00", json.dumps({"x": 1.0, "y": 1.0}))
    ]
    
    for obj_id, name, last_time, location in test_objects:
        cursor.execute("""
            INSERT OR REPLACE INTO objects (id, name, last_time, location)
            VALUES (?, ?, ?, ?)
        """, (obj_id, name, last_time, location))
        logger.info(f"[INIT] Inserted object {name} with id {obj_id} at location {location}")
    
    conn.commit()
    conn.close()

    # 测试视频
    test_video = os.path.join(VIDEO_BASE_DIR, "20251102.mp4")
    if not os.path.exists(test_video):
        fourcc = cv2.VideoWriter.fourcc(*'mp4v')
        out = cv2.VideoWriter(test_video, fourcc, 1, (640, 480))
        for _ in range(20):
            frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            out.write(frame)
        out.release()

# ======================
# Agent实现
# ======================

# 全局MemoryManager实例
memory_manager = MemoryManager()

def create_smart_robot_agent():
    """创建智能机器人agent"""
    # 创建工具包
    toolkit = Toolkit()
    toolkit.register_tool_function(find_object)
    toolkit.register_tool_function(go_to_object)
    toolkit.register_tool_function(follow_person)
    toolkit.register_tool_function(stop_follow)
    toolkit.register_tool_function(stop_navigate)
    
    # 获取API密钥
    api_key = os.environ.get("DASHSCOPE_API_KEY") or dashscope.api_key
    if not api_key:
        raise ValueError("DashScope API key not found")
    
    # 创建agent
    agent = ReActAgent(
        name="SmartRobotAgent",
        sys_prompt=f"""你是一个智能机器人助手，能够执行以下操作：
1. 查找物品 (find_object): 根据物品名称查找其位置
2. 导航到物品 (go_to_object): 根据位置描述导航到物品
3. 跟随人员 (follow_person): 跟随指定人员
4. 停止跟随 (stop_follow): 停止跟随行为
5. 停止导航 (stop_navigate): 停止导航行为

请根据用户指令选择合适的工具来完成任务。你可以组合使用多个工具来完成复杂的任务。
例如，当用户说"找到遥控器然后拿给我"时，你可以先使用find_object工具找到遥控器，然后使用go_to_object工具导航到遥控器位置。

你可以记住之前对话中的信息，包括用户关心的物品和之前的对话内容。这可以帮助你更好地理解和响应用户的请求。

之前的对话历史：
{format_conversation_history(memory_manager.get_conversation_history())}

用户关心的物品和人员：
{format_user_interests(memory_manager)}

用户爱好：
{format_user_hobbies(memory_manager)}

用户偏好：
{format_user_preferences(memory_manager)}

用户名：
{format_user_names(memory_manager)}
""",
        model=DashScopeChatModel(
            api_key=api_key,
            model_name="qwen3-max",
            enable_thinking=True,
            stream=True,
        ),
        formatter=DashScopeChatFormatter(),
        toolkit=toolkit,
        memory=InMemoryMemory(),
    )
    
    return agent

# ======================
# WebSocket服务器
# ======================

class WebSocketServer:
    def __init__(self, agent):
        self.agent = agent
        self.clients = set()
        
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
            user_input = data.get("text", "")
            
            if not user_input:
                await websocket.send(json.dumps({"error": "Empty message"}))
                return
                
            logger.info(f"Received message from client: {user_input}")
            
            # 创建消息对象
            msg = Msg("user", user_input, "user")
            
            # 让agent处理消息
            result = await self.agent(msg)
            
            # 处理结果
            if hasattr(result, 'content'):
                # 如果result有content属性，直接获取文本内容
                if isinstance(result.content, list):
                    texts = []
                    for block in result.content:
                        # 根据不同的block类型提取文本
                        if isinstance(block, dict) and 'text' in block:
                            texts.append(block['text'])
                        elif hasattr(block, 'text'):
                            # 对于具有text属性的对象，直接访问属性
                            texts.append(str(getattr(block, 'text', block)))
                        elif hasattr(block, '__str__'):
                            texts.append(str(block))
                        else:
                            # 对于其他类型，尝试转换为字符串
                            texts.append(str(block))
                    response = ''.join(texts)
                else:
                    response = str(result.content)
            else:
                # 否则尝试使用get_text_content方法
                response = result.get_text_content()
                
            # 发送响应给客户端
            await websocket.send(json.dumps({"response": response}))
            
            # 同时广播给所有客户端
            await self.send_to_clients(json.dumps({"response": response}))
            
            # 只提取一次对话信息（避免重复提取）
            # 检查是否已经处理过相同的对话
            conversation_key = f"conversation_{user_input[:50]}"  # 使用输入内容的前50个字符作为键
            if not memory_manager.retrieve_memory(conversation_key):
                memory_manager.extract_and_store_info(user_input, response)
                # 标记为已处理
                memory_manager.store_memory(conversation_key, {
                    "processed": True,
                    "timestamp": datetime.now().isoformat()
                })
            
        except json.JSONDecodeError:
            await websocket.send(json.dumps({"error": "Invalid JSON format"}))
        except Exception as e:
            logger.error(f"Error handling message: {e}")
            await websocket.send(json.dumps({"error": str(e)}))
            
    async def websocket_handler(self, websocket):
        """WebSocket处理函数"""
        # 注册客户端
        await self.register_client(websocket)
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            # 注销客户端
            await self.unregister_client(websocket)

# ======================
# 终端输入处理
# ======================

class TerminalInputHandler:
    def __init__(self, agent):
        self.agent = agent
        
    async def handle_input(self):
        """处理终端输入"""
        print("Terminal input mode. Type 'exit' to quit.")
        while True:
            user_input = ""
            response = ""
            try:
                user_input = await asyncio.get_event_loop().run_in_executor(None, input, ">>> ")
                if user_input.lower() == 'exit':
                    # 存储对话历史到记忆中
                    memory_manager.store_memory("conversation_history", memory_manager.conversation_history)
                    print("对话历史已保存到记忆文件中。")
                    # 退出程序
                    print("正在退出程序...")
                    os._exit(0)  # 强制退出程序
                    break
                    
                # 创建消息对象
                msg = Msg("user", user_input, "user")
                
                # 让agent处理消息
                result = await self.agent(msg)
                
                # 处理结果
                if hasattr(result, 'content'):
                    # 如果result有content属性，直接获取文本内容
                    if isinstance(result.content, list):
                        texts = []
                        for block in result.content:
                            # 根据不同的block类型提取文本
                            if isinstance(block, dict) and 'text' in block:
                                texts.append(block['text'])
                            elif hasattr(block, 'text'):
                                # 对于具有text属性的对象，直接访问属性
                                texts.append(str(getattr(block, 'text', block)))
                            elif hasattr(block, '__str__'):
                                texts.append(str(block))
                            else:
                                # 对于其他类型，尝试转换为字符串
                                texts.append(str(block))
                        response = ''.join(texts)
                    else:
                        response = str(result.content)
                else:
                    # 否则尝试使用get_text_content方法
                    response = result.get_text_content()
                    
                print(f"<<< {response}")
                
            except EOFError:
                break
            except Exception as e:
                logger.error(f"Error handling terminal input: {e}")
                print(f"Error: {e}")
            
            # 提取并存储对话中的有效信息
            memory_manager.extract_and_store_info(user_input, response)

# ======================
# 主程序
# ======================

async def start_websocket_server(websocket_server):
    """启动WebSocket服务器"""
    server = await websockets.serve(
        websocket_server.websocket_handler,
        WEBSOCKET_HOST,
        WEBSOCKET_PORT
    )
    logger.info(f"WebSocket server started on {WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
    return server

async def main():
    """主程序入口"""
    # 初始化测试数据
    init_test_data()
    
    # 创建智能机器人agent
    agent = create_smart_robot_agent()
    
    # 创建WebSocket服务器
    websocket_server = WebSocketServer(agent)
    
    # 启动WebSocket服务器
    ws_server = await start_websocket_server(websocket_server)
    
    # 创建终端输入处理器
    terminal_handler = TerminalInputHandler(agent)
    
    # 同时运行WebSocket服务器和终端输入处理器
    print("Smart Robot Agent is running...")
    print(f"WebSocket server listening on {WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
    print("You can also interact via terminal input.")
    print("Type 'exit' in terminal to quit.")
    
    # 同时运行WebSocket服务器和终端输入处理
    await asyncio.gather(
        ws_server.wait_closed(),
        terminal_handler.handle_input()
    )

# ======================
# 新增工具方法：探索寻找目标物体/人
# ======================

def explore_and_find_object(obj_name: str) -> ToolResponse:
    """
    通过WebSocket客户端与服务器通信，探索寻找目标物体/人
    注意：此方法已经包含了导航过程，如果成功找到目标，则不需要再调用go_to_object
    
    Args:
        obj_name (str): 要寻找的物品/人员名称
    
    Returns:
        ToolResponse: 工具响应对象
    """
    logger.info(f"[EXPLORE_FIND] 开始探索寻找: {obj_name}")
    
    def sync_explore_find():
        """同步探索寻找函数，使用同步WebSocket客户端"""
        try:
            # WebSocket服务器配置
            websocket_host = "localhost"
            websocket_port = 8765
            uri = f"ws://{websocket_host}:{websocket_port}"
            
            logger.info(f"[EXPLORE_FIND] 连接到WebSocket服务器: {uri}")
            
            # 使用线程安全的WebSocket连接
            import websockets.sync.client as ws_client
            
            with ws_client.connect(uri) as websocket:
                logger.info("[EXPLORE_FIND] 连接成功")
                
                # 发送探索寻找指令
                exploration_command = {
                    "type": "exploration",
                    "target": obj_name,
                    "action": "find_object",
                    "timestamp": datetime.now().isoformat()
                }
                
                websocket.send(json.dumps(exploration_command))
                logger.info(f"[EXPLORE_FIND] 发送探索指令: {exploration_command}")
                
                # 持续接收服务器响应（包括进度更新和最终结果）
                final_result = None
                progress_messages = []
                
                while True:
                    response = websocket.recv()
                    logger.info(f"[EXPLORE_FIND] 收到服务器响应: {response}")
                    
                    # 解析响应
                    response_data = json.loads(response)
                    
                    # 检查消息类型
                    if response_data.get("type") == "progress":
                        # 进度更新消息
                        progress_msg = response_data.get("message", "")
                        progress_messages.append(progress_msg)
                        logger.info(f"[EXPLORE_FIND] 进度更新: {progress_msg}")
                        
                    elif response_data.get("type") == "result":
                        # 最终结果消息
                        final_result = response_data
                        break
                        
                    elif response_data.get("type") == "navigation_progress":
                        # 导航进度消息（暂时忽略）
                        pass
                        
                    else:
                        # 其他类型的消息作为最终结果
                        final_result = response_data
                        break
                
                return {
                    "final_result": final_result,
                    "progress_messages": progress_messages
                }
                
        except Exception as e:
            logger.error(f"[EXPLORE_FIND] WebSocket通信失败: {e}")
            return {"status": "error", "message": f"探索寻找失败: {str(e)}"}
    
    try:
        # 运行同步探索函数
        result = sync_explore_find()
        
        if "error" in result:
            # 连接或通信错误
            error_msg = result.get("message", "探索寻找失败")
            logger.error(f"[EXPLORE_FIND] {error_msg}")
            return ToolResponse(
                content=[{"type": "text", "text": error_msg}],
                metadata={
                    "status": "error", 
                    "result": error_msg
                }
            )
        
        # 获取最终结果
        final_result = result.get("final_result", {})
        progress_messages = result.get("progress_messages", [])
        
        if final_result.get("status") == "success":
            # 探索成功，返回结果
            success_msg = f"探索寻找 {obj_name} 成功: {final_result.get('message', '已找到目标')}"
            logger.info(f"[EXPLORE_FIND] {success_msg}")
            
            # 构建包含进度信息的完整响应
            full_response = success_msg
            if progress_messages:
                full_response += "\n\n探索过程:"
                for i, msg in enumerate(progress_messages, 1):
                    full_response += f"\n{i}. {msg}"
            
            return ToolResponse(
                content=[{"type": "text", "text": full_response}],
                metadata={
                    "status": "success", 
                    "result": success_msg,
                    "exploration_data": final_result,
                    "progress_messages": progress_messages,
                    "already_navigated": True  # 标记已经完成导航
                }
            )
        elif final_result.get("status") == "not_found":
            # 未找到目标
            not_found_msg = f"探索寻找 {obj_name} 未找到: {final_result.get('message', '未找到目标')}"
            logger.info(f"[EXPLORE_FIND] {not_found_msg}")
            
            # 构建包含建议的完整响应
            full_response = not_found_msg
            suggestions = final_result.get("suggestions", [])
            if suggestions:
                full_response += "\n\n建议:"
                for i, suggestion in enumerate(suggestions, 1):
                    full_response += f"\n{i}. {suggestion}"
            
            return ToolResponse(
                content=[{"type": "text", "text": full_response}],
                metadata={
                    "status": "not_found", 
                    "result": not_found_msg,
                    "suggestions": suggestions
                }
            )
        else:
            # 其他状态或错误
            error_msg = final_result.get("message", "探索寻找失败")
            logger.error(f"[EXPLORE_FIND] {error_msg}")
            return ToolResponse(
                content=[{"type": "text", "text": error_msg}],
                metadata={
                    "status": "error", 
                    "result": error_msg
                }
            )
            
    except Exception as e:
        error_msg = f"探索寻找失败: {str(e)}"
        logger.error(f"[EXPLORE_FIND] {error_msg}")
        return ToolResponse(
            content=[{"type": "text", "text": error_msg}],
            metadata={"status": "error", "result": error_msg}
        )

# ======================
# 更新工具注册
# ======================

def create_smart_robot_agent():
    """创建智能机器人agent"""
    # 创建工具包
    toolkit = Toolkit()
    toolkit.register_tool_function(find_object)
    toolkit.register_tool_function(go_to_object)
    toolkit.register_tool_function(follow_person)
    toolkit.register_tool_function(stop_follow)
    toolkit.register_tool_function(stop_navigate)
    toolkit.register_tool_function(explore_and_find_object)  # 新增探索寻找工具
    
    # 获取API密钥
    api_key = os.environ.get("DASHSCOPE_API_KEY") or dashscope.api_key
    if not api_key:
        raise ValueError("DashScope API key not found")
    
    # 创建agent
    agent = ReActAgent(
        name="SmartRobotAgent",
        sys_prompt=f"""你是一个智能机器人助手，能够执行以下操作：
1. 查找物品 (find_object): 根据物品名称查找其位置
2. 导航到物品 (go_to_object): 根据位置描述导航到物品
3. 跟随人员 (follow_person): 跟随指定人员
4. 停止跟随 (stop_follow): 停止跟随行为
5. 停止导航 (stop_navigate): 停止导航行为
6. 探索寻找 (explore_and_find_object): 通过探索方式寻找目标

请根据用户指令选择合适的工具来完成任务。你必须严格按照以下流程执行：

**标准查找流程：**
1. **优先使用find_object工具**：它会按照ASM→DB→探索的优先级自动执行
2. **ASM找到**：如果ASM找到物品位置，会询问用户是否需要导航过去
3. **DB找到**：如果DB有记录，会进行视频切片推理，反馈结果
4. **探索寻找**：如果ASM和DB都没有找到，会询问用户是否要探索寻找

**关键规则：**
- **只有ASM找到物品时**，才会询问用户是否需要导航
- **探索寻找找到物品后**，直接反馈结果，不调用导航，不询问导航
- **DB找到物品后**，直接反馈结果，不询问导航
- **导航方法(go_to_object)只在ASM找到物品且用户确认需要导航时使用**

**工具使用场景：**
- **find_object**: 当用户说"找剪刀"、"寻找手机"等寻找物品的指令时使用（会自动执行完整流程）
- **go_to_object**: 只在ASM找到物品且用户确认需要导航时使用
- **explore_and_find_object**: 只在find_object工具询问是否要探索寻找且用户确认时使用

**示例流程：**
1. 用户说"找剪刀" → 使用find_object工具
   - ASM找到剪刀位置 → 询问"找到剪刀在厨房抽屉，需要我导航过去吗？"
   - 用户说"需要" → 使用go_to_object工具导航
   - 用户说"不需要" → 结束

2. 用户说"找手机" → 使用find_object工具
   - ASM没找到 → DB找到记录 → 视频切片推理 → 反馈"找到手机在客厅沙发上"
   - 不询问导航，直接结束

3. 用户说"找钥匙" → 使用find_object工具
   - ASM没找到 → DB没找到 → 询问"常规查找没找到钥匙，是否要探索寻找？"
   - 用户说"要" → 使用explore_and_find_object工具
   - 探索找到钥匙 → 反馈"探索找到钥匙在玄关柜子上"
   - 不询问导航，直接结束

你可以记住之前对话中的信息，包括用户关心的物品和之前的对话内容。这可以帮助你更好地理解和响应用户的请求。

之前的对话历史：
{format_conversation_history(memory_manager.get_conversation_history())}

用户关心的物品和人员：
{format_user_interests(memory_manager)}

用户爱好：
{format_user_hobbies(memory_manager)}

用户偏好：
{format_user_preferences(memory_manager)}

用户名：
{format_user_names(memory_manager)}
""",
        model=DashScopeChatModel(
            api_key=api_key,
            model_name="qwen3-max",
            enable_thinking=True,
            stream=True,
        ),
        formatter=DashScopeChatFormatter(),
        toolkit=toolkit,
        memory=InMemoryMemory()
    )
    
    return agent

if __name__ == "__main__":
    asyncio.run(main())















































































