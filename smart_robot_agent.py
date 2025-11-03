# -*- coding: utf-8 -*-
"""A smart robot agent implementation using AgentScope framework."""

import os
import json
import sqlite3
import cv2
import numpy as np
from datetime import datetime
from typing import Optional, Dict, Any, List
import re
import asyncio

import dashscope

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

ASM_JSON_PATH = "/tmp/asm_data.json"
VIDEO_BASE_DIR = "/tmp/videos/"
DB_PATH = "/tmp/history.db"
os.makedirs(VIDEO_BASE_DIR, exist_ok=True)

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

def cut_video(video_path: str, timestamp: str, output_dir: str) -> Optional[str]:
    """剪切视频前后1分钟（简化：复制原视频，实际应调用 ffmpeg）"""
    try:
        dt = datetime.fromisoformat(timestamp.replace("Z", "+00:00"))
        output_name = f"cut_{dt.strftime('%Y%m%d_%H%M%S')}.mp4"
        output_path = os.path.join(output_dir, output_name)
        # 简化：直接复制（实际项目用 ffmpeg）
        if os.path.exists(video_path):
            import shutil
            shutil.copy(video_path, output_path)
            return output_path
        else:
            # 创建一个空视频用于测试
            fourcc = cv2.VideoWriter.fourcc(*'mp4v')
            out = cv2.VideoWriter(output_path, fourcc, 1, (640, 480))
            for _ in range(10):
                frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
                out.write(frame)
            out.release()
            return output_path
    except Exception as e:
        print(f"[ERROR] Video cut failed: {e}")
        return None

def extract_key_frame(video_path: str) -> str:
    """从视频中提取中间帧作为关键帧"""
    cap = cv2.VideoCapture(video_path)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if total_frames <= 0:
        cap.release()
        raise RuntimeError("Empty video")
    cap.set(cv2.CAP_PROP_POS_FRAMES, total_frames // 2)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        raise RuntimeError("Failed to read frame")
    img_path = video_path.replace(".mp4", "_key.jpg")
    cv2.imwrite(img_path, frame)
    return img_path

# ======================
# 数据查询
# ======================

def query_asm_object(obj_name: str) -> Optional[Dict[str, Any]]:
    """查询ASM中的对象信息"""
    if not os.path.exists(ASM_JSON_PATH):
        return None
    try:
        with open(ASM_JSON_PATH, 'r') as f:
            data = json.load(f)
        obj = data.get("objects", {}).get(obj_name)
        if obj and obj.get("is_on_map", False):
            return {
                "location": obj["location"],
                "last_time": obj["last_seen_time"]
            }
    except Exception as e:
        print(f"[ERROR] ASM read failed: {e}")
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
            return {
                "id": row[0],
                "name": row[1],
                "last_time": row[2],
                "location": json.loads(row[3])
            }
    except Exception as e:
        print(f"[ERROR] DB query failed: {e}")
    return None

def get_video_path_by_time(timestamp: str) -> str:
    """根据时间获取视频路径"""
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
            print(f"[ERROR] Text API call failed (attempt {attempt + 1}/{max_retries}): {e}")
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

# ======================
# Tools
# ======================

def find_object(obj_name: str) -> ToolResponse:
    """
    查找物品的位置信息
    
    Args:
        obj_name (str): 物品名称
    
    Returns:
        ToolResponse: 工具响应对象
    """
    try:
        # Step 1: ASM
        asm_res = query_asm_object(obj_name)
        if asm_res:
            loc = asm_res["location"]
            if navigate_to(loc["x"], loc["y"]):
                return ToolResponse(
                    content=[{"type": "text", "text": f"{obj_name} 已导航至地图位置"}],
                    metadata={"status": "success", "result": f"{obj_name} 已导航至地图位置"}
                )
            else:
                return ToolResponse(
                    content=[{"type": "text", "text": "导航失败"}],
                    metadata={"status": "error", "result": "导航失败"}
                )

        # Step 2: DB
        db_res = query_history_db(obj_name)
        if not db_res:
            return ToolResponse(
                content=[{"type": "text", "text": f"未找到物品 {obj_name}"}],
                metadata={"status": "error", "result": f"未找到物品 {obj_name}"}
            )

        # Step 3: 视频剪切 + 抽帧
        last_time = db_res["last_time"]
        raw_video = get_video_path_by_time(last_time)
        cut_video_path = cut_video(raw_video, last_time, VIDEO_BASE_DIR)
        if not cut_video_path:
            return ToolResponse(
                content=[{"type": "text", "text": "视频处理失败"}],
                metadata={"status": "error", "result": "视频处理失败"}
            )

        key_frame = extract_key_frame(cut_video_path)

        # Step 4: VL 推理
        prompt = f"根据视频关键帧分析，用一句话描述 '{obj_name}' 的具体位置。只回答位置，不要其他内容。"
        try:
            desc = call_qwen_vl_api(prompt)
            return ToolResponse(
                content=[{"type": "text", "text": desc}],
                metadata={"status": "success", "result": desc}
            )
        except Exception as e:
            print(f"[ERROR] VL inference failed: {e}")
            return ToolResponse(
                content=[{"type": "text", "text": "视觉推理失败"}],
                metadata={"status": "error", "result": "视觉推理失败"}
            )
    except Exception as e:
        return ToolResponse(
            content=[{"type": "text", "text": f"查找物品失败: {str(e)}"}],
            metadata={"status": "error", "result": f"查找物品失败: {str(e)}"}
        )

def go_to_object(desc: str) -> ToolResponse:
    """
    根据描述导航到物品位置
    
    Args:
        desc (str): 物品位置描述
    
    Returns:
        ToolResponse: 工具响应对象
    """
    try:
        # 简化：假设描述中包含坐标（实际应调用端侧模型）
        print(f"[NAV_MODEL] Processing: {desc}")
        result = navigate_to(2.5, 1.8)
        if result:
            return ToolResponse(
                content=[{"type": "text", "text": f"已导航到位置: {desc}"}],
                metadata={"status": "success", "result": f"已导航到位置: {desc}"}
            )
        else:
            return ToolResponse(
                content=[{"type": "text", "text": "导航失败"}],
                metadata={"status": "error", "result": "导航失败"}
            )
    except Exception as e:
        return ToolResponse(
            content=[{"type": "text", "text": f"导航失败: {str(e)}"}],
            metadata={"status": "error", "result": f"导航失败: {str(e)}"}
        )

def follow_person(name: str) -> ToolResponse:
    """
    跟随指定人员
    
    Args:
        name (str): 人员名称
    
    Returns:
        ToolResponse: 工具响应对象
    """
    try:
        result = start_following(f"id_{name}")
        if result:
            return ToolResponse(
                content=[{"type": "text", "text": f"开始跟随 {name}"}],
                metadata={"status": "success", "result": f"开始跟随 {name}"}
            )
        else:
            return ToolResponse(
                content=[{"type": "text", "text": "跟随失败"}],
                metadata={"status": "error", "result": "跟随失败"}
            )
    except Exception as e:
        return ToolResponse(
            content=[{"type": "text", "text": f"跟随失败: {str(e)}"}],
            metadata={"status": "error", "result": f"跟随失败: {str(e)}"}
        )

def stop_follow() -> ToolResponse:
    """
    停止跟随
    
    Returns:
        ToolResponse: 工具响应对象
    """
    try:
        result = stop_following()
        if result:
            notify_navigation_model_stop_following()
            return ToolResponse(
                content=[{"type": "text", "text": "已停止跟随"}],
                metadata={"status": "success", "result": "已停止跟随"}
            )
        else:
            return ToolResponse(
                content=[{"type": "text", "text": "停止跟随失败"}],
                metadata={"status": "error", "result": "停止跟随失败"}
            )
    except Exception as e:
        return ToolResponse(
            content=[{"type": "text", "text": f"停止跟随失败: {str(e)}"}],
            metadata={"status": "error", "result": f"停止跟随失败: {str(e)}"}
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
            return ToolResponse(
                content=[{"type": "text", "text": "已停止导航"}],
                metadata={"status": "success", "result": "已停止导航"}
            )
        else:
            return ToolResponse(
                content=[{"type": "text", "text": "停止导航失败"}],
                metadata={"status": "error", "result": "停止导航失败"}
            )
    except Exception as e:
        return ToolResponse(
            content=[{"type": "text", "text": f"停止导航失败: {str(e)}"}],
            metadata={"status": "error", "result": f"停止导航失败: {str(e)}"}
        )

# ======================
# 初始化测试数据
# ======================

def init_test_data():
    """初始化测试数据"""
    # ASM
    asm_data = {
        "objects": {
            "水杯": {
                "last_seen_time": "2025-11-02T10:00:00",
                "location": {"x": 1.2, "y": 0.8},
                "is_on_map": True
            },
            "手机": {
                "last_seen_time": "2025-11-02T09:45:00",
                "location": {"x": 3.0, "y": 2.5},
                "is_on_map": True
            }
        }
    }
    with open(ASM_JSON_PATH, 'w') as f:
        json.dump(asm_data, f, indent=2)

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
    
    # 插入更多测试数据
    test_objects = [
        ("obj_001", "遥控器", "2025-11-02T09:30:00", json.dumps({"x": 2.1, "y": 1.5})),
        ("obj_002", "钥匙", "2025-11-02T08:15:00", json.dumps({"x": 0.5, "y": 3.2})),
        ("obj_003", "书本", "2025-11-02T07:45:00", json.dumps({"x": 4.0, "y": 1.0})),
        ("obj_004", "钱包", "2025-11-02T06:30:00", json.dumps({"x": 1.8, "y": 4.5}))
    ]
    
    for obj_id, name, last_time, location in test_objects:
        cursor.execute("""
            INSERT OR REPLACE INTO objects (id, name, last_time, location)
            VALUES (?, ?, ?, ?)
        """, (obj_id, name, last_time, location))
    
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
        sys_prompt="""你是一个智能机器人助手，能够执行以下操作：
1. 查找物品 (find_object): 根据物品名称查找其位置
2. 导航到物品 (go_to_object): 根据位置描述导航到物品
3. 跟随人员 (follow_person): 跟随指定人员
4. 停止跟随 (stop_follow): 停止跟随行为
5. 停止导航 (stop_navigate): 停止导航行为

请根据用户指令选择合适的工具来完成任务。你可以组合使用多个工具来完成复杂的任务。
例如，当用户说"找到遥控器然后拿给我"时，你可以先使用find_object工具找到遥控器，然后使用go_to_object工具导航到遥控器位置。""",
        model=DashScopeChatModel(
            api_key=api_key,
            model_name="qwen3-max",
            enable_thinking=True,
            stream=False,
        ),
        formatter=DashScopeChatFormatter(),
        toolkit=toolkit,
        memory=InMemoryMemory(),
    )
    
    return agent

# ======================
# 主程序
# ======================

async def main():
    """主程序入口"""
    # 初始化测试数据
    init_test_data()
    
    # 创建智能机器人agent
    agent = create_smart_robot_agent()
    
    # 测试用例
    test_cases = [
        "你好",
        "去找遥控器",
        "去找手机",
        "去找钥匙",
        "跟着小明",
        "跟着小红",
        "停止跟随",
        "停止导航",
        "去找书本",
        "去找钱包",
        "请帮我找到水杯然后跟着小李"
    ]
    
    for case in test_cases:
        print(f"\n>>> 用户: {case}")
        try:
            # 创建消息对象
            msg = Msg("user", case, "user")
            # 让agent处理消息
            result = await agent(msg)
            
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
            print(f"<<< 系统: {response}")
        except Exception as e:
            print(f"<<< 系统: 任务失败：{e}")

if __name__ == "__main__":
    asyncio.run(main())

























