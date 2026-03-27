#!/usr/bin/env python3
"""
模拟WebSocket服务器（不依赖ROS2）
用于验证所有场景的接口格式和通信链路，包含障碍物绕行场景
"""
import asyncio
import json
import math
import websockets
from datetime import datetime


def log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {msg}")


# 各场景的模拟执行时间（秒）
SCENARIO_DELAY = {
    "user_position_tracking": 2.0,
    "patrol_table_inspection": 8.0,   # 4步骤，每步约2秒
    "wake_head_range": 1.0,
    "wake_beyond_head_range": 5.0,    # 3步骤
    "wake_side_moving": 5.0,          # 3步骤
    "wake_back_moving": 6.0,          # 3步骤，最后一步旋转180°
    "obstacle_avoidance_turn": 4.0,   # 步骤1慢速3s + 步骤2快速1s
}


async def handle_motor_command(task_type, params):
    """模拟电机指令执行"""
    delay = SCENARIO_DELAY.get(task_type, 2.0)

    if task_type == "obstacle_avoidance_turn":
        turn_angle = params.get("turn_angle", math.radians(45))
        head_speed = params.get("head_speed", 30)
        turn_deg = math.degrees(turn_angle)

        log(f"  [模拟] 步骤1: 头部预转右侧{turn_deg:.1f}° (低速, {head_speed}°/s)...")
        await asyncio.sleep(3.0)  # 低速档约3秒

        log(f"  [模拟] 步骤2: 底盘右转{turn_deg:.1f}° + 头部回正 (快速)...")
        await asyncio.sleep(1.0)  # 快速档约1秒

    elif task_type == "patrol_table_inspection":
        log("  [模拟] 步骤1: 头部俯视-15°...")
        await asyncio.sleep(2.0)
        log("  [模拟] 步骤2: 头部左扫-45°...")
        await asyncio.sleep(2.0)
        log("  [模拟] 步骤3: 头部右扫45°...")
        await asyncio.sleep(2.0)
        log("  [模拟] 步骤4: 头部回正...")
        await asyncio.sleep(1.0)

    elif task_type in ("wake_beyond_head_range", "wake_side_moving", "wake_back_moving"):
        log(f"  [模拟] 步骤1: 头部转向...")
        await asyncio.sleep(1.0)
        log(f"  [模拟] 步骤2: 底盘旋转...")
        await asyncio.sleep(2.0)
        log(f"  [模拟] 步骤3: 头部回正...")
        await asyncio.sleep(1.0)

    else:
        await asyncio.sleep(delay)

    return {"success": True, "type": task_type}


async def handler(websocket):
    addr = websocket.remote_address
    log(f"客户端连接: {addr}")

    try:
        async for message in websocket:
            data = json.loads(message)
            task_type = data.get("type")
            params = data.get("params", {})

            log(f"\n收到指令: {task_type}")
            log(f"  参数: {json.dumps(params, ensure_ascii=False)}")

            if task_type not in SCENARIO_DELAY:
                response = {"success": False, "error_msg": f"未知场景类型: {task_type}"}
            else:
                response = await handle_motor_command(task_type, params)

            log(f"  返回: {json.dumps(response, ensure_ascii=False)}")
            await websocket.send(json.dumps(response))

    except websockets.exceptions.ConnectionClosed:
        log(f"客户端断开: {addr}")


async def main():
    port = 8767  # 与Agent的8766区分
    print("=" * 60)
    print(f"模拟WebSocket服务器（测试用，不依赖ROS2）")
    print(f"监听端口: {port}")
    print(f"支持场景: {', '.join(SCENARIO_DELAY.keys())}")
    print("=" * 60)

    async with websockets.serve(handler, "0.0.0.0", port):
        log(f"服务器已启动 ws://localhost:{port}")
        await asyncio.Future()  # 永久运行


if __name__ == "__main__":
    asyncio.run(main())
