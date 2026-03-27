#!/usr/bin/env python3
"""
自动化全场景测试脚本（含障碍物绕行场景）
无需交互输入，直接运行所有场景并输出结果
"""
import asyncio
import json
import math
import time
from datetime import datetime
import websockets

WS_URI = "ws://localhost:8766"
TIMEOUT = 60  # 秒


def log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {msg}")


SCENARIOS = [
    {
        "id": 1,
        "name": "user_position_tracking",
        "command": {"type": "user_position_tracking", "params": {"yaw_angle": 255, "pitch_angle": 255}}
    },
    {
        "id": 2,
        "name": "patrol_table_inspection",
        "command": {"type": "patrol_table_inspection", "params": {}}
    },
    {
        "id": 3,
        "name": "wake_head_range",
        "command": {"type": "wake_head_range", "params": {"yaw_angle": 255, "pitch_angle": 255}}
    },
    {
        "id": 4,
        "name": "wake_beyond_head_range",
        "command": {"type": "wake_beyond_head_range", "params": {"yaw_angle": 255, "pitch_angle": 255}}
    },
    {
        "id": 5,
        "name": "wake_side_moving",
        "command": {"type": "wake_side_moving", "params": {"yaw_angle": 255}}
    },
    {
        "id": 6,
        "name": "wake_back_moving",
        "command": {"type": "wake_back_moving", "params": {"yaw_angle": 255}}
    },
    {
        "id": 7,
        "name": "obstacle_avoidance_turn",
        "command": {"type": "obstacle_avoidance_turn", "params": {"turn_angle": 0.785, "head_speed": 30}}
    },
]


async def run_all():
    print("=" * 70)
    print("全场景自动化测试 (含障碍物绕行)")
    print(f"服务器: {WS_URI}")
    print("=" * 70)

    passed, failed = 0, 0
    results = []

    try:
        async with websockets.connect(WS_URI) as ws:
            log("已连接到WebSocket服务器")

            for sc in SCENARIOS:
                log(f"\n▶ 场景{sc['id']}: {sc['name']}")
                start = time.time()
                try:
                    await ws.send(json.dumps(sc["command"]))
                    response = await asyncio.wait_for(ws.recv(), timeout=TIMEOUT)
                    elapsed = time.time() - start
                    resp = json.loads(response)
                    ok = resp.get("success", False)
                    if ok:
                        log(f"  ✅ 通过 ({elapsed:.1f}s)")
                        passed += 1
                    else:
                        log(f"  ❌ 失败: {resp.get('error_msg', '未知错误')}")
                        failed += 1
                    results.append((sc["name"], ok, elapsed))
                except asyncio.TimeoutError:
                    log(f"  ❌ 超时 (>{TIMEOUT}s)")
                    failed += 1
                    results.append((sc["name"], False, TIMEOUT))
                except Exception as e:
                    log(f"  ❌ 异常: {e}")
                    failed += 1
                    results.append((sc["name"], False, 0))

                if sc["id"] < len(SCENARIOS):
                    await asyncio.sleep(1.0)

    except ConnectionRefusedError:
        print(f"\n❌ 无法连接到 {WS_URI}")
        print("请先启动:")
        print("  1. python3 mock_motor_node.py --mode progress")
        print("  2. python3 smart_robot_agent.py --test-mode")
        return

    # 汇总
    print("\n" + "=" * 70)
    print("测试结果汇总")
    print("=" * 70)
    for name, ok, t in results:
        status = "✅" if ok else "❌"
        print(f"  {status} {name:<35} ({t:.1f}s)")
    print(f"\n总计: {passed}/{len(SCENARIOS)} 通过 | 通过率: {passed/len(SCENARIOS)*100:.0f}%")

    # 高亮新场景
    obstacle_result = next((r for r in results if r[0] == "obstacle_avoidance_turn"), None)
    if obstacle_result:
        ok = obstacle_result[1]
        status = "✅ 通过" if ok else "❌ 失败"
        print(f"\n新增场景 obstacle_avoidance_turn: {status}")


if __name__ == "__main__":
    asyncio.run(run_all())
