#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全流程端到端测试脚本

前置条件（需要先 source ROS2 环境）:
    source install/setup.bash

用法:
    # 终端1: 启动 mock 电机节点
    python3 mock_motor_node.py --mode progress --delay 0.5

    # 终端2: 启动 Agent 并运行测试
    USB_SERIAL_ENABLED=false python3 test_e2e_scenarios.py
"""
import asyncio
import json
import os
import time
from datetime import datetime

os.environ.setdefault('USB_SERIAL_ENABLED', 'false')

from smart_robot_agent import SmartRobotAgent, robot_state
import websockets

SCENARIOS = [
    {"id": 1, "name": "用户移动位置时的视线跟踪",
     "command": {"type": "user_position_tracking", "params": {"yaw_angle": 255, "pitch_angle": 255}}},
    {"id": 2, "name": "巡逻中停至桌子识别记忆物品",
     "command": {"type": "patrol_table_inspection", "params": {}}},
    {"id": 3, "name": "声源在头部转角范围内",
     "command": {"type": "wake_head_range", "params": {"yaw_angle": 255, "pitch_angle": 255}}},
    {"id": 4, "name": "声源超出头部转角极限",
     "command": {"type": "wake_beyond_head_range", "params": {"yaw_angle": 255, "pitch_angle": 255}}},
    {"id": 5, "name": "行走中侧方被唤醒",
     "command": {"type": "wake_side_moving", "params": {"yaw_angle": 255}}},
    {"id": 6, "name": "行走中后方被唤醒并停止",
     "command": {"type": "wake_back_moving", "params": {"yaw_angle": 255}}},
]


async def main():
    print("=" * 70)
    print("  全流程端到端测试 (Agent + mock_motor_node + WebSocket)")
    print("=" * 70)
    print("\n请确保已在另一个终端运行:")
    print("  python3 mock_motor_node.py --mode progress --delay 0.5\n")

    # 1. 启动 Agent
    print("[1/3] 启动 SmartRobotAgent...")
    agent = SmartRobotAgent()
    agent.event_loop = asyncio.get_running_loop()
    robot_state.agent_instance = agent
    success = await agent.initialize()
    if not success:
        print("Agent 初始化失败")
        return False
    await asyncio.sleep(1.5)

    # 2. 运行 6 个场景测试
    print("[2/3] 通过 WebSocket 运行 6 个场景测试...\n")
    results = []
    passed = 0
    failed = 0

    async with websockets.connect("ws://localhost:8766") as ws:
        print("WebSocket 已连接\n")

        for s in SCENARIOS:
            sid = s["id"]
            name = s["name"]
            print("━" * 60)
            print(f"  场景{sid}: {name}")
            print("─" * 60)

            start = time.time()
            await ws.send(json.dumps(s["command"]))

            try:
                resp_str = await asyncio.wait_for(ws.recv(), timeout=60)
                elapsed = time.time() - start
                resp = json.loads(resp_str)
                ok = resp.get("success", False)

                if ok:
                    passed += 1
                    print(f"  ✓ 测试通过 ({elapsed:.2f}s)")
                else:
                    failed += 1
                    err = resp.get("error_msg", "未知错误")
                    print(f"  ✗ 测试失败 ({elapsed:.2f}s): {err}")

                results.append({
                    "id": sid, "name": name, "type": s["command"]["type"],
                    "status": "PASS" if ok else "FAIL",
                    "success": ok, "elapsed": f"{elapsed:.2f}s",
                    "response": resp
                })
            except asyncio.TimeoutError:
                failed += 1
                elapsed = time.time() - start
                print(f"  ✗ 超时 ({elapsed:.2f}s)")
                results.append({
                    "id": sid, "name": name, "type": s["command"]["type"],
                    "status": "TIMEOUT", "success": False, "elapsed": f"{elapsed:.2f}s"
                })

            await asyncio.sleep(0.5)

    # 3. 汇总
    print(f"\n{'━' * 60}")
    print(f"  测试汇总: 通过 {passed} / 失败 {failed} / 共 {len(SCENARIOS)}")
    print(f"{'━' * 60}")

    report = {
        "test_date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "test_type": "全流程E2E (Agent + mock_motor_node + WebSocket)",
        "passed": passed,
        "failed": failed,
        "total": len(SCENARIOS),
        "all_passed": passed == len(SCENARIOS),
        "results": results,
    }
    with open("test_e2e_report.json", "w") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(f"\n测试报告已保存: test_e2e_report.json")

    agent.cleanup()
    return passed == len(SCENARIOS)


if __name__ == "__main__":
    ok = asyncio.run(main())
    exit(0 if ok else 1)
