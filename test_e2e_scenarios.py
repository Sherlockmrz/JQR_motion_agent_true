#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全流程自动化测试脚本

启动 SmartRobotAgent（无串口模式），内嵌 mock 电机结果注入，
通过 WebSocket 发送 test_websocket_control.py 中的 6 个场景命令，
验证全流程通信和任务执行。

用法:
    USB_SERIAL_ENABLED=false python3 test_e2e_scenarios.py
"""
import asyncio
import json
import os
import time
import threading
from datetime import datetime

os.environ['USB_SERIAL_ENABLED'] = 'false'

# 导入 Agent
from smart_robot_agent import SmartRobotAgent, robot_state, MotorResultCode

import websockets

# 6 个测试场景（与 test_websocket_control.py 一致）
SCENARIOS = [
    {"id": 1, "name": "用户移动位置时的视线跟踪", "command": {"type": "user_position_tracking", "params": {"yaw_angle": 255, "pitch_angle": 255}}},
    {"id": 2, "name": "巡逻中停至桌子识别记忆物品", "command": {"type": "patrol_table_inspection", "params": {}}},
    {"id": 3, "name": "声源在头部转角范围内", "command": {"type": "wake_head_range", "params": {"yaw_angle": 255, "pitch_angle": 255}}},
    {"id": 4, "name": "声源超出头部转角极限", "command": {"type": "wake_beyond_head_range", "params": {"yaw_angle": 255, "pitch_angle": 255}}},
    {"id": 5, "name": "行走中侧方被唤醒", "command": {"type": "wake_side_moving", "params": {"yaw_angle": 255}}},
    {"id": 6, "name": "行走中后方被唤醒并停止", "command": {"type": "wake_back_moving", "params": {"yaw_angle": 255}}},
]


def install_motor_mock(ros2_interface):
    """Monkey-patch ROS2Interface 的电机控制方法，模拟发布成功并异步注入结果"""

    original_publish = ros2_interface.publish_combine_motor_control

    def mock_publish(task_id, control_pitch=False, pitch_angle=0.0,
                     control_yaw=False, yaw_angle=0.0,
                     control_chassis_move=False, chassis_offset=0.0,
                     control_chassis_rotate=False, chassis_rotation=0.0,
                     speed_level=0):
        tid = int(task_id)
        print(f"  [MOCK] 电机指令: task_id={tid}, pitch={control_pitch}/{pitch_angle:.2f}, "
              f"yaw={control_yaw}/{yaw_angle:.2f}, move={control_chassis_move}/{chassis_offset:.2f}, "
              f"rotate={control_chassis_rotate}/{chassis_rotation:.2f}, speed={speed_level}")

        # 模拟进度上报 + 最终成功（在后台线程中延迟注入）
        def inject_result():
            delay_map = {0: 0.3, 1: 0.2, 2: 0.1}
            step_delay = delay_map.get(speed_level, 0.3)
            for progress in [25.0, 50.0, 75.0]:
                time.sleep(step_delay)
                ros2_interface.combine_motor_result[tid] = {"result": progress}
            time.sleep(step_delay)
            ros2_interface.combine_motor_result[tid] = {"result": MotorResultCode.SUCCESS}
            print(f"  [MOCK] 电机任务 {tid} 完成: SUCCESS (101)")

        threading.Thread(target=inject_result, daemon=True).start()
        return {"success": True, "task_id": tid}

    ros2_interface.publish_combine_motor_control = mock_publish
    print("[MOCK] 电机控制已替换为模拟模式")


async def main():
    print("=" * 70)
    print("  全流程端到端测试 (Agent + Mock电机 + WebSocket)")
    print("=" * 70)

    # 1. 启动 Agent
    print("\n[1/4] 启动 SmartRobotAgent (无串口模式)...")
    agent = SmartRobotAgent()
    agent.event_loop = asyncio.get_running_loop()
    robot_state.agent_instance = agent
    await agent.initialize()
    await asyncio.sleep(1)

    # 2. 安装 mock 电机
    print("[2/4] 安装 mock 电机结果注入...")
    install_motor_mock(agent.ros2_interface)

    # 3. 运行 6 个场景测试
    print("[3/4] 通过 WebSocket 运行 6 个场景测试...\n")
    results = []
    passed = 0
    failed = 0

    async with websockets.connect("ws://localhost:8766") as ws:
        print("WebSocket 已连接\n")

        for s in SCENARIOS:
            print(f"{'━' * 60}")
            print(f"  场景{s['id']}: {s['name']}")
            print(f"{'─' * 60}")

            start = time.time()
            await ws.send(json.dumps(s["command"]))

            try:
                resp_str = await asyncio.wait_for(ws.recv(), timeout=60)
                elapsed = time.time() - start
                resp = json.loads(resp_str)
                success = resp.get("success", False)

                if success:
                    passed += 1
                    status = "PASS"
                    print(f"  ✓ 测试通过 ({elapsed:.2f}s)")
                else:
                    failed += 1
                    status = "FAIL"
                    err = resp.get("error_msg", "未知错误")
                    print(f"  ✗ 测试失败 ({elapsed:.2f}s): {err}")

                results.append({
                    "id": s["id"], "name": s["name"], "type": s["command"]["type"],
                    "status": status, "success": success, "elapsed": f"{elapsed:.2f}s",
                    "response": resp
                })
            except asyncio.TimeoutError:
                failed += 1
                elapsed = time.time() - start
                print(f"  ✗ 超时 ({elapsed:.2f}s)")
                results.append({
                    "id": s["id"], "name": s["name"], "type": s["command"]["type"],
                    "status": "TIMEOUT", "success": False, "elapsed": f"{elapsed:.2f}s"
                })

            # 场景间间隔
            await asyncio.sleep(0.5)

    # 4. 汇总
    print(f"\n{'━' * 60}")
    print(f"  测试汇总: 通过 {passed} / 失败 {failed} / 共 {len(SCENARIOS)}")
    print(f"{'━' * 60}")

    # 保存报告
    report = {
        "test_date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "test_type": "全流程端到端测试 (Agent + Mock电机 + WebSocket)",
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
    success = asyncio.run(main())
    exit(0 if success else 1)
