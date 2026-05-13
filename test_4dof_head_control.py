#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""4自由度头颈控制测试脚本

适配agent中新的4自由度任务控制接口，将test.py中的演示动作转换为
roll/pitch/yaw三轴控制（移除底盘控制部分）。

坐标系：ROS2标准坐标系（头颈部正前方向）
- yaw: 偏航角（左右转头），正值=左转，负值=右转
- pitch: 俯仰角（上下点头），正值=低头，负值=抬头
- roll: 翻滚角（左右歪头），正值=向左歪，负值=向右歪
"""
import asyncio
import json
import math
import time
from datetime import datetime
import websockets


def log(msg):
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] {msg}")


async def send_and_recv(websocket, command, timeout=60):
    """发送命令并等待响应"""
    msg = json.dumps(command, ensure_ascii=False)
    log(f"  发送: {msg}")
    start = time.time()
    await websocket.send(msg)
    try:
        response = await asyncio.wait_for(websocket.recv(), timeout=timeout)
        elapsed = time.time() - start
        resp_data = json.loads(response)
        log(f"  响应 (耗时{elapsed:.2f}s): {json.dumps(resp_data, ensure_ascii=False, indent=2)}")
        return resp_data
    except asyncio.TimeoutError:
        log("  ✗ 等待响应超时")
        return None


def input_float(prompt, default=0.0):
    """安全读取浮点数输入"""
    try:
        val = input(prompt).strip()
        if val == '':
            return default
        return float(val)
    except ValueError:
        print("  输入无效，使用默认值:", default)
        return default


def input_int(prompt, default=0):
    """安全读取整数输入"""
    try:
        val = input(prompt).strip()
        if val == '':
            return default
        return int(val)
    except ValueError:
        print("  输入无效，使用默认值:", default)
        return default


# ========================
# 4自由度头颈控制场景定义
# ========================
SCENARIOS = [
    # ---------- 交互中 / 用户对话 ----------
    {
        "id": 1,
        "category": "交互中 / 用户对话",
        "name": "用户移动位置时的视线跟踪",
        "description": (
            "用户从机器人正前方起身，走到侧面继续提问。\n"
            "  头部: 俯仰0°→30°(低头), 偏航0°→30°(左转), 翻滚0°(保持)\n"
            "  ROS2坐标系: pitch=30°(低头), yaw=30°(左转), roll=0°"
        ),
        "command": {
            "type": "set_four_combine_motor_control",
            "params": {
                "control_yaw": True,
                "yaw_angle": math.radians(30),
                "control_pitch": True,
                "pitch_angle": math.radians(30),
                "control_roll": False,
                "roll_angle": 0.0,
                "speed_level": 1
            }
        }
    },
    # ---------- 头部归零 ----------
    {
        "id": 2,
        "category": "头部电机归零位",
        "name": "头部电机回归0位",
        "description": (
            "将头部的yaw、pitch、roll都回归到0度位置。\n"
            "  ROS2坐标系: yaw=0°, pitch=0°, roll=0°"
        ),
        "command": {
            "type": "set_four_combine_motor_control",
            "params": {
                "control_yaw": True,
                "yaw_angle": 0.0,
                "control_pitch": True,
                "pitch_angle": 0.0,
                "control_roll": True,
                "roll_angle": 0.0,
                "speed_level": 1
            }
        }
    },
    # ---------- 左右摆头 ----------
    {
        "id": 3,
        "category": "行走/巡逻",
        "name": "左右摆头观察",
        "description": (
            "机器人左右张望，适用于巡逻观察等场景。\n"
            "  左转80° → 右转80° → 回中0°"
        ),
        "command": {
            "type": "head_sweep_sequence",
            "params": {
                "sequence": [
                    {"yaw": 80, "pitch": 0, "roll": 0, "speed": 1},
                    {"yaw": -80, "pitch": 0, "roll": 0, "speed": 1},
                    {"yaw": 0, "pitch": 0, "roll": 0, "speed": 1}
                ]
            }
        }
    },
    # ---------- 手动输入控制 ----------
    {
        "id": 10,
        "category": "手动输入控制",
        "name": "4自由度头颈手动控制（输入roll/pitch/yaw角度）",
        "description": (
            "手动输入头颈三轴角度（单位：度），转换为弧度后下发控制。\n"
            "  ROS2坐标系（头颈部正前方向）:\n"
            "  - yaw: 偏航角，正值=左转，负值=右转\n"
            "  - pitch: 俯仰角，正值=低头，负值=抬头\n"
            "  - roll: 翻滚角，正值=向左歪，负值=向右歪\n"
            "  输入0跳过该轴控制"
        ),
        "command": None,
        "interactive": "head_4dof"
    },
]


def build_head_4dof_command():
    """交互式构建4自由度头颈控制命令"""
    print("  ── 4自由度头颈参数输入 ──")
    yaw_deg = input_float("    yaw偏航角(度, 正=左转, 负=右转, 0=不控制): ", 0.0)
    pitch_deg = input_float("    pitch俯仰角(度, 正=低头, 负=抬头, 0=不控制): ", 0.0)
    roll_deg = input_float("    roll翻滚角(度, 正=左歪, 负=右歪, 0=不控制): ", 0.0)
    speed = input_int("    速度档位(0=低速, 1=中速, 2=快速) [默认1]: ", 1)

    control_yaw = True
    control_pitch = True
    control_roll = True

    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(pitch_deg)
    roll_rad = math.radians(roll_deg)

    print(f"  → yaw={yaw_deg}°({yaw_rad:.4f}rad), pitch={pitch_deg}°({pitch_rad:.4f}rad), roll={roll_deg}°({roll_rad:.4f}rad), speed={speed}")

    return {
        "type": "set_four_combine_motor_control",
        "params": {
            "control_yaw": control_yaw,
            "yaw_angle": yaw_rad,
            "control_pitch": control_pitch,
            "pitch_angle": pitch_rad,
            "control_roll": control_roll,
            "roll_angle": roll_rad,
            "speed_level": speed
        }
    }


def print_scenario_menu():
    """打印场景选择菜单"""
    print("\n" + "=" * 80)
    print("  4自由度头颈控制 · 场景测试")
    print("=" * 80)

    current_category = None
    for s in SCENARIOS:
        if s["category"] != current_category:
            current_category = s["category"]
            print(f"\n  【{current_category}】")
        print(f"    {s['id']}. {s['name']}")

    print(f"\n    0. 全部运行（按顺序逐个测试）")
    print("    q. 退出")
    print("=" * 80)


async def run_scenario(websocket, scenario):
    """运行单个场景测试"""
    print(f"\n{'━' * 80}")
    print(f"  场景{scenario['id']}: {scenario['name']}")
    print(f"  分类: {scenario['category']}")
    print(f"{'─' * 80}")
    print(f"  {scenario['description']}")
    print(f"{'─' * 80}")

    # 交互式场景：需要用户输入参数
    interactive = scenario.get("interactive")
    if interactive:
        if interactive == "head_4dof":
            command = build_head_4dof_command()
        else:
            log(f"  ✗ 未知交互类型: {interactive}")
            return False

        if command is None:
            log("  跳过（无操作参数）")
            return True

        resp = await send_and_recv(websocket, command)
    else:
        resp = await send_and_recv(websocket, scenario["command"])

    if resp is not None:
        success = resp.get("success", False)
        if success:
            log(f"  ✓ 场景{scenario['id']}测试通过")
        else:
            log(f"  ✗ 场景{scenario['id']}测试失败: {resp.get('error_msg', '未知错误')}")
        return success
    else:
        log(f"  ✗ 场景{scenario['id']}测试失败: 无响应")
        return False


async def main():
    ws_uri = "ws://localhost:8766"

    while True:
        print_scenario_menu()
        choice = input("\n请选择场景编号: ").strip()

        if choice.lower() == 'q':
            log("退出测试")
            break

        # 解析选择
        if choice == '0':
            selected = SCENARIOS
        else:
            try:
                idx = int(choice)
                selected = [s for s in SCENARIOS if s["id"] == idx]
                if not selected:
                    print(f"  ✗ 无效编号: {idx}")
                    continue
            except ValueError:
                print(f"  ✗ 请输入数字或 'q'")
                continue

        # 连接并执行
        log(f"连接到 {ws_uri} ...")
        try:
            async with websockets.connect(ws_uri) as websocket:
                log("✓ 已连接到WebSocket服务器")

                passed = 0
                failed = 0

                for scenario in selected:
                    success = await run_scenario(websocket, scenario)
                    if success:
                        passed += 1
                    else:
                        failed += 1

                    # 场景间间隔
                    if len(selected) > 1:
                        await asyncio.sleep(1.5)

                # 汇总（多场景时显示）
                if len(selected) > 1:
                    print(f"\n{'━' * 80}")
                    log(f"测试汇总: 共 {len(selected)} 个场景, 通过 {passed}, 失败 {failed}")
                    print("━" * 80)

        except ConnectionRefusedError:
            log("✗ 无法连接到WebSocket服务器，请确保Agent已启动")
        except Exception as e:
            log(f"✗ 连接失败: {e}")

        # 单场景测试完后回到菜单继续选择
        if choice != '0':
            continue
        else:
            break


if __name__ == "__main__":
    print("4自由度头颈控制 · 场景测试工具")
    print("请确保以下服务已启动:")
    print("  1. SmartRobotAgent (WebSocket端口 8766)")
    print()
    asyncio.run(main())
