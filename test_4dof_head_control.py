#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""4自由度头颈控制场景测试脚本

覆盖根目录 1.png/2.png/3.png 中与头颈相关的测试场景。
本次测试不涉及底盘：所有命令都只下发 yaw/roll/pitch，明确不发送底盘位移/旋转控制。

坐标系：ROS2标准坐标系（头颈部正前方向）
- yaw: 偏航角（左右转头），正值=左转，负值=右转
- pitch: 俯仰角（上下点头），正值=低头，负值=抬头
- roll: 翻滚角（左右歪头），正值=向左歪，负值=向右歪
"""
import argparse
import asyncio
import json
import math
import time
from datetime import datetime
from typing import Any, Dict, List, Optional

import websockets


DEFAULT_WS_URI = "ws://localhost:8766"


def log(msg: str) -> None:
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] {msg}")


def deg(value: float) -> float:
    return math.radians(value)


def speed_level_for(deg_per_sec: float) -> int:
    """将图片中的角速度诉求映射到现有接口速度档位。"""
    if deg_per_sec <= 30:
        return 0
    if deg_per_sec <= 45:
        return 1
    return 2


def head_command(
    *,
    yaw: Optional[float] = None,
    pitch: Optional[float] = None,
    roll: Optional[float] = None,
    speed_deg_s: float = 45,
    timeout: float = 30.0,
) -> Dict[str, Any]:
    """构建单步四联头颈控制命令，角度单位为度。"""
    return {
        "type": "set_four_combine_motor_control",
        "params": {
            "control_yaw": yaw is not None,
            "yaw_angle": deg(yaw or 0.0),
            "control_roll": roll is not None,
            "roll_angle": deg(roll or 0.0),
            "control_pitch": pitch is not None,
            "pitch_angle": deg(pitch or 0.0),
            "control_chassis_move": False,
            "chassis_offset": 0.0,
            "control_chassis_rotate": False,
            "chassis_rotation": 0.0,
            "speed_level": speed_level_for(speed_deg_s),
            "timeout": timeout,
        },
    }


def head_sequence(steps: List[Dict[str, Any]], timeout: float = 45.0) -> Dict[str, Any]:
    """构建头颈多步动作序列；步骤角度单位为度，不包含任何底盘字段。"""
    return {
        "type": "four_dof_head_sequence",
        "params": {
            "angle_unit": "deg",
            "timeout": timeout,
            "sequence": steps,
        },
    }


# ========================
# 1.png / 2.png / 3.png 场景定义
# ========================
SCENARIOS = [
    {
        "id": 1,
        "category": "交互中 / 用户对话",
        "name": "用户移动位置时的视线跟踪",
        "description": (
            "用户从机器人正前方起身，走到机器人侧面继续提问。\n"
            "  头部: 俯仰0°→45°(低头), 水平0°→45°(左转), 同时平滑跟踪。\n"
            "  速度: 图片要求俯仰45°/s、水平45°/s，映射为 speed_level=1。\n"
            "  底盘: 保持静止，本脚本不下发底盘控制。"
        ),
        "command": head_command(yaw=45, pitch=45, speed_deg_s=45),
    },
    {
        "id": 2,
        "category": "行走/巡逻 / 在家庭中行走",
        "name": "绕行障碍物时的协同转向（仅头部）",
        "description": (
            "机器人遇到障碍物需要绕行，路径向右转弯。\n"
            "  头部: 提前向绕行方向右侧预转，水平0°→-45°，引导视线。\n"
            "  速度: 图片要求水平30°/s，映射为 speed_level=0。\n"
            "  底盘: 图片中底盘转向动作本次不测，不下发底盘控制。"
        ),
        "command": head_command(yaw=-45, speed_deg_s=30),
    },
    {
        "id": 3,
        "category": "行走/巡逻 / 在家庭中行走",
        "name": "巡逻中停至桌子识别记忆物品",
        "description": (
            "机器人巡逻途中检测到桌子，自主靠近并停稳，识别记忆桌面物品后恢复巡逻。\n"
            "  头部: 停稳后低头看桌面，再左/中/右/中扫描，识别完成后抬头回正。\n"
            "  序列: pitch 0°→15°；yaw 0°→45°→0°→-45°→0°；pitch 15°→0°。\n"
            "  速度: 图片要求俯仰15°/s、水平30°/s，映射为低速/中低速。\n"
            "  底盘: 接近/减速动作本次不测，不下发底盘控制。"
        ),
        "command": head_sequence([
            {"pitch": 15, "speed_deg_s": 15, "timeout": 20.0},
            {"yaw": 45, "pitch": 15, "speed_deg_s": 30, "timeout": 20.0},
            {"yaw": 0, "pitch": 15, "speed_deg_s": 30, "timeout": 20.0},
            {"yaw": -45, "pitch": 15, "speed_deg_s": 30, "timeout": 20.0},
            {"yaw": 0, "pitch": 15, "speed_deg_s": 30, "timeout": 20.0},
            {"yaw": 0, "pitch": 0, "roll": 0, "speed_deg_s": 45, "timeout": 20.0},
        ]),
    },
    {
        "id": 4,
        "category": "唤醒 / 静止状态被唤醒",
        "name": "声源在头部转角范围内",
        "description": (
            "机器人静止充电，用户站立在正前方45°范围内呼唤唤醒词。\n"
            "  头部: 快速转向声源，俯仰0°→45°，水平0°→45°，同时运动并锁定用户。\n"
            "  速度: 图片要求俯仰45°/s、水平45°/s，偏快速响应，映射为 speed_level=1。\n"
            "  底盘: 不介入，保持静止。"
        ),
        "command": head_command(yaw=45, pitch=45, speed_deg_s=45),
    },
    {
        "id": 5,
        "category": "唤醒 / 静止状态被唤醒",
        "name": "声源超出头部转角极限（仅头部）",
        "description": (
            "机器人静止背对用户，用户站在后方呼唤唤醒词。\n"
            "  头部: 优先锁定声源，俯仰0°→45°，水平0°→90°；随后回正。\n"
            "  速度: 图片要求俯仰45°/s、水平90°/s；回正水平45°/s。\n"
            "  底盘: 图片中的原地旋转本次不测，不下发底盘控制。"
        ),
        "command": head_sequence([
            {"yaw": 90, "pitch": 45, "speed_deg_s": 90, "timeout": 25.0},
            {"yaw": 0, "pitch": 0, "roll": 0, "speed_deg_s": 45, "timeout": 25.0},
        ]),
    },
    {
        "id": 6,
        "category": "唤醒 / 运动状态被唤醒",
        "name": "行走中侧方被唤醒（仅头部）",
        "description": (
            "机器人正在向前行走，用户坐在左侧呼唤唤醒词。\n"
            "  头部: 优先锁定声源，水平0°→45°；随后回正。\n"
            "  速度: 图片要求水平45°/s，映射为 speed_level=1。\n"
            "  底盘: 图片中的左转本次不测，不下发底盘控制。"
        ),
        "command": head_sequence([
            {"yaw": 45, "speed_deg_s": 45, "timeout": 20.0},
            {"yaw": 0, "roll": 0, "pitch": 0, "speed_deg_s": 45, "timeout": 20.0},
        ]),
    },
    {
        "id": 7,
        "category": "唤醒 / 运动状态被唤醒",
        "name": "行走中后方被唤醒并停止（仅头部）",
        "description": (
            "机器人巡逻中，用户坐在后方喊停并唤醒。\n"
            "  头部: 优先锁定后方声源，水平0°→90°；随后柔和回正。\n"
            "  速度: 图片要求水平90°/s；回正水平45°/s。\n"
            "  底盘: 图片中的减速/转向/停止本次不测，不下发底盘控制。"
        ),
        "command": head_sequence([
            {"yaw": 90, "speed_deg_s": 90, "timeout": 25.0},
            {"yaw": 0, "roll": 0, "pitch": 0, "speed_deg_s": 45, "timeout": 25.0},
        ]),
    },
    {
        "id": 90,
        "category": "辅助",
        "name": "头部电机回归0位",
        "description": "将 yaw、pitch、roll 都回归到0°，不控制底盘。",
        "command": head_command(yaw=0, pitch=0, roll=0, speed_deg_s=45),
    },
    {
        "id": 99,
        "category": "手动输入控制",
        "name": "4自由度头颈手动控制（输入roll/pitch/yaw角度）",
        "description": (
            "手动输入头颈三轴角度（单位：度），转换为弧度后下发控制。\n"
            "  留空=不控制该轴；输入0=控制该轴回正。\n"
            "  本手动控制同样不下发底盘控制。"
        ),
        "command": None,
        "interactive": "head_4dof",
        "run_in_all": False,
    },
]


def input_optional_float(prompt: str) -> Optional[float]:
    """读取可选浮点数；留空表示不控制该轴。"""
    while True:
        val = input(prompt).strip()
        if val == "":
            return None
        try:
            return float(val)
        except ValueError:
            print("  输入无效，请输入数字或直接回车跳过")


def input_float(prompt: str, default: float = 0.0) -> float:
    try:
        val = input(prompt).strip()
        if val == "":
            return default
        return float(val)
    except ValueError:
        print("  输入无效，使用默认值:", default)
        return default


def build_head_4dof_command() -> Optional[Dict[str, Any]]:
    """交互式构建4自由度头颈控制命令。"""
    print("  ── 4自由度头颈参数输入 ──")
    yaw_deg = input_optional_float("    yaw偏航角(度, 正=左转, 负=右转, 留空=不控制): ")
    pitch_deg = input_optional_float("    pitch俯仰角(度, 正=低头, 负=抬头, 留空=不控制): ")
    roll_deg = input_optional_float("    roll翻滚角(度, 正=左歪, 负=右歪, 留空=不控制): ")
    speed_deg_s = input_float("    期望头部速度(°/s, 30/45/90; 默认45): ", 45.0)

    if yaw_deg is None and pitch_deg is None and roll_deg is None:
        print("  未输入任何轴，跳过")
        return None

    command = head_command(yaw=yaw_deg, pitch=pitch_deg, roll=roll_deg, speed_deg_s=speed_deg_s)
    params = command["params"]
    print(
        "  → "
        f"yaw={yaw_deg if yaw_deg is not None else '不控制'}, "
        f"pitch={pitch_deg if pitch_deg is not None else '不控制'}, "
        f"roll={roll_deg if roll_deg is not None else '不控制'}, "
        f"speed_level={params['speed_level']}"
    )
    return command


def command_has_chassis_control(command: Optional[Dict[str, Any]]) -> bool:
    """防止场景误带底盘控制字段。"""
    if not command:
        return False
    params = command.get("params", {})
    if params.get("control_chassis_move") or params.get("control_chassis_rotate"):
        return True
    for step in params.get("sequence", []):
        if step.get("control_chassis_move") or step.get("control_chassis_rotate"):
            return True
        if "chassis_offset" in step or "chassis_rotation" in step:
            return True
    return False


async def send_and_recv(websocket, command: Dict[str, Any], timeout: float = 90.0) -> Optional[Dict[str, Any]]:
    """发送命令并等待响应。"""
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


def response_success(resp: Dict[str, Any]) -> bool:
    """兼容 WebSocket 的 success 字段和 agent 内部 result 布尔字段。"""
    if "success" in resp:
        return bool(resp.get("success"))
    if isinstance(resp.get("result"), bool):
        return bool(resp.get("result"))
    return False


def print_scenario_menu() -> None:
    print("\n" + "=" * 80)
    print("  4自由度头颈控制 · 场景测试（1.png/2.png/3.png，底盘不参与）")
    print("=" * 80)

    current_category = None
    for s in SCENARIOS:
        if s["category"] != current_category:
            current_category = s["category"]
            print(f"\n  【{current_category}】")
        print(f"    {s['id']}. {s['name']}")

    print("\n    0. 全部运行（跳过手动输入场景）")
    print("    q. 退出")
    print("=" * 80)


async def run_scenario(websocket, scenario: Dict[str, Any]) -> bool:
    print(f"\n{'━' * 80}")
    print(f"  场景{scenario['id']}: {scenario['name']}")
    print(f"  分类: {scenario['category']}")
    print(f"{'─' * 80}")
    print(f"  {scenario['description']}")
    print(f"{'─' * 80}")

    interactive = scenario.get("interactive")
    if interactive == "head_4dof":
        command = build_head_4dof_command()
    else:
        command = scenario.get("command")

    if command is None:
        log("  跳过（无操作参数）")
        return True

    if command_has_chassis_control(command):
        log("  ✗ 场景包含底盘控制字段，已阻止下发")
        return False

    resp = await send_and_recv(websocket, command)
    if resp is None:
        log(f"  ✗ 场景{scenario['id']}测试失败: 无响应")
        return False

    success = response_success(resp)
    if success:
        log(f"  ✓ 场景{scenario['id']}测试通过")
    else:
        log(f"  ✗ 场景{scenario['id']}测试失败: {resp.get('error_msg', '未知错误')}")
    return success


async def main(ws_uri: str) -> None:
    while True:
        print_scenario_menu()
        choice = input("\n请选择场景编号: ").strip()

        if choice.lower() == "q":
            log("退出测试")
            break

        if choice == "0":
            selected = [s for s in SCENARIOS if s.get("run_in_all", True) and not s.get("interactive")]
        else:
            try:
                idx = int(choice)
                selected = [s for s in SCENARIOS if s["id"] == idx]
                if not selected:
                    print(f"  ✗ 无效编号: {idx}")
                    continue
            except ValueError:
                print("  ✗ 请输入数字或 'q'")
                continue

        log(f"连接到 {ws_uri} ...")
        try:
            async with websockets.connect(ws_uri) as websocket:
                log("✓ 已连接到WebSocket服务器")

                passed = 0
                failed = 0
                for scenario in selected:
                    success = await run_scenario(websocket, scenario)
                    passed += 1 if success else 0
                    failed += 0 if success else 1
                    if len(selected) > 1:
                        await asyncio.sleep(1.5)

                if len(selected) > 1:
                    print(f"\n{'━' * 80}")
                    log(f"测试汇总: 共 {len(selected)} 个场景, 通过 {passed}, 失败 {failed}")
                    print("━" * 80)
        except ConnectionRefusedError:
            log("✗ 无法连接到WebSocket服务器，请确保Agent已启动")
        except Exception as e:
            log(f"✗ 连接失败: {e}")

        if choice == "0":
            break


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="4自由度头颈控制场景测试工具")
    parser.add_argument("--ws", default=DEFAULT_WS_URI, help=f"WebSocket地址，默认 {DEFAULT_WS_URI}")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    print("4自由度头颈控制 · 场景测试工具")
    print("请确保以下服务已启动:")
    print("  1. SmartRobotAgent (WebSocket端口 8766)")
    print("  2. 四联组合电机控制下游/模拟节点 (/four_combine_motor_control_result)")
    print("  3. 本脚本只测头颈，不会下发底盘控制")
    print()
    asyncio.run(main(args.ws))
