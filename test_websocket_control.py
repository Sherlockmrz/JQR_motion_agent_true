#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WebSocket控制接口测试脚本

按需求场景逐个测试组合电机控制接口，每个场景对应agent中的一个任务方法。
支持选择单个场景或全部运行。
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


# ========================
# 场景定义
# ========================
SCENARIOS = [
    # ---------- 交互中 / 用户对话 ----------
    {
        "id": 1,
        "category": "交互中 / 用户对话",
        "name": "用户移动位置时的视线跟踪（user_position_tracking）",
        "description": (
            "用户从机器人正前方起身，走到侧面继续提问。\n"
            "  头部: 俯仰0°→45°, 水平0°→45°, 同时运动, 速度适中\n"
            "  底盘: 保持静止\n"
            "  参数: yaw_angle(弧度), pitch_angle(弧度), 255表示使用默认值"
        ),
        "command": {
            "type": "user_position_tracking",
            "params": {
                "yaw_angle": -30,  # 255表示使用默认值
                "pitch_angle": -30
            }
        }
    },
    # ---------- 行走/巡逻 ----------
    {
        "id": 2,
        "category": "行走/巡逻",
        "name": "巡逻中停至桌子识别记忆物品（patrol_table_inspection）",
        "description": (
            "机器人巡逻中检测到桌子，靠近停稳后扫描桌面物品。\n"
            "  头部: 俯视→左扫→右扫→回正\n"
            "  底盘: 识别期间保持静止\n"
            "  多步骤: 步骤1-俯视-15° → 步骤2-左扫-45° → 步骤3-右扫45° → 步骤4-回正"
        ),
        "command": {
            "type": "patrol_table_inspection",
            "params": {}
        }
    },
    # ---------- 唤醒 / 静止状态 ----------
    {
        "id": 3,
        "category": "唤醒 / 静止状态",
        "name": "声源在头部转角范围内（wake_head_range）",
        "description": (
            "机器人静止充电，用户在正前方45°范围内唤醒。\n"
            "  头部: 快速转向声源, 俯仰0°→45°, 水平0°→45°, 同时运动\n"
            "  底盘: 保持静止\n"
            "  参数: yaw_angle(弧度), pitch_angle(弧度), 255表示使用默认值"
        ),
        "command": {
            "type": "wake_head_range",
            "params": {
                "yaw_angle": -30,
                "pitch_angle": -30
            }
        }
    },
    {
        "id": 4,
        "category": "唤醒 / 静止状态",
        "name": "声源超出头部转角极限（wake_beyond_head_range）",
        "description": (
            "机器人静止背对用户，用户在后方唤醒。\n"
            "  头部: 先转至极限(俯仰45°+水平90°), 底盘转向后头部回正\n"
            "  底盘: 辅助原地旋转正对用户\n"
            "  多步骤: 步骤1-头部极限 → 步骤2-底盘旋转90° → 步骤3-头部回正\n"
            "  参数: yaw_angle(弧度), pitch_angle(弧度), 255表示使用默认值"
        ),
        "command": {
            "type": "wake_beyond_head_range",
            "params": {
                "yaw_angle": 255,
                "pitch_angle": 255
            }
        }
    },
    # ---------- 唤醒 / 运动状态 ----------
    {
        "id": 5,
        "category": "唤醒 / 运动状态",
        "name": "行走中侧方被唤醒（wake_side_moving）",
        "description": (
            "机器人行走中，用户坐在左侧唤醒。\n"
            "  头部: 先转向声源(水平45°), 底盘转向后头部回正\n"
            "  底盘: 行走中平滑左转45°\n"
            "  多步骤: 步骤1-头部偏航45° → 步骤2-底盘旋转45° → 步骤3-头部回正\n"
            "  参数: yaw_angle(弧度), 255表示使用默认值"
        ),
        "command": {
            "type": "wake_side_moving",
            "params": {
                "yaw_angle": 255
            }
        }
    },
    {
        "id": 6,
        "category": "唤醒 / 运动状态",
        "name": "行走中后方被唤醒并停止（wake_back_moving）",
        "description": (
            "机器人巡逻中，用户在后方喊停并唤醒。\n"
            "  头部: 先转至极限(水平90°), 底盘转向后头部回正\n"
            "  底盘: 减速并原地旋转180°正对用户\n"
            "  多步骤: 步骤1-头部偏航90° → 步骤2-底盘旋转180° → 步骤3-头部回正\n"
            "  参数: yaw_angle(弧度), 255表示使用默认值"
        ),
        "command": {
            "type": "wake_back_moving",
            "params": {
                "yaw_angle": 255
            }
        }
    },
    # ---------- 导航 / 避障 ----------
    {
        "id": 7,
        "category": "导航 / 避障",
        "name": "绕行障碍物时的协同转向（obstacle_avoidance_turn）",
        "description": (
            "机器人遇到障碍物需要绕行，路径需要向右转弯。\n"
            "  头部: 提前向绕行方向（右侧）缓慢预转，引导视线 → 水平0°→45°\n"
            "  底盘: 执行转向动作，头部同步回正\n"
            "  多步骤: 步骤1-头部缓慢右转45°(低速) → 步骤2-底盘右转+头部回正(快速)\n"
            "  速度: 底盘快速避障(speed=2), 头部慢速引导(speed=0, 30°/s)\n"
            "  参数: turn_angle(弧度), head_speed(°/s), 均可省略(默认45°, 30°/s)"
        ),
        "command": {
            "type": "obstacle_avoidance_turn",
            "params": {
                "turn_angle": 0.785,  # 45°右转
                "head_speed": 30
            }
        }
    },
    # ---------- 头部控制 ----------
    {
        "id": 8,
        "category": "头部电机归零位",
        "name": "头部电机回归0位（head_reset_to_zero）",
        "description": (
            "将头部的yaw和pitch都回归到0度位置。\n"
            "  头部: yaw 0°, pitch 0°\n"
            "  底盘: 保持静止"
        ),
        "command": {
            "type": "head_reset_to_zero",
            "params": {}
        }
    },
    # ---------- 手动输入控制 ----------
    {
        "id": 9,
        "category": "手动输入控制",
        "name": "头部电机手动控制（输入pitch/yaw角度）",
        "description": (
            "手动输入头部俯仰(pitch)和偏航(yaw)角度（单位：度），转换为弧度后下发控制。\n"
            "  pitch: 正值=抬头, 负值=低头\n"
            "  yaw: 正值=左转, 负值=右转\n"
            "  输入0跳过该轴控制"
        ),
        "command": None,  # 交互式输入，不使用预设command
        "interactive": "head_motor"
    },
    {
        "id": 10,
        "category": "手动输入控制",
        "name": "底盘手动控制（输入前进距离/旋转角度）",
        "description": (
            "手动输入底盘前进距离(米)和旋转角度(度)，转换后下发控制。\n"
            "  前进: 正值=前进, 负值=后退, 单位：米\n"
            "  旋转: 正值=逆时针, 负值=顺时针, 单位：度→弧度\n"
            "  输入0跳过该项控制\n"
            "  速度档位: 0=低速, 1=中速, 2=快速"
        ),
        "command": None,  # 交互式输入，不使用预设command
        "interactive": "chassis"
    },
]


def print_scenario_menu():
    """打印场景选择菜单"""
    print("\n" + "=" * 80)
    print("  组合电机控制 · 场景测试")
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


def build_head_motor_command():
    """交互式构建头部电机控制命令"""
    print("  ── 头部电机参数输入 ──")
    pitch_deg = input_float("    pitch角度(度, 正=抬头, 负=低头, 0=不控制): ", 0.0)
    yaw_deg = input_float("    yaw角度(度, 正=左转, 负=右转, 0=不控制): ", 0.0)
    speed = input_int("    速度档位(0=低速, 1=中速, 2=快速) [默认1]: ", 1)

    control_pitch = True
    control_yaw = True

    # if not control_pitch and not control_yaw:
    #     print("  ⚠ pitch和yaw都为0，无操作")
    #     return None

    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)

    print(f"  → pitch={pitch_deg}°({pitch_rad:.4f}rad), yaw={yaw_deg}°({yaw_rad:.4f}rad), speed={speed}")

    return {
        "type": "set_combine_motor_control",
        "params": {
            "control_pitch": control_pitch,
            "pitch_angle": pitch_rad,
            "control_yaw": control_yaw,
            "yaw_angle": yaw_rad,
            "speed_level": speed
        }
    }


def build_chassis_command():
    """交互式构建底盘控制命令"""
    print("  ── 底盘控制参数输入 ──")
    move_dist = input_float("    前进距离(米, 正=前进, 负=后退, 0=不移动): ", 0.0)
    rotate_deg = input_float("    旋转角度(度, 正=逆时针, 负=顺时针, 0=不旋转): ", 0.0)
    speed = input_int("    速度档位(0=低速, 1=中速, 2=快速) [默认1]: ", 1)

    control_move = move_dist != 0.0
    control_rotate = rotate_deg != 0.0

    if not control_move and not control_rotate:
        print("  ⚠ 移动和旋转都为0，无操作")
        return None

    rotate_rad = math.radians(rotate_deg)

    print(f"  → 移动={move_dist}m, 旋转={rotate_deg}°({rotate_rad:.4f}rad), speed={speed}")

    return {
        "type": "set_combine_motor_control",
        "params": {
            "control_chassis_move": control_move,
            "chassis_offset": move_dist,
            "control_chassis_rotate": control_rotate,
            "chassis_rotation": rotate_rad,
            "speed_level": speed
        }
    }


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
        if interactive == "head_motor":
            command = build_head_motor_command()
        elif interactive == "chassis":
            command = build_chassis_command()
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
            log("✗ 无法连接到WebSocket服务器，请确保Agent和mock_motor_node已启动")
        except Exception as e:
            log(f"✗ 连接失败: {e}")

        # 单场景测试完后回到菜单继续选择
        if choice != '0':
            continue
        else:
            break


if __name__ == "__main__":
    print("组合电机控制 · 场景测试工具")
    print("请确保以下服务已启动:")
    print("  1. SmartRobotAgent (WebSocket端口 8766)")
    # print("  2. mock_motor_node.py --mode progress")
    print()
    asyncio.run(main())
