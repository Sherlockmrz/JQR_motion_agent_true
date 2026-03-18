#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WebSocket控制接口测试脚本

用于测试智能机器人Agent的WebSocket控制功能
"""
import asyncio
import json
import time
from datetime import datetime
import websockets


async def test_websocket_control():
    """测试WebSocket控制接口"""
    
    # WebSocket服务器地址
    ws_uri = "ws://localhost:8766"
    
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 开始测试WebSocket控制接口")
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 连接到: {ws_uri}")
    print("-" * 80)
    
    try:
        # 连接到WebSocket服务器
        async with websockets.connect(ws_uri) as websocket:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✓ 已成功连接到WebSocket服务器")
            
            # 测试用例列表
            test_cases = [
                {
                    "name": "测试go_to_door命令",
                    "command": {
                        "type": "go_to_door",
                        "params": {}
                    }
                },
                {
                    "name": "测试get_move_mode命令",
                    "command": {
                        "type": "get_move_mode",
                        "params": {}
                    }
                },
                {
                    "name": "测试get_robot_rise_state命令",
                    "command": {
                        "type": "get_robot_rise_state",
                        "params": {}
                    }
                },
                {
                    "name": "测试set_rgb命令",
                    "command": {
                        "type": "set_rgb",
                        "params": {
                            "switch": True,
                            "color": "green",
                            "mode": 0
                        }
                    }
                },
                {
                    "name": "测试无效命令",
                    "command": {
                        "type": "invalid_command",
                        "params": {}
                    }
                },
                {
                    "name": "测试缺少type字段",
                    "command": {
                        "params": {}
                    }
                }
            ]
            
            # 执行测试用例
            for i, test_case in enumerate(test_cases, 1):
                print(f"\n[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 测试用例 {i}: {test_case['name']}")
                print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 发送命令: {json.dumps(test_case['command'], ensure_ascii=False)}")
                
                # 发送命令
                start_time = time.time()
                await websocket.send(json.dumps(test_case['command'], ensure_ascii=False))
                
                # 接收响应
                try:
                    response_str = await asyncio.wait_for(websocket.recv(), timeout=30.0)
                    elapsed = time.time() - start_time
                    
                    response = json.loads(response_str)
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 收到响应 (耗时: {elapsed:.2f}s):")
                    print(f"  success: {response.get('success')}")
                    print(f"  error_msg: {response.get('error_msg', '')}")
                    
                    # 打印完整响应（格式化）
                    print(f"  完整响应: {json.dumps(response, ensure_ascii=False, indent=2)}")
                    
                except asyncio.TimeoutError:
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✗ 等待响应超时")
                except json.JSONDecodeError as e:
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✗ JSON解析失败: {e}")
                
                # 测试间隔
                if i < len(test_cases):
                    await asyncio.sleep(1)
            
            print("\n" + "-" * 80)
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 所有测试用例执行完成")
            
    except ConnectionRefusedError:
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✗ 无法连接到WebSocket服务器")
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 请确保SmartRobotAgent已启动并运行WebSocket服务")
    except OSError as e:
        if "Connect call failed" in str(e) or "Connection refused" in str(e):
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✗ 无法连接到WebSocket服务器")
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 请确保SmartRobotAgent已启动并运行WebSocket服务")
        else:
            raise
    except Exception as e:
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✗ 测试失败: {e}")
        import traceback
        traceback.print_exc()


async def interactive_test():
    """交互式测试模式 - 菜单选择方式"""
    
    ws_uri = "ws://localhost:8766"
    
    # 可用命令列表
    commands = [
        {"name": "go_to_door", "desc": "前往门口", "params": {}},
        {"name": "go_to_elevator", "desc": "前往电梯", "params": {}},
        {"name": "stop_navigate", "desc": "停止导航", "params": {}},
        {"name": "get_move_mode", "desc": "获取移动模式", "params": {}},
        {"name": "get_robot_rise_state", "desc": "获取升降状态", "params": {}},
        {"name": "set_robot_rise", "desc": "控制升降 (输入参数)", "params": {"rise": True}},
        {"name": "get_screen_tilt_state", "desc": "获取屏幕俯仰状态", "params": {}},
        {"name": "set_screen_tilt", "desc": "控制屏幕俯仰 (输入参数)", "params": {"tilt": 0}},
        {"name": "get_rgb_light_strip_state", "desc": "获取RGB灯状态", "params": {}},
        {"name": "set_rgb", "desc": "设置RGB灯 (输入参数)", "params": {"switch": True, "color": "green", "mode": 0}},
        {"name": "set_head_motor_control", "desc": "头部电机控制 (输入参数)", "params": {"control_pitch": True, "pitch_angle": 0.0, "control_yaw": True, "yaw_angle": 0.0}},
        {"name": "get_battery_level", "desc": "获取电池电量", "params": {}},
        {"name": "get_robot_position", "desc": "获取机器人位置", "params": {}},
    ]
    
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 交互式WebSocket控制测试")
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 连接到: {ws_uri}")
    print("-" * 80)
    
    try:
        async with websockets.connect(ws_uri) as websocket:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✓ 已连接\n")
            
            while True:
                try:
                    # 显示菜单
                    print("=" * 60)
                    print("可用命令列表:")
                    print("-" * 60)
                    for i, cmd in enumerate(commands, 1):
                        print(f"  {i:2d}. {cmd['name']:30s} - {cmd['desc']}")
                    print("-" * 60)
                    print("  q. 退出")
                    print("=" * 60)
                    
                    # 获取用户选择
                    choice = input("\n请选择命令编号: ").strip()
                    
                    if choice.lower() == 'q':
                        print("退出交互式测试")
                        break
                    
                    # 解析选择
                    try:
                        idx = int(choice) - 1
                        if idx < 0 or idx >= len(commands):
                            print("✗ 无效的编号，请重新选择\n")
                            continue
                    except ValueError:
                        print("✗ 请输入数字或 'q' 退出\n")
                        continue
                    
                    # 获取选中的命令
                    selected = commands[idx]
                    task_type = selected['name']
                    params = selected['params'].copy()
                    
                    # 对于需要参数的命令，让用户输入
                    if params:
                        print(f"\n命令: {task_type}")
                        print(f"默认参数: {json.dumps(params, ensure_ascii=False)}")
                        custom = input("使用默认参数? (Y/n/输入JSON): ").strip()
                        
                        if custom.lower() == 'n':
                            # 让用户输入每个参数
                            for key in params:
                                val = input(f"  {key} (默认: {params[key]}): ").strip()
                                if val:
                                    # 尝试解析类型
                                    try:
                                        if isinstance(params[key], bool):
                                            params[key] = val.lower() in ('true', '1', 'yes', 'y')
                                        elif isinstance(params[key], int):
                                            params[key] = int(val)
                                        elif isinstance(params[key], float):
                                            params[key] = float(val)
                                        else:
                                            params[key] = val
                                    except ValueError:
                                        pass  # 保持原值
                        elif custom and custom.lower() != 'y':
                            # 用户输入完整JSON
                            try:
                                params = json.loads(custom)
                            except json.JSONDecodeError:
                                print("✗ JSON格式错误，使用默认参数")
                    
                    # 构造命令
                    command = {"type": task_type, "params": params}
                    
                    # 发送命令
                    print(f"\n[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 发送: {json.dumps(command, ensure_ascii=False)}")
                    await websocket.send(json.dumps(command, ensure_ascii=False))
                    
                    # 接收响应
                    response_str = await asyncio.wait_for(websocket.recv(), timeout=60.0)
                    response = json.loads(response_str)
                    
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 响应:")
                    print(json.dumps(response, ensure_ascii=False, indent=2))
                    print()
                    
                except asyncio.TimeoutError:
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✗ 等待响应超时\n")
                except KeyboardInterrupt:
                    print("\n退出交互式测试")
                    break
                except Exception as e:
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✗ 错误: {e}\n")
                    
    except ConnectionRefusedError:
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✗ 无法连接到WebSocket服务器")
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 请确保SmartRobotAgent已启动并运行WebSocket服务")
    except OSError as e:
        if "Connect call failed" in str(e) or "Connection refused" in str(e):
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✗ 无法连接到WebSocket服务器")
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 请确保SmartRobotAgent已启动并运行WebSocket服务")
        else:
            raise
    except Exception as e:
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✗ 连接失败: {e}")


if __name__ == "__main__":
    import sys
    
    print("WebSocket控制接口测试工具")
    print("=" * 80)
    print("1. 自动测试模式（运行预设测试用例）")
    print("2. 交互式测试模式（手动输入命令）")
    print("=" * 80)
    
    choice = input("请选择模式 (1/2): ").strip()
    
    if choice == "1":
        asyncio.run(test_websocket_control())
    elif choice == "2":
        asyncio.run(interactive_test())
    else:
        print("无效选择，退出")
        sys.exit(1)
