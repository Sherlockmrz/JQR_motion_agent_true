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
    """交互式测试模式"""
    
    ws_uri = "ws://localhost:8766"
    
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 交互式WebSocket控制测试")
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 连接到: {ws_uri}")
    print("输入命令格式: type params_json")
    print("示例: go_to_door {}")
    print("示例: set_rgb {\"switch\": true, \"color\": \"green\"}")
    print("输入 'quit' 退出")
    print("-" * 80)
    
    try:
        async with websockets.connect(ws_uri) as websocket:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✓ 已连接")
            
            while True:
                try:
                    # 获取用户输入
                    user_input = input("\n请输入命令: ").strip()
                    
                    if user_input.lower() == 'quit':
                        print("退出交互式测试")
                        break
                    
                    # 解析输入
                    parts = user_input.split(maxsplit=1)
                    if len(parts) == 0:
                        continue
                    
                    task_type = parts[0]
                    params = {}
                    
                    if len(parts) > 1:
                        try:
                            params = json.loads(parts[1])
                        except json.JSONDecodeError:
                            print(f"✗ 参数JSON格式错误")
                            continue
                    
                    # 构造命令
                    command = {
                        "type": task_type,
                        "params": params
                    }
                    
                    # 发送命令
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 发送: {json.dumps(command, ensure_ascii=False)}")
                    await websocket.send(json.dumps(command, ensure_ascii=False))
                    
                    # 接收响应
                    response_str = await asyncio.wait_for(websocket.recv(), timeout=60.0)
                    response = json.loads(response_str)
                    
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 响应:")
                    print(json.dumps(response, ensure_ascii=False, indent=2))
                    
                except asyncio.TimeoutError:
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✗ 等待响应超时")
                except KeyboardInterrupt:
                    print("\n退出交互式测试")
                    break
                except Exception as e:
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ✗ 错误: {e}")
                    
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
