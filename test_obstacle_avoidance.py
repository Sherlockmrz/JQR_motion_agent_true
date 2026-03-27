#!/usr/bin/env python3
"""
障碍物绕行协同转向场景测试脚本

测试场景：obstacle_avoidance_turn
- 头部预转右侧45°（慢速引导视线）
- 底盘右转45° + 头部回正（快速避障）
"""

import asyncio
import websockets
import json
import math


async def test_obstacle_avoidance():
    """测试障碍物绕行协同转向场景"""
    uri = "ws://localhost:8766"

    print("=" * 60)
    print("障碍物绕行协同转向场景测试")
    print("=" * 60)

    try:
        async with websockets.connect(uri) as websocket:
            print(f"✓ 已连接到 WebSocket 服务器: {uri}")

            # 测试场景：障碍物绕行右转45°
            test_case = {
                "type": "obstacle_avoidance_turn",
                "params": {
                    "turn_angle": math.radians(45),  # 45°右转
                    "head_speed": 30  # 30°/s
                }
            }

            print(f"\n发送测试指令:")
            print(f"  场景: {test_case['type']}")
            print(f"  转向角度: {math.degrees(test_case['params']['turn_angle']):.1f}°")
            print(f"  头部转速: {test_case['params']['head_speed']}°/s")

            # 发送指令
            await websocket.send(json.dumps(test_case))
            print(f"✓ 指令已发送")

            # 接收响应
            print(f"\n等待执行结果...")
            response = await websocket.recv()
            result = json.loads(response)

            print(f"\n收到响应:")
            print(json.dumps(result, indent=2, ensure_ascii=False))

            # 验证结果
            if result.get("success"):
                print(f"\n✅ 测试通过")
                print(f"   场景类型: {result.get('type')}")
                print(f"   执行状态: 成功")
                return True
            else:
                print(f"\n❌ 测试失败")
                print(f"   错误信息: {result.get('error_msg', '未知错误')}")
                return False

    except ConnectionRefusedError:
        print(f"\n❌ 连接失败: 无法连接到 {uri}")
        print(f"   请确保 Agent 或测试服务器正在运行")
        return False
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        return False


async def test_with_custom_angle():
    """测试自定义转向角度"""
    uri = "ws://localhost:8766"

    print("\n" + "=" * 60)
    print("自定义角度测试（30°右转）")
    print("=" * 60)

    try:
        async with websockets.connect(uri) as websocket:
            test_case = {
                "type": "obstacle_avoidance_turn",
                "params": {
                    "turn_angle": math.radians(30),  # 30°右转
                    "head_speed": 30
                }
            }

            print(f"\n发送测试指令:")
            print(f"  转向角度: {math.degrees(test_case['params']['turn_angle']):.1f}°")

            await websocket.send(json.dumps(test_case))
            response = await websocket.recv()
            result = json.loads(response)

            if result.get("success"):
                print(f"✅ 30°转向测试通过")
                return True
            else:
                print(f"❌ 30°转向测试失败: {result.get('error_msg')}")
                return False

    except Exception as e:
        print(f"❌ 测试异常: {e}")
        return False


async def test_default_params():
    """测试默认参数（不传参数）"""
    uri = "ws://localhost:8766"

    print("\n" + "=" * 60)
    print("默认参数测试")
    print("=" * 60)

    try:
        async with websockets.connect(uri) as websocket:
            test_case = {
                "type": "obstacle_avoidance_turn",
                "params": {}  # 使用默认参数
            }

            print(f"\n发送测试指令（使用默认参数）")

            await websocket.send(json.dumps(test_case))
            response = await websocket.recv()
            result = json.loads(response)

            if result.get("success"):
                print(f"✅ 默认参数测试通过")
                return True
            else:
                print(f"❌ 默认参数测试失败: {result.get('error_msg')}")
                return False

    except Exception as e:
        print(f"❌ 测试异常: {e}")
        return False


async def main():
    """运行所有测试"""
    print("\n" + "=" * 60)
    print("障碍物绕行协同转向 - 完整测试套件")
    print("=" * 60)

    results = []

    # 测试1: 标准45°右转
    result1 = await test_obstacle_avoidance()
    results.append(("标准45°右转", result1))

    await asyncio.sleep(1)

    # 测试2: 自定义30°右转
    result2 = await test_with_custom_angle()
    results.append(("自定义30°右转", result2))

    await asyncio.sleep(1)

    # 测试3: 默认参数
    result3 = await test_default_params()
    results.append(("默认参数", result3))

    # 汇总结果
    print("\n" + "=" * 60)
    print("测试结果汇总")
    print("=" * 60)

    passed = sum(1 for _, r in results if r)
    total = len(results)

    for name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{status} - {name}")

    print(f"\n总计: {passed}/{total} 测试通过")
    print(f"通过率: {passed/total*100:.1f}%")

    if passed == total:
        print("\n🎉 所有测试通过！")
    else:
        print(f"\n⚠️  {total - passed} 个测试失败")


if __name__ == "__main__":
    asyncio.run(main())
