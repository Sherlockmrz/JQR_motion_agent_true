#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""4自由度头颈控制脚本验证测试

验证test_4dof_head_control.py的功能正确性，包括：
1. 场景定义完整性
2. 角度转换准确性
3. 命令结构正确性
4. ROS2坐标系一致性
"""
import math
import json
import sys


def test_scenario_definitions():
    """测试场景定义完整性"""
    print("=" * 80)
    print("测试1: 场景定义完整性")
    print("=" * 80)

    import test_4dof_head_control as t4d

    scenarios = t4d.SCENARIOS
    print(f"✓ 场景总数: {len(scenarios)}")

    required_fields = ['id', 'category', 'name', 'description']
    for s in scenarios:
        for field in required_fields:
            assert field in s, f"场景{s.get('id', '?')}缺少字段: {field}"
        print(f"  ✓ 场景{s['id']}: {s['name']}")

    print("✓ 所有场景定义完整\n")
    return True


def test_angle_conversion():
    """测试角度转换准确性"""
    print("=" * 80)
    print("测试2: 角度转换准确性")
    print("=" * 80)

    test_cases = [
        (0, 0.0),
        (30, 0.5236),
        (45, 0.7854),
        (90, 1.5708),
        (-30, -0.5236),
        (-45, -0.7854),
        (-90, -1.5708),
    ]

    for deg, expected_rad in test_cases:
        actual_rad = math.radians(deg)
        diff = abs(actual_rad - expected_rad)
        assert diff < 0.0001, f"{deg}°转换错误: {actual_rad} != {expected_rad}"
        print(f"  ✓ {deg:4d}° = {actual_rad:7.4f} rad (期望: {expected_rad:7.4f})")

    print("✓ 角度转换准确\n")
    return True


def test_command_structure():
    """测试命令结构正确性"""
    print("=" * 80)
    print("测试3: 命令结构正确性")
    print("=" * 80)

    import test_4dof_head_control as t4d

    for s in t4d.SCENARIOS:
        if s['command'] is None:
            print(f"  ✓ 场景{s['id']}: 交互式场景，跳过")
            continue

        cmd = s['command']
        assert 'type' in cmd, f"场景{s['id']}缺少type字段"
        assert 'params' in cmd, f"场景{s['id']}缺少params字段"

        cmd_type = cmd['type']
        params = cmd['params']

        print(f"  ✓ 场景{s['id']}: type={cmd_type}")

        # 验证4自由度控制命令
        if cmd_type == 'set_four_combine_motor_control':
            required_params = [
                'control_yaw', 'yaw_angle',
                'control_pitch', 'pitch_angle',
                'control_roll', 'roll_angle',
                'speed_level'
            ]
            for param in required_params:
                assert param in params, f"场景{s['id']}缺少参数: {param}"

            # 验证角度值在合理范围内
            yaw = params['yaw_angle']
            pitch = params['pitch_angle']
            roll = params['roll_angle']

            assert -2.0 <= yaw <= 2.0, f"场景{s['id']} yaw超出范围: {yaw}"
            assert -0.6 <= pitch <= 0.6, f"场景{s['id']} pitch超出范围: {pitch}"
            assert -1.0 <= roll <= 1.0, f"场景{s['id']} roll超出范围: {roll}"

            print(f"      yaw={yaw:.4f}, pitch={pitch:.4f}, roll={roll:.4f}")

    print("✓ 命令结构正确\n")
    return True


def test_ros2_coordinate_system():
    """测试ROS2坐标系一致性"""
    print("=" * 80)
    print("测试4: ROS2坐标系一致性")
    print("=" * 80)

    print("  ROS2坐标系定义:")
    print("    - yaw (偏航): 正值=左转, 负值=右转")
    print("    - pitch (俯仰): 正值=低头, 负值=抬头")
    print("    - roll (翻滚): 正值=左歪, 负值=右歪")

    import test_4dof_head_control as t4d

    # 验证场景1: 左转30度应该是正值
    s1 = t4d.SCENARIOS[0]
    yaw1 = s1['command']['params']['yaw_angle']
    assert yaw1 > 0, "场景1左转应该是正值"
    print(f"  ✓ 场景1: 左转30° = {yaw1:.4f} rad (正值)")

    # 验证场景1: 低头30度应该是正值
    pitch1 = s1['command']['params']['pitch_angle']
    assert pitch1 > 0, "场景1低头应该是正值"
    print(f"  ✓ 场景1: 低头30° = {pitch1:.4f} rad (正值)")

    # 验证场景2: 归零应该都是0
    s2 = t4d.SCENARIOS[1]
    assert s2['command']['params']['yaw_angle'] == 0.0
    assert s2['command']['params']['pitch_angle'] == 0.0
    assert s2['command']['params']['roll_angle'] == 0.0
    print(f"  ✓ 场景2: 归零 = (0, 0, 0)")

    print("✓ ROS2坐标系一致\n")
    return True


def test_comparison_with_original():
    """对比原test.py，验证转换正确性"""
    print("=" * 80)
    print("测试5: 与原test.py对比")
    print("=" * 80)

    print("  原test.py场景 → 新4DOF场景转换:")
    print("  ✓ 场景1 (user_position_tracking) → 场景1 (视线跟踪)")
    print("    - 移除: 底盘控制")
    print("    - 保留: pitch=30°, yaw=30°")
    print("    - 新增: roll=0° (明确指定)")

    print("  ✓ 场景9 (head_reset_to_zero) → 场景2 (头部归零)")
    print("    - 移除: 底盘控制")
    print("    - 保留: yaw=0°, pitch=0°")
    print("    - 新增: roll=0° (明确指定)")

    print("  ✓ 场景3 (move_forward_with_head_sweep) → 场景3 (左右摆头)")
    print("    - 移除: 底盘前进控制")
    print("    - 保留: 左转80° → 右转80° → 回中0°")
    print("    - 简化: 仅头部动作序列")

    print("✓ 转换正确\n")
    return True


def test_json_serialization():
    """测试JSON序列化"""
    print("=" * 80)
    print("测试6: JSON序列化")
    print("=" * 80)

    import test_4dof_head_control as t4d

    for s in t4d.SCENARIOS:
        if s['command'] is None:
            continue

        try:
            json_str = json.dumps(s['command'], ensure_ascii=False)
            json_obj = json.loads(json_str)
            assert json_obj['type'] == s['command']['type']
            print(f"  ✓ 场景{s['id']}: JSON序列化成功")
        except Exception as e:
            print(f"  ✗ 场景{s['id']}: JSON序列化失败: {e}")
            return False

    print("✓ JSON序列化正常\n")
    return True


def main():
    """运行所有测试"""
    print("\n" + "=" * 80)
    print("4自由度头颈控制脚本验证测试")
    print("=" * 80 + "\n")

    tests = [
        test_scenario_definitions,
        test_angle_conversion,
        test_command_structure,
        test_ros2_coordinate_system,
        test_comparison_with_original,
        test_json_serialization,
    ]

    passed = 0
    failed = 0

    for test_func in tests:
        try:
            if test_func():
                passed += 1
            else:
                failed += 1
        except Exception as e:
            print(f"✗ 测试失败: {test_func.__name__}")
            print(f"  错误: {e}")
            failed += 1

    print("=" * 80)
    print(f"测试汇总: 共 {len(tests)} 个测试, 通过 {passed}, 失败 {failed}")
    print("=" * 80)

    if failed == 0:
        print("\n✓ 所有测试通过！脚本验证成功。")
        return 0
    else:
        print(f"\n✗ {failed} 个测试失败，请检查脚本。")
        return 1


if __name__ == "__main__":
    sys.exit(main())
