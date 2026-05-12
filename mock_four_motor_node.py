#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四自由度头颈运控组合电机控制模拟节点

模拟下游电机控制器，订阅 /four_combine_motor_control 话题，
根据指令模拟执行并在 /four_combine_motor_control_result 话题发布反馈。

话题数据类型：std_msgs/msg/Float32MultiArray（12字段）
    data[0]  task_id
    data[1]  control_yaw (0.0/1.0)            # 是否控制偏航
    data[2]  yaw_angle (rad)                  # 偏航目标角度
    data[3]  control_roll (0.0/1.0)           # 是否控制翻滚
    data[4]  roll_angle (rad)                 # 翻滚目标角度
    data[5]  control_pitch (0.0/1.0)          # 是否控制俯仰
    data[6]  pitch_angle (rad)                # 俯仰目标角度
    data[7]  control_chassis_move (0.0/1.0)   # 是否控制底盘位移
    data[8]  chassis_offset (m)               # +前进 -后退
    data[9]  control_chassis_rotate (0.0/1.0) # 是否控制底盘旋转
    data[10] chassis_rotation (rad)           # +逆时针 -顺时针
    data[11] speed_level                      # 0=低速 1=中速 2=快速, 其它按0处理

用法:
    python3 mock_four_motor_node.py                      # 默认成功
    python3 mock_four_motor_node.py --mode fail          # 103 执行失败
    python3 mock_four_motor_node.py --mode abort         # 102 执行中止
    python3 mock_four_motor_node.py --mode reject        # 104 拒绝执行
    python3 mock_four_motor_node.py --mode random        # 随机结果
    python3 mock_four_motor_node.py --mode progress      # 带进度上报的成功
    python3 mock_four_motor_node.py --delay 2.0          # 指定模拟延迟
"""
import argparse
import math
import random
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[ERROR] ROS2 (rclpy) 不可用，请先 source ROS2 环境")
    exit(1)


RESULT_SUCCESS = 101.0
RESULT_ABORT = 102.0
RESULT_FAIL = 103.0
RESULT_REJECT = 104.0

SPEED_NAMES = {0: "低速", 1: "中速", 2: "快速"}
FIELD_COUNT = 12


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


class MockFourMotorNode(Node):
    """四自由度头颈运控组合电机控制模拟节点"""

    def __init__(self, mode: str = "success", delay=None):
        super().__init__('mock_four_motor_controller')
        self.mode = mode
        self.delay = delay

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/four_combine_motor_control',
            self._on_motor_command,
            10,
        )

        self.result_publisher = self.create_publisher(
            Float32MultiArray,
            '/four_combine_motor_control_result',
            10,
        )

        self.get_logger().info(
            f"四联模拟电机节点启动 | 模式: {mode} | 延迟: {delay if delay is not None else '自动'}"
        )
        self.get_logger().info("订阅: /four_combine_motor_control")
        self.get_logger().info("发布: /four_combine_motor_control_result")

    def _on_motor_command(self, msg):
        data = msg.data
        if len(data) < FIELD_COUNT:
            self.get_logger().error(f"数据长度不足: {len(data)}, 需要 {FIELD_COUNT} 个字段")
            return

        task_id = data[0]
        control_yaw = data[1] == 1.0
        yaw_angle = data[2]
        control_roll = data[3] == 1.0
        roll_angle = data[4]
        control_pitch = data[5] == 1.0
        pitch_angle = data[6]
        control_chassis_move = data[7] == 1.0
        chassis_offset = data[8]
        control_chassis_rotate = data[9] == 1.0
        chassis_rotation = data[10]
        raw_speed = int(data[11])
        speed_level = raw_speed if raw_speed in (0, 1, 2) else 0

        self.get_logger().info("=" * 64)
        self.get_logger().info(f"收到四自由度头颈运控指令 | task_id={task_id:.0f}")
        if control_yaw:
            self.get_logger().info(f"  偏航 (yaw):   {yaw_angle:.4f} rad ({rad2deg(yaw_angle):.1f}°)")
        if control_roll:
            self.get_logger().info(f"  翻滚 (roll):  {roll_angle:.4f} rad ({rad2deg(roll_angle):.1f}°)")
        if control_pitch:
            self.get_logger().info(f"  俯仰 (pitch): {pitch_angle:.4f} rad ({rad2deg(pitch_angle):.1f}°)")
        if control_chassis_move:
            direction = "前进" if chassis_offset > 0 else "后退"
            self.get_logger().info(f"  底盘位移: {direction} {abs(chassis_offset):.3f} 米")
        if control_chassis_rotate:
            direction = "逆时针" if chassis_rotation > 0 else "顺时针"
            self.get_logger().info(
                f"  底盘旋转: {direction} {abs(chassis_rotation):.4f} rad ({abs(rad2deg(chassis_rotation)):.1f}°)"
            )
        self.get_logger().info(
            f"  速度档位: {SPEED_NAMES.get(speed_level, '未知')} ({raw_speed}→{speed_level})"
        )

        thread = threading.Thread(
            target=self._simulate_execution,
            args=(task_id, speed_level),
            daemon=True,
        )
        thread.start()

    def _simulate_execution(self, task_id, speed_level):
        if self.delay is not None:
            exec_delay = float(self.delay)
        else:
            delay_map = {0: 3.0, 1: 2.0, 2: 1.0}
            exec_delay = delay_map.get(speed_level, 3.0)

        if self.mode == "success":
            self._simulate_with_progress(task_id, exec_delay, RESULT_SUCCESS)

        elif self.mode == "fail":
            self._simulate_with_progress(task_id, exec_delay * 0.5, RESULT_FAIL)

        elif self.mode == "abort":
            self._simulate_with_progress(task_id, exec_delay * 0.3, RESULT_ABORT)

        elif self.mode == "reject":
            self.get_logger().warn(f"任务 {task_id:.0f} 拒绝执行")
            self._publish_result(task_id, RESULT_REJECT)

        elif self.mode == "progress":
            self._simulate_with_progress(task_id, exec_delay, RESULT_SUCCESS, report_progress=True)

        elif self.mode == "random":
            result_code = random.choice(
                [RESULT_SUCCESS, RESULT_SUCCESS, RESULT_SUCCESS,
                 RESULT_FAIL, RESULT_ABORT, RESULT_REJECT]
            )
            if result_code == RESULT_REJECT:
                self.get_logger().warn(f"任务 {task_id:.0f} 随机拒绝执行")
                self._publish_result(task_id, RESULT_REJECT)
            elif result_code == RESULT_SUCCESS:
                self._simulate_with_progress(task_id, exec_delay, RESULT_SUCCESS, report_progress=True)
            else:
                fail_point = random.uniform(0.2, 0.8)
                self._simulate_with_progress(task_id, exec_delay * fail_point, result_code)

    def _simulate_with_progress(self, task_id, total_delay, final_result, report_progress=False):
        if report_progress and total_delay > 0.5:
            steps = [25.0, 50.0, 75.0]
            step_delay = total_delay / 4.0
            for progress in steps:
                time.sleep(step_delay)
                self.get_logger().info(f"任务 {task_id:.0f} 进度: {progress:.0f}%")
                self._publish_result(task_id, progress)
            time.sleep(step_delay)
        else:
            time.sleep(total_delay)

        result_name = {
            RESULT_SUCCESS: "成功",
            RESULT_FAIL: "失败",
            RESULT_ABORT: "中止",
            RESULT_REJECT: "拒绝",
        }.get(final_result, f"未知({final_result})")

        self.get_logger().info(
            f"任务 {task_id:.0f} 执行结果: {result_name} ({final_result:.0f})"
        )
        self._publish_result(task_id, final_result)

    def _publish_result(self, task_id, result_code):
        msg = Float32MultiArray()
        msg.data = [float(task_id), float(result_code)]
        self.result_publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="四自由度头颈运控组合电机控制模拟节点")
    parser.add_argument(
        "--mode", type=str, default="success",
        choices=["success", "fail", "abort", "reject", "random", "progress"],
        help="模拟模式",
    )
    parser.add_argument(
        "--delay", type=float, default=None,
        help="模拟执行延迟（秒），默认根据速度档位自动计算",
    )
    args = parser.parse_args()

    rclpy.init()
    node = MockFourMotorNode(mode=args.mode, delay=args.delay)

    try:
        print("\n四联模拟电机节点运行中，等待指令... (Ctrl+C 退出)\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n正在关闭四联模拟电机节点...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
