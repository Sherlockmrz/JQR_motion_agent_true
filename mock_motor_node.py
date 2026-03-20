#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""组合电机控制模拟节点

模拟下游电机控制器，订阅 /combine_motor_control 话题，
根据指令模拟执行并在 /combine_motor_control_result 话题发布反馈。

用法:
    # 默认模式（全部成功，模拟执行延迟1-3秒）
    python3 mock_motor_node.py

    # 指定失败模式（模拟特定结果码）
    python3 mock_motor_node.py --mode fail       # 103 执行失败
    python3 mock_motor_node.py --mode abort      # 102 执行中止
    python3 mock_motor_node.py --mode reject     # 104 拒绝执行
    python3 mock_motor_node.py --mode random     # 随机结果（含进度上报）
    python3 mock_motor_node.py --mode progress   # 带进度上报的成功模式

    # 指定模拟延迟（秒）
    python3 mock_motor_node.py --delay 2.0
"""
import argparse
import random
import time
import threading

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[ERROR] ROS2 (rclpy) 不可用，请确保已source ROS2环境")
    exit(1)


# 结果码定义
RESULT_SUCCESS = 101.0
RESULT_ABORT = 102.0
RESULT_FAIL = 103.0
RESULT_REJECT = 104.0

# 速度档位名称
SPEED_NAMES = {0: "低速", 1: "中速", 2: "快速"}


class MockMotorNode(Node):
    """模拟组合电机控制节点"""

    def __init__(self, mode="success", delay=None):
        super().__init__('mock_motor_controller')
        self.mode = mode
        self.delay = delay  # None表示根据速度档位自动计算

        # 订阅 combine_motor_control
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/combine_motor_control',
            self._on_motor_command,
            10
        )

        # 发布 combine_motor_control_result
        self.result_publisher = self.create_publisher(
            Float32MultiArray,
            '/combine_motor_control_result',
            10
        )

        self.get_logger().info(f"模拟电机节点已启动 | 模式: {mode} | 延迟: {delay or '自动'}")
        self.get_logger().info("订阅: /combine_motor_control")
        self.get_logger().info("发布: /combine_motor_control_result")

    def _on_motor_command(self, msg):
        """收到电机控制指令的回调"""
        data = msg.data
        if len(data) < 10:
            self.get_logger().error(f"数据长度不足: {len(data)}, 需要10个字段")
            return

        task_id = data[0]
        control_pitch = data[1] == 1.0
        pitch_angle = data[2]
        control_yaw = data[3] == 1.0
        yaw_angle = data[4]
        control_chassis_move = data[5] == 1.0
        chassis_offset = data[6]
        control_chassis_rotate = data[7] == 1.0
        chassis_rotation = data[8]
        speed_level = int(data[9])

        # 打印收到的指令
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"收到组合电机控制指令 | task_id={task_id:.0f}")
        if control_pitch:
            self.get_logger().info(f"  俯仰控制: {pitch_angle:.4f} rad ({pitch_angle * 180 / 3.14159:.1f}°)")
        if control_yaw:
            self.get_logger().info(f"  偏航控制: {yaw_angle:.4f} rad ({yaw_angle * 180 / 3.14159:.1f}°)")
        if control_chassis_move:
            direction = "前进" if chassis_offset > 0 else "后退"
            self.get_logger().info(f"  底盘位移: {direction} {abs(chassis_offset):.3f} 米")
        if control_chassis_rotate:
            direction = "逆时针" if chassis_rotation > 0 else "顺时针"
            self.get_logger().info(f"  底盘旋转: {direction} {abs(chassis_rotation):.4f} rad ({abs(chassis_rotation) * 180 / 3.14159:.1f}°)")
        self.get_logger().info(f"  速度档位: {SPEED_NAMES.get(speed_level, '未知')} ({speed_level})")

        # 在独立线程中模拟执行（避免阻塞ROS2回调）
        thread = threading.Thread(
            target=self._simulate_execution,
            args=(task_id, speed_level),
            daemon=True
        )
        thread.start()

    def _simulate_execution(self, task_id, speed_level):
        """模拟电机执行过程"""
        # 计算延迟
        if self.delay is not None:
            exec_delay = self.delay
        else:
            # 根据速度档位自动计算延迟
            delay_map = {0: 3.0, 1: 2.0, 2: 1.0}
            exec_delay = delay_map.get(speed_level, 3.0)

        # 根据模式决定结果
        if self.mode == "success":
            self._simulate_with_progress(task_id, exec_delay, RESULT_SUCCESS)

        elif self.mode == "fail":
            # 模拟执行到一半失败
            self._simulate_with_progress(task_id, exec_delay * 0.5, RESULT_FAIL)

        elif self.mode == "abort":
            # 模拟执行到30%中止
            self._simulate_with_progress(task_id, exec_delay * 0.3, RESULT_ABORT)

        elif self.mode == "reject":
            # 立即拒绝
            self.get_logger().warn(f"任务 {task_id:.0f} 拒绝执行")
            self._publish_result(task_id, RESULT_REJECT)

        elif self.mode == "progress":
            # 带完整进度上报的成功模式
            self._simulate_with_progress(task_id, exec_delay, RESULT_SUCCESS, report_progress=True)

        elif self.mode == "random":
            # 随机结果
            result_code = random.choice([RESULT_SUCCESS, RESULT_SUCCESS, RESULT_SUCCESS,
                                          RESULT_FAIL, RESULT_ABORT, RESULT_REJECT])
            if result_code == RESULT_REJECT:
                self.get_logger().warn(f"任务 {task_id:.0f} 随机拒绝执行")
                self._publish_result(task_id, RESULT_REJECT)
            elif result_code == RESULT_SUCCESS:
                self._simulate_with_progress(task_id, exec_delay, RESULT_SUCCESS, report_progress=True)
            else:
                fail_point = random.uniform(0.2, 0.8)
                self._simulate_with_progress(task_id, exec_delay * fail_point, result_code)

    def _simulate_with_progress(self, task_id, total_delay, final_result, report_progress=False):
        """模拟执行并可选上报进度"""
        if report_progress and total_delay > 0.5:
            # 分步上报进度: 0%, 25%, 50%, 75%, 100%
            steps = [25.0, 50.0, 75.0]
            step_delay = total_delay / 4.0

            for progress in steps:
                time.sleep(step_delay)
                self.get_logger().info(f"任务 {task_id:.0f} 进度: {progress:.0f}%")
                self._publish_result(task_id, progress)

            time.sleep(step_delay)
        else:
            time.sleep(total_delay)

        # 发布最终结果
        result_name = {
            RESULT_SUCCESS: "成功",
            RESULT_FAIL: "失败",
            RESULT_ABORT: "中止",
            RESULT_REJECT: "拒绝"
        }.get(final_result, f"未知({final_result})")

        self.get_logger().info(f"任务 {task_id:.0f} 执行结果: {result_name} ({final_result:.0f})")
        self._publish_result(task_id, final_result)

    def _publish_result(self, task_id, result_code):
        """发布执行结果"""
        msg = Float32MultiArray()
        msg.data = [float(task_id), float(result_code)]
        self.result_publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="组合电机控制模拟节点")
    parser.add_argument("--mode", type=str, default="success",
                        choices=["success", "fail", "abort", "reject", "random", "progress"],
                        help="模拟模式: success(成功), fail(失败), abort(中止), reject(拒绝), random(随机), progress(带进度)")
    parser.add_argument("--delay", type=float, default=None,
                        help="模拟执行延迟（秒），默认根据速度档位自动计算")
    args = parser.parse_args()

    rclpy.init()
    node = MockMotorNode(mode=args.mode, delay=args.delay)

    try:
        print("\n模拟电机节点运行中，等待指令... (Ctrl+C 退出)\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n正在关闭模拟电机节点...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
