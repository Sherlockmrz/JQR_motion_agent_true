#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四自由度头颈运控多路点组合电机控制模拟节点

模拟下游电机控制器，订阅 /four_combine_waypoint_control 话题，
根据多路点指令模拟执行并在 /four_combine_waypoint_control_result 话题发布反馈。

话题数据类型：std_msgs/msg/Float32MultiArray
数组总长度：3 + N × 12（N 为路点数量）

---------- 任务头（固定3位） ----------
data[0]  task_id                    # 任务id，确保一个工作周期内id唯一
data[1]  pose_mode                  # 0=相对位姿 1=绝对位姿
data[2]  waypoint_count             # 路点数量N，必须 >= 1

---------- 第i个路点（通式，base = 3 + i*12） ----------
data[base+0]   control_yaw           # 是否控制偏航，0.0/1.0
data[base+1]   yaw_angle             # 偏航目标角度（弧度）
data[base+2]   control_roll          # 是否控制翻滚，0.0/1.0
data[base+3]   roll_angle            # 翻滚目标角度（弧度）
data[base+4]   control_pitch         # 是否控制俯仰，0.0/1.0
data[base+5]   pitch_angle           # 俯仰目标角度（弧度）
data[base+6]   control_chassis_move  # 是否控制底盘位移，0.0/1.0
data[base+7]   chassis_offset        # 底盘位置偏移量（米）
data[base+8]   control_chassis_rotate # 是否控制底盘旋转，0.0/1.0
data[base+9]   chassis_rotation      # 底盘旋转偏移量（弧度）
data[base+10]  speed_level           # 执行档位 0=低速 1=中速 2=快速
data[base+11]  timeout               # 本路点超时时间（秒），0=无限制

用法:
    python3 mock_four_waypoint_node.py                      # 默认成功
    python3 mock_four_waypoint_node.py --mode fail          # 103 执行失败
    python3 mock_four_waypoint_node.py --mode abort         # 102 执行中止
    python3 mock_four_waypoint_node.py --mode reject        # 104 拒绝执行
    python3 mock_four_waypoint_node.py --mode random        # 随机结果
    python3 mock_four_waypoint_node.py --mode progress      # 带进度上报的成功
    python3 mock_four_waypoint_node.py --delay 2.0          # 指定每个路点延迟
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
HEADER_SIZE = 3
WAYPOINT_SIZE = 12


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


class MockFourWaypointNode(Node):
    """四自由度头颈运控多路点组合电机控制模拟节点"""

    def __init__(self, mode: str = "success", delay=None):
        super().__init__('mock_four_waypoint_controller')
        self.mode = mode
        self.delay = delay

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/four_combine_waypoint_control',
            self._on_waypoint_command,
            10,
        )

        self.result_publisher = self.create_publisher(
            Float32MultiArray,
            '/four_combine_waypoint_control_result',
            10,
        )

        self.get_logger().info(
            f"多路点模拟电机节点启动 | 模式: {mode} | 延迟: {delay if delay is not None else '自动'}"
        )
        self.get_logger().info("订阅: /four_combine_waypoint_control")
        self.get_logger().info("发布: /four_combine_waypoint_control_result")

    def _on_waypoint_command(self, msg):
        data = msg.data
        if len(data) < HEADER_SIZE:
            self.get_logger().error(f"数据长度不足: {len(data)}, 需要至少 {HEADER_SIZE} 个头部字段")
            return

        # 解析任务头
        task_id = data[0]
        pose_mode = int(data[1])
        waypoint_count = int(data[2])

        # 验证数据长度
        expected_len = HEADER_SIZE + waypoint_count * WAYPOINT_SIZE
        if len(data) < expected_len:
            self.get_logger().error(
                f"数据长度不足: {len(data)}, 需要 {expected_len} 个字段 "
                f"(头部{HEADER_SIZE} + {waypoint_count}路点×{WAYPOINT_SIZE})"
            )
            return

        pose_mode_str = "相对位姿" if pose_mode == 0 else "绝对位姿"
        self.get_logger().info("=" * 64)
        self.get_logger().info(
            f"收到多路点指令 | task_id={task_id:.0f} | {pose_mode_str} | {waypoint_count}个路点"
        )

        # 解析并打印所有路点
        waypoints = []
        for i in range(waypoint_count):
            base = HEADER_SIZE + i * WAYPOINT_SIZE
            wp = {
                'control_yaw': data[base+0] == 1.0,
                'yaw_angle': data[base+1],
                'control_roll': data[base+2] == 1.0,
                'roll_angle': data[base+3],
                'control_pitch': data[base+4] == 1.0,
                'pitch_angle': data[base+5],
                'control_chassis_move': data[base+6] == 1.0,
                'chassis_offset': data[base+7],
                'control_chassis_rotate': data[base+8] == 1.0,
                'chassis_rotation': data[base+9],
                'speed_level': int(data[base+10]) if int(data[base+10]) in (0, 1, 2) else 0,
                'timeout': data[base+11]
            }
            waypoints.append(wp)
            self._log_waypoint(i, wp)

        # 后台执行
        thread = threading.Thread(
            target=self._simulate_waypoint_execution,
            args=(task_id, waypoints),
            daemon=True,
        )
        thread.start()

    def _log_waypoint(self, idx, wp):
        """打印路点信息"""
        self.get_logger().info(f"--- 路点 {idx} ---")
        if wp['control_yaw']:
            self.get_logger().info(f"  偏航 (yaw):   {wp['yaw_angle']:.4f} rad ({rad2deg(wp['yaw_angle']):.1f}°)")
        if wp['control_roll']:
            self.get_logger().info(f"  翻滚 (roll):  {wp['roll_angle']:.4f} rad ({rad2deg(wp['roll_angle']):.1f}°)")
        if wp['control_pitch']:
            self.get_logger().info(f"  俯仰 (pitch): {wp['pitch_angle']:.4f} rad ({rad2deg(wp['pitch_angle']):.1f}°)")
        if wp['control_chassis_move']:
            direction = "前进" if wp['chassis_offset'] > 0 else "后退"
            self.get_logger().info(f"  底盘位移: {direction} {abs(wp['chassis_offset']):.3f} 米")
        if wp['control_chassis_rotate']:
            direction = "逆时针" if wp['chassis_rotation'] > 0 else "顺时针"
            self.get_logger().info(
                f"  底盘旋转: {direction} {abs(wp['chassis_rotation']):.4f} rad ({abs(rad2deg(wp['chassis_rotation'])):.1f}°)"
            )
        self.get_logger().info(f"  速度档位: {SPEED_NAMES.get(wp['speed_level'], '未知')} ({wp['speed_level']})")
        timeout_str = f"{wp['timeout']:.1f}s" if wp['timeout'] > 0 else "无限制"
        self.get_logger().info(f"  超时时间: {timeout_str}")

    def _simulate_waypoint_execution(self, task_id, waypoints):
        """模拟多路点执行"""
        total_waypoints = len(waypoints)

        # 拒绝模式直接返回
        if self.mode == "reject":
            self.get_logger().warn(f"任务 {task_id:.0f} 拒绝执行")
            self._publish_result(task_id, RESULT_REJECT)
            return

        # 执行每个路点
        for idx, wp in enumerate(waypoints):
            current_waypoint = idx + 1
            self.get_logger().info(f"开始执行路点 {idx}/{total_waypoints-1}")

            # 计算延迟
            if self.delay is not None:
                exec_delay = float(self.delay)
            else:
                delay_map = {0: 3.0, 1: 2.0, 2: 1.0}
                exec_delay = delay_map.get(wp['speed_level'], 3.0)

            # 根据模式决定结果
            if self.mode == "fail":
                if idx >= total_waypoints // 2:  # 在中间路点失败
                    self._simulate_with_progress(task_id, exec_delay * 0.5, RESULT_FAIL, current_waypoint, total_waypoints)
                    return
                else:
                    self._simulate_with_progress(task_id, exec_delay, None, current_waypoint, total_waypoints, report_progress=True)

            elif self.mode == "abort":
                if idx >= total_waypoints // 3:  # 在前1/3处中止
                    self._simulate_with_progress(task_id, exec_delay * 0.3, RESULT_ABORT, current_waypoint, total_waypoints)
                    return
                else:
                    self._simulate_with_progress(task_id, exec_delay, None, current_waypoint, total_waypoints, report_progress=True)

            elif self.mode == "random":
                if random.random() < 0.2:  # 20%概率失败
                    result_code = random.choice([RESULT_FAIL, RESULT_ABORT, RESULT_REJECT])
                    self._simulate_with_progress(task_id, exec_delay * 0.5, result_code, current_waypoint, total_waypoints)
                    return
                else:
                    self._simulate_with_progress(task_id, exec_delay, None, current_waypoint, total_waypoints, report_progress=True)

            else:  # success or progress mode
                report_progress = (self.mode == "progress")
                # 最后一个路点返回最终结果，其他路点只报进度
                if idx == total_waypoints - 1:
                    self._simulate_with_progress(task_id, exec_delay, RESULT_SUCCESS, current_waypoint, total_waypoints, report_progress)
                else:
                    self._simulate_with_progress(task_id, exec_delay, None, current_waypoint, total_waypoints, report_progress)

    def _simulate_with_progress(self, task_id, total_delay, final_result, current_wp, total_wp, report_progress=False):
        """模拟路点执行（带进度上报）

        Args:
            task_id: 任务ID
            total_delay: 总延迟时间
            final_result: 最终结果码（None表示仅报告进度）
            current_wp: 当前路点编号（1-based）
            total_wp: 总路点数
            report_progress: 是否上报子进度
        """
        if report_progress and total_delay > 0.5:
            steps = [25.0, 50.0, 75.0]
            step_delay = total_delay / 4.0
            for progress in steps:
                time.sleep(step_delay)
                # 上报当前路点的子进度
                self.get_logger().info(f"路点 {current_wp-1}/{total_wp-1} 进度: {progress:.0f}%")
                # 这里可以选择性上报子进度到话题
            time.sleep(step_delay)
        else:
            time.sleep(total_delay)

        # 上报路点完成或最终结果
        if final_result is not None:
            result_name = {
                RESULT_SUCCESS: "成功",
                RESULT_FAIL: "失败",
                RESULT_ABORT: "中止",
                RESULT_REJECT: "拒绝",
            }.get(final_result, f"未知({final_result})")
            self.get_logger().info(f"任务 {task_id:.0f} 执行结果: {result_name} ({final_result:.0f})")
            self._publish_result(task_id, final_result)
        else:
            # 仅报告路点完成进度
            waypoint_progress = (current_wp / total_wp) * 100.0
            self.get_logger().info(f"路点 {current_wp-1}/{total_wp-1} 完成，总进度: {waypoint_progress:.1f}%")
            self._publish_result(task_id, waypoint_progress)

    def _publish_result(self, task_id, result_code):
        """发布执行结果"""
        msg = Float32MultiArray()
        msg.data = [float(task_id), float(result_code)]
        self.result_publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="四自由度头颈运控多路点组合电机控制模拟节点")
    parser.add_argument(
        "--mode", type=str, default="success",
        choices=["success", "fail", "abort", "reject", "random", "progress"],
        help="模拟模式",
    )
    parser.add_argument(
        "--delay", type=float, default=None,
        help="每个路点的模拟执行延迟（秒），默认根据速度档位自动计算",
    )
    args = parser.parse_args()

    rclpy.init()
    node = MockFourWaypointNode(mode=args.mode, delay=args.delay)

    try:
        print("\n多路点模拟电机节点运行中，等待指令... (Ctrl+C 退出)\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n正在关闭多路点模拟电机节点...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

