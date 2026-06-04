#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四自由度头颈运控多路点组合电机控制端到端测试脚本

模拟上游业务节点：直接发布指令到 /four_combine_waypoint_control，
并订阅 /four_combine_waypoint_control_result 收集下游反馈。

用法：
    # 1. 先启动下游 mock 节点:
    #    python3 mock_four_waypoint_node.py --mode progress
    # 2. 再运行本脚本:
    python3 test_four_waypoint_control.py                    # 跑全部用例
    python3 test_four_waypoint_control.py --case single      # 单路点
    python3 test_four_waypoint_control.py --case multi       # 3路点
    python3 test_four_waypoint_control.py --case complex     # 复杂5路点
    python3 test_four_waypoint_control.py --timeout 30       # 自定义超时
"""
import argparse
import math
import threading
import time
from typing import Dict, Any, List, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
except ImportError:
    print("[ERROR] ROS2 (rclpy) 不可用，请先 source ROS2 环境")
    exit(1)


RESULT_SUCCESS = 101
RESULT_ABORT = 102
RESULT_FAIL = 103
RESULT_REJECT = 104

RESULT_NAMES = {
    RESULT_SUCCESS: "SUCCESS",
    RESULT_ABORT: "ABORTED",
    RESULT_FAIL: "FAILED",
    RESULT_REJECT: "REJECTED",
}

HEADER_SIZE = 3
WAYPOINT_SIZE = 12


class UpstreamWaypointTestClient(Node):
    """模拟上游业务：发任务 + 收反馈"""

    def __init__(self):
        super().__init__('upstream_waypoint_tester')

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/four_combine_waypoint_control',
            10,
        )

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/four_combine_waypoint_control_result',
            self._on_result,
            10,
        )

        self._results: Dict[int, Dict[str, Any]] = {}
        self._lock = threading.Lock()
        self._task_counter = 0

        self.get_logger().info("上游多路点测试客户端已启动")
        self.get_logger().info("发布: /four_combine_waypoint_control")
        self.get_logger().info("订阅: /four_combine_waypoint_control_result")

    def _on_result(self, msg):
        data = msg.data
        if len(data) < 2:
            self.get_logger().warn(f"反馈字段不足: {len(data)}")
            return

        task_id = int(data[0])
        result = float(data[1])

        with self._lock:
            entry = self._results.setdefault(task_id, {"progress": [], "final": None, "final_ts": None})
            is_final = int(result) in (RESULT_SUCCESS, RESULT_ABORT, RESULT_FAIL, RESULT_REJECT)
            if is_final:
                entry["final"] = int(result)
                entry["final_ts"] = time.time()
                self.get_logger().info(
                    f"[REPORT] task={task_id} final={RESULT_NAMES.get(int(result), result)}"
                )
            else:
                entry["progress"].append(result)
                self.get_logger().info(f"[REPORT] task={task_id} progress={result:.0f}%")

    def next_task_id(self) -> int:
        self._task_counter += 1
        base = int(time.time()) % 100000
        return base * 100 + self._task_counter

    def send_waypoints(self, task_id: int, pose_mode: int, waypoints: List[Dict[str, Any]]) -> None:
        """发送多路点控制指令

        Args:
            task_id: 任务ID
            pose_mode: 0=相对位姿 1=绝对位姿
            waypoints: 路点列表，每个路点包含12个字段
        """
        msg = Float32MultiArray()

        # 构建数据数组：头部(3) + N个路点(每个12字段)
        data = [
            float(task_id),
            float(pose_mode),
            float(len(waypoints))
        ]

        # 添加每个路点的数据
        for wp in waypoints:
            data.extend([
                1.0 if wp.get('control_yaw', False) else 0.0,
                float(wp.get('yaw_angle', 0.0)),
                1.0 if wp.get('control_roll', False) else 0.0,
                float(wp.get('roll_angle', 0.0)),
                1.0 if wp.get('control_pitch', False) else 0.0,
                float(wp.get('pitch_angle', 0.0)),
                1.0 if wp.get('control_chassis_move', False) else 0.0,
                float(wp.get('chassis_offset', 0.0)),
                1.0 if wp.get('control_chassis_rotate', False) else 0.0,
                float(wp.get('chassis_rotation', 0.0)),
                float(wp.get('speed_level', 0)),
                float(wp.get('timeout', 0.0)),
            ])

        msg.data = data
        expected_len = HEADER_SIZE + len(waypoints) * WAYPOINT_SIZE
        assert len(msg.data) == expected_len, f"字段数应为{expected_len}, 实际{len(msg.data)}"

        self.publisher.publish(msg)
        self.get_logger().info(f"[SEND] task={task_id} pose_mode={pose_mode} waypoints={len(waypoints)}")

    def wait_for_final(self, task_id: int, timeout: float) -> Optional[Dict[str, Any]]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                entry = self._results.get(task_id)
                if entry and entry["final"] is not None:
                    return entry
            time.sleep(0.05)
        with self._lock:
            return self._results.get(task_id)


# ----- 测试用例 -----

def case_single_waypoint(client: UpstreamWaypointTestClient, timeout: float) -> Dict[str, Any]:
    """单路点测试：仅控制 yaw"""
    task_id = client.next_task_id()
    waypoints = [
        {
            'control_yaw': True,
            'yaw_angle': math.radians(30),
            'speed_level': 2,
            'timeout': 10.0
        }
    ]
    client.send_waypoints(task_id, pose_mode=0, waypoints=waypoints)
    return _collect(client, task_id, "single_waypoint", timeout)


def case_multi_waypoint(client: UpstreamWaypointTestClient, timeout: float) -> Dict[str, Any]:
    """多路点测试：3个路点的组合动作"""
    task_id = client.next_task_id()
    waypoints = [
        # 路点0: 偏航30°
        {
            'control_yaw': True,
            'yaw_angle': math.radians(30),
            'speed_level': 1,
            'timeout': 5.0
        },
        # 路点1: roll 15° + pitch -20°
        {
            'control_roll': True,
            'roll_angle': math.radians(15),
            'control_pitch': True,
            'pitch_angle': math.radians(-20),
            'speed_level': 1,
            'timeout': 5.0
        },
        # 路点2: 底盘前进 + 旋转
        {
            'control_chassis_move': True,
            'chassis_offset': 0.3,
            'control_chassis_rotate': True,
            'chassis_rotation': math.radians(45),
            'speed_level': 2,
            'timeout': 8.0
        }
    ]
    client.send_waypoints(task_id, pose_mode=0, waypoints=waypoints)
    return _collect(client, task_id, "multi_waypoint", timeout)


def case_complex_sequence(client: UpstreamWaypointTestClient, timeout: float) -> Dict[str, Any]:
    """复杂序列测试：5个路点的完整动作序列"""
    task_id = client.next_task_id()
    waypoints = [
        # 路点0: 头部三轴同时
        {
            'control_yaw': True,
            'yaw_angle': math.radians(20),
            'control_roll': True,
            'roll_angle': math.radians(5),
            'control_pitch': True,
            'pitch_angle': math.radians(-10),
            'speed_level': 1,
            'timeout': 5.0
        },
        # 路点1: 底盘前进
        {
            'control_chassis_move': True,
            'chassis_offset': 0.2,
            'speed_level': 1,
            'timeout': 3.0
        },
        # 路点2: 底盘旋转
        {
            'control_chassis_rotate': True,
            'chassis_rotation': math.radians(90),
            'speed_level': 1,
            'timeout': 4.0
        },
        # 路点3: 头部回正
        {
            'control_yaw': True,
            'yaw_angle': 0.0,
            'control_roll': True,
            'roll_angle': 0.0,
            'control_pitch': True,
            'pitch_angle': 0.0,
            'speed_level': 2,
            'timeout': 5.0
        },
        # 路点4: 底盘后退
        {
            'control_chassis_move': True,
            'chassis_offset': -0.15,
            'speed_level': 0,
            'timeout': 3.0
        }
    ]
    client.send_waypoints(task_id, pose_mode=0, waypoints=waypoints)
    return _collect(client, task_id, "complex_sequence", timeout)


def case_absolute_pose(client: UpstreamWaypointTestClient, timeout: float) -> Dict[str, Any]:
    """绝对位姿模式测试"""
    task_id = client.next_task_id()
    waypoints = [
        {
            'control_yaw': True,
            'yaw_angle': math.radians(45),
            'control_pitch': True,
            'pitch_angle': math.radians(-15),
            'speed_level': 1,
            'timeout': 5.0
        },
        {
            'control_yaw': True,
            'yaw_angle': math.radians(90),
            'control_pitch': True,
            'pitch_angle': math.radians(-30),
            'speed_level': 1,
            'timeout': 5.0
        }
    ]
    client.send_waypoints(task_id, pose_mode=1, waypoints=waypoints)  # pose_mode=1 绝对位姿
    return _collect(client, task_id, "absolute_pose", timeout)


def _collect(client: UpstreamWaypointTestClient, task_id: int, name: str, timeout: float) -> Dict[str, Any]:
    entry = client.wait_for_final(task_id, timeout=timeout)
    if entry is None or entry.get("final") is None:
        return {"case": name, "task_id": task_id, "success": False, "error": "TIMEOUT",
                "progress": entry["progress"] if entry else []}
    final = entry["final"]
    return {
        "case": name,
        "task_id": task_id,
        "success": final == RESULT_SUCCESS,
        "result_code": final,
        "result_name": RESULT_NAMES.get(final, str(final)),
        "progress": entry["progress"],
    }


CASES = {
    "single": case_single_waypoint,
    "multi": case_multi_waypoint,
    "complex": case_complex_sequence,
    "absolute": case_absolute_pose,
}


def run_selected(client: UpstreamWaypointTestClient, names: List[str], timeout: float) -> List[Dict[str, Any]]:
    results = []
    for name in names:
        fn = CASES[name]
        client.get_logger().info("-" * 60)
        client.get_logger().info(f"运行用例: {name}")
        res = fn(client, timeout=timeout)
        results.append(res)
        time.sleep(0.3)
    return results


def print_summary(results: List[Dict[str, Any]]):
    print("\n" + "=" * 68)
    print("测试结果汇总")
    print("=" * 68)
    for r in results:
        status = "OK  " if r["success"] else "FAIL"
        rc = r.get("result_name") or r.get("error") or "-"
        prog = r.get("progress") or []
        prog_str = f"progress={prog}" if prog else "no_progress"
        print(f"  [{status}] {r['case']:20s} task={r['task_id']:<12d} result={rc:8s} {prog_str}")
    ok = sum(1 for r in results if r["success"])
    print("-" * 68)
    print(f"通过: {ok}/{len(results)}")
    print("=" * 68)


def main():
    parser = argparse.ArgumentParser(description="四自由度头颈运控多路点组合电机端到端测试")
    parser.add_argument(
        "--case", type=str, default="all",
        choices=list(CASES.keys()) + ["all"],
        help="运行指定用例，默认 all",
    )
    parser.add_argument("--timeout", type=float, default=30.0, help="每个用例等待反馈超时（秒）")
    parser.add_argument("--warmup", type=float, default=0.5,
                        help="发任务前等待时间（秒），给订阅握手")
    args = parser.parse_args()

    rclpy.init()
    client = UpstreamWaypointTestClient()

    spin_thread = threading.Thread(target=lambda: rclpy.spin(client), daemon=True)
    spin_thread.start()

    try:
        time.sleep(args.warmup)
        names = list(CASES.keys()) if args.case == "all" else [args.case]
        results = run_selected(client, names, timeout=args.timeout)
        print_summary(results)
        exit_code = 0 if all(r["success"] for r in results) else 1
    finally:
        client.destroy_node()
        rclpy.shutdown()

    exit(exit_code)


if __name__ == "__main__":
    main()

