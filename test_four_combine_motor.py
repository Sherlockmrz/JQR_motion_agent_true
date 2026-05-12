#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四自由度头颈运控组合电机控制端到端测试脚本（上游业务模拟）

模拟上游业务节点：直接发布指令到 /four_combine_motor_control（12字段），
并订阅 /four_combine_motor_control_result 收集下游（mock_four_motor_node）反馈。

字段顺序（与协议一致）：
    data[0]  task_id
    data[1]  control_yaw         data[2]  yaw_angle    (rad)
    data[3]  control_roll        data[4]  roll_angle   (rad)
    data[5]  control_pitch       data[6]  pitch_angle  (rad)
    data[7]  control_chassis_move data[8] chassis_offset (m)
    data[9]  control_chassis_rotate data[10] chassis_rotation (rad)
    data[11] speed_level

用法：
    # 1. 先启动下游 mock 节点:
    #    python3 mock_four_motor_node.py --mode progress
    # 2. 再运行本脚本:
    python3 test_four_combine_motor.py                      # 跑全部用例
    python3 test_four_combine_motor.py --case yaw           # 仅 yaw
    python3 test_four_combine_motor.py --case roll          # 仅 roll
    python3 test_four_combine_motor.py --case pitch         # 仅 pitch
    python3 test_four_combine_motor.py --case neck3         # 三轴同时
    python3 test_four_combine_motor.py --case chassis       # 仅底盘
    python3 test_four_combine_motor.py --case full          # 四自由度全开
    python3 test_four_combine_motor.py --timeout 10         # 自定义超时
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

FIELD_COUNT = 12


class UpstreamTestClient(Node):
    """模拟上游业务：发任务 + 收反馈"""

    def __init__(self):
        super().__init__('upstream_four_motor_tester')

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/four_combine_motor_control',
            10,
        )

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/four_combine_motor_control_result',
            self._on_result,
            10,
        )

        self._results: Dict[int, Dict[str, Any]] = {}
        self._lock = threading.Lock()
        self._task_counter = 0

        self.get_logger().info("上游测试客户端已启动")
        self.get_logger().info("发布: /four_combine_motor_control")
        self.get_logger().info("订阅: /four_combine_motor_control_result")

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

    def send(self,
             task_id: int,
             control_yaw: bool = False, yaw_angle: float = 0.0,
             control_roll: bool = False, roll_angle: float = 0.0,
             control_pitch: bool = False, pitch_angle: float = 0.0,
             control_chassis_move: bool = False, chassis_offset: float = 0.0,
             control_chassis_rotate: bool = False, chassis_rotation: float = 0.0,
             speed_level: int = 0) -> None:
        msg = Float32MultiArray()
        msg.data = [
            float(task_id),
            1.0 if control_yaw else 0.0,
            float(yaw_angle),
            1.0 if control_roll else 0.0,
            float(roll_angle),
            1.0 if control_pitch else 0.0,
            float(pitch_angle),
            1.0 if control_chassis_move else 0.0,
            float(chassis_offset),
            1.0 if control_chassis_rotate else 0.0,
            float(chassis_rotation),
            float(speed_level),
        ]
        assert len(msg.data) == FIELD_COUNT, f"字段数应为{FIELD_COUNT}, 实际{len(msg.data)}"
        self.publisher.publish(msg)
        self.get_logger().info(f"[SEND]   task={task_id} data={[round(x, 4) for x in msg.data]}")

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

def case_yaw_only(client: UpstreamTestClient, timeout: float) -> Dict[str, Any]:
    """仅控制偏航 yaw"""
    task_id = client.next_task_id()
    client.send(
        task_id,
        control_yaw=True, yaw_angle=math.radians(30),
        speed_level=2,
    )
    return _collect(client, task_id, "yaw_only", timeout)


def case_roll_only(client: UpstreamTestClient, timeout: float) -> Dict[str, Any]:
    """仅控制翻滚 roll"""
    task_id = client.next_task_id()
    client.send(
        task_id,
        control_roll=True, roll_angle=math.radians(15),
        speed_level=1,
    )
    return _collect(client, task_id, "roll_only", timeout)


def case_pitch_only(client: UpstreamTestClient, timeout: float) -> Dict[str, Any]:
    """仅控制俯仰 pitch"""
    task_id = client.next_task_id()
    client.send(
        task_id,
        control_pitch=True, pitch_angle=math.radians(-20),
        speed_level=2,
    )
    return _collect(client, task_id, "pitch_only", timeout)


def case_neck_three_axis(client: UpstreamTestClient, timeout: float) -> Dict[str, Any]:
    """三轴同时：yaw + roll + pitch"""
    task_id = client.next_task_id()
    client.send(
        task_id,
        control_yaw=True, yaw_angle=math.radians(30),
        control_roll=True, roll_angle=math.radians(5),
        control_pitch=True, pitch_angle=math.radians(-10),
        speed_level=1,
    )
    return _collect(client, task_id, "neck_three_axis", timeout)


def case_chassis_only(client: UpstreamTestClient, timeout: float) -> Dict[str, Any]:
    """仅控制底盘：前进 0.3m + 逆时针旋转 45°"""
    task_id = client.next_task_id()
    client.send(
        task_id,
        control_chassis_move=True, chassis_offset=0.3,
        control_chassis_rotate=True, chassis_rotation=math.radians(45),
        speed_level=1,
    )
    return _collect(client, task_id, "chassis_only", timeout)


def case_full_combo(client: UpstreamTestClient, timeout: float) -> Dict[str, Any]:
    """四自由度全开：yaw + roll + pitch + 底盘位移 + 底盘旋转"""
    task_id = client.next_task_id()
    client.send(
        task_id,
        control_yaw=True, yaw_angle=math.radians(20),
        control_roll=True, roll_angle=math.radians(3),
        control_pitch=True, pitch_angle=math.radians(-8),
        control_chassis_move=True, chassis_offset=0.2,
        control_chassis_rotate=True, chassis_rotation=math.radians(30),
        speed_level=1,
    )
    return _collect(client, task_id, "full_combo", timeout)


def case_negative_chassis(client: UpstreamTestClient, timeout: float) -> Dict[str, Any]:
    """负值测试：后退 + 顺时针旋转"""
    task_id = client.next_task_id()
    client.send(
        task_id,
        control_chassis_move=True, chassis_offset=-0.25,
        control_chassis_rotate=True, chassis_rotation=math.radians(-60),
        speed_level=0,
    )
    return _collect(client, task_id, "negative_chassis", timeout)


def case_invalid_speed(client: UpstreamTestClient, timeout: float) -> Dict[str, Any]:
    """非法档位（speed_level=7），下游应按 0 处理"""
    task_id = client.next_task_id()
    client.send(
        task_id,
        control_yaw=True, yaw_angle=math.radians(10),
        speed_level=7,
    )
    return _collect(client, task_id, "invalid_speed", timeout)


def _collect(client: UpstreamTestClient, task_id: int, name: str, timeout: float) -> Dict[str, Any]:
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
    "yaw": case_yaw_only,
    "roll": case_roll_only,
    "pitch": case_pitch_only,
    "neck3": case_neck_three_axis,
    "chassis": case_chassis_only,
    "full": case_full_combo,
    "negative": case_negative_chassis,
    "invalid_speed": case_invalid_speed,
}


def run_selected(client: UpstreamTestClient, names: List[str], timeout: float) -> List[Dict[str, Any]]:
    results = []
    for name in names:
        fn = CASES[name]
        client.get_logger().info("-" * 60)
        client.get_logger().info(f"运行用例: {name}")
        res = fn(client, timeout=timeout)
        results.append(res)
        time.sleep(0.2)
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
        print(f"  [{status}] {r['case']:16s} task={r['task_id']:<12d} result={rc:8s} {prog_str}")
    ok = sum(1 for r in results if r["success"])
    print("-" * 68)
    print(f"通过: {ok}/{len(results)}")
    print("=" * 68)


def main():
    parser = argparse.ArgumentParser(description="四自由度头颈运控组合电机端到端测试（上游模拟）")
    parser.add_argument(
        "--case", type=str, default="all",
        choices=list(CASES.keys()) + ["all"],
        help="运行指定用例，默认 all",
    )
    parser.add_argument("--timeout", type=float, default=15.0, help="每个用例等待反馈超时（秒）")
    parser.add_argument("--warmup", type=float, default=0.5,
                        help="发任务前等待时间（秒），给订阅握手")
    args = parser.parse_args()

    rclpy.init()
    client = UpstreamTestClient()

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
