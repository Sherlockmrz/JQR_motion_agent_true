#!/bin/bash
# 全流程E2E测试启动脚本
# 用法: bash run_e2e_test.sh

set -e
cd "$(dirname "$0")"

# Source ROS2 环境
echo "[1/4] Source ROS2 环境..."
source ../install/setup.bash

# 启动 mock 电机节点（后台）
echo "[2/4] 启动 mock_motor_node (progress模式, delay=0.5s)..."
python3 mock_motor_node.py --mode progress --delay 0.5 &
MOCK_PID=$!
sleep 2

# 运行测试
echo "[3/4] 启动 Agent 并运行测试..."
USB_SERIAL_ENABLED=false python3 test_e2e_scenarios.py
TEST_EXIT=$?

# 清理
echo "[4/4] 清理..."
kill $MOCK_PID 2>/dev/null
wait $MOCK_PID 2>/dev/null

if [ $TEST_EXIT -eq 0 ]; then
    echo "全部测试通过"
else
    echo "存在失败的测试"
fi
exit $TEST_EXIT
