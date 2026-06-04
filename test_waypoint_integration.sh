#!/bin/bash
# 四联多路点组合电机控制集成测试脚本
#
# 功能：自动启动 mock 节点 -> 运行测试 -> 检查结果 -> 清理
#
# 用法：
#   ./test_waypoint_integration.sh                  # 全部测试用例
#   ./test_waypoint_integration.sh --case single    # 单个用例

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}四联多路点组合电机控制集成测试${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查 ROS2 环境
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}[ERROR] ros2 命令不可用，请先 source ROS2 环境${NC}"
    exit 1
fi

# 启动 mock 节点
echo -e "${YELLOW}[1/4] 启动 mock 节点 (progress 模式)...${NC}"
python3 mock_four_waypoint_node.py --mode progress &
MOCK_PID=$!
sleep 2

# 检查 mock 节点是否存活
if ! kill -0 $MOCK_PID 2>/dev/null; then
    echo -e "${RED}[ERROR] mock 节点启动失败${NC}"
    exit 1
fi
echo -e "${GREEN}✓ mock 节点已启动 (PID: $MOCK_PID)${NC}"

# 清理函数
cleanup() {
    echo -e "${YELLOW}[4/4] 清理环境...${NC}"
    if kill -0 $MOCK_PID 2>/dev/null; then
        kill $MOCK_PID
        wait $MOCK_PID 2>/dev/null || true
    fi
    echo -e "${GREEN}✓ 清理完成${NC}"
}
trap cleanup EXIT

# 运行测试
echo -e "${YELLOW}[2/4] 运行测试用例...${NC}"
TEST_ARGS="${@}"
if [ -z "$TEST_ARGS" ]; then
    TEST_ARGS="--case all"
fi

python3 test_four_waypoint_control.py $TEST_ARGS
TEST_EXIT_CODE=$?

# 检查结果
echo -e "${YELLOW}[3/4] 检查测试结果...${NC}"
if [ $TEST_EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}✓ 全部测试通过${NC}"
    echo -e "${GREEN}========================================${NC}"
    exit 0
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}✗ 测试失败 (exit code: $TEST_EXIT_CODE)${NC}"
    echo -e "${RED}========================================${NC}"
    exit 1
fi
