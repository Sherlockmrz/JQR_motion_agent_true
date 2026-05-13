#!/bin/bash
# 4自由度头颈控制脚本 - 快速测试脚本

echo "=================================="
echo "4自由度头颈控制脚本 - 快速测试"
echo "=================================="
echo ""

# 1. 检查文件是否存在
echo "1. 检查文件..."
files=(
    "test_4dof_head_control.py"
    "verify_4dof_test.py"
    "README_4DOF_TEST.md"
    "TEST_SUMMARY_4DOF.md"
)

for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✓ $file 存在"
    else
        echo "  ✗ $file 不存在"
        exit 1
    fi
done
echo ""

# 2. 语法检查
echo "2. Python语法检查..."
python3 -m py_compile test_4dof_head_control.py && echo "  ✓ test_4dof_head_control.py 语法正确" || exit 1
python3 -m py_compile verify_4dof_test.py && echo "  ✓ verify_4dof_test.py 语法正确" || exit 1
echo ""

# 3. 导入测试
echo "3. 模块导入测试..."
python3 -c "import test_4dof_head_control; print(f'  ✓ 脚本可导入\n  ✓ 场景数量: {len(test_4dof_head_control.SCENARIOS)}')" || exit 1
echo ""

# 4. 运行验证测试
echo "4. 运行验证测试..."
python3 verify_4dof_test.py || exit 1
echo ""

# 5. 显示文件信息
echo "5. 文件信息..."
ls -lh test_4dof_head_control.py verify_4dof_test.py README_4DOF_TEST.md TEST_SUMMARY_4DOF.md
echo ""

echo "=================================="
echo "✓ 所有测试通过！"
echo "=================================="
echo ""
echo "使用方法："
echo "  1. 启动 SmartRobotAgent:"
echo "     python3 Jrobot_agent/smart_robot_agent.py"
echo ""
echo "  2. 运行测试脚本:"
echo "     python3 test_4dof_head_control.py"
echo ""
echo "  3. 选择场景进行测试"
echo ""
