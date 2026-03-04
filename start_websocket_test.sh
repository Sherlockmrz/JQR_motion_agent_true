#!/bin/bash
# WebSocket控制接口快速测试脚本

echo "======================================"
echo "智能机器人WebSocket控制接口测试"
echo "======================================"
echo ""

# 检查Python是否安装
if ! command -v python3 &> /dev/null
then
    echo "错误: Python3 未安装"
    exit 1
fi

# 检查必要的Python包
echo "检查依赖包..."
python3 -c "import websockets" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "正在安装 websockets 包..."
    pip3 install websockets
fi

echo ""
echo "请选择测试方式:"
echo "1) 运行Python测试脚本 (自动测试)"
echo "2) 运行Python测试脚本 (交互式测试)"
echo "3) 打开HTML控制面板 (需要图形界面)"
echo "4) 查看使用说明"
echo ""

read -p "请输入选项 (1-4): " choice

case $choice in
    1)
        echo ""
        echo "启动自动测试..."
        echo "确保SmartRobotAgent已在运行!"
        echo ""
        python3 test_websocket_control.py <<< "1"
        ;;
    2)
        echo ""
        echo "启动交互式测试..."
        echo "确保SmartRobotAgent已在运行!"
        echo ""
        python3 test_websocket_control.py <<< "2"
        ;;
    3)
        echo ""
        echo "打开HTML控制面板..."
        HTML_FILE="websocket_control_client.html"
        if [ -f "$HTML_FILE" ]; then
            # 尝试使用默认浏览器打开
            if command -v xdg-open &> /dev/null; then
                xdg-open "$HTML_FILE"
            elif command -v open &> /dev/null; then
                open "$HTML_FILE"
            else
                echo "请在浏览器中手动打开文件: $HTML_FILE"
            fi
        else
            echo "错误: 找不到文件 $HTML_FILE"
        fi
        ;;
    4)
        echo ""
        cat WEBSOCKET_CONTROL_README.md | less
        ;;
    *)
        echo "无效选项"
        exit 1
        ;;
esac

echo ""
echo "测试完成"
