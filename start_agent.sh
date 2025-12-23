#!/bin/bash
# 脚本名称：start_robot.sh
# 功能：设置设备权限，配置ROS2环境，并启动机器人代理程序

# 1. 设置设备权限
echo "正在设置设备权限..."
echo "sunrise" | sudo -S chmod 777 /dev/ttyACM0

# 检查上一步是否成功
if [ $? -ne 0 ]; then
    echo "错误：设置设备权限失败，请检查设备是否存在或密码是否正确"
    exit 1
fi

# 2. 配置ROS2环境
echo "正在配置ROS2环境..."
source /opt/ros/humble/setup.bash

# 3. 配置工作空间
echo "正在设置工作空间..."
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "错误：找不到install/setup.bash文件，请确保在正确的目录中运行此脚本"
    exit 1
fi

# 4. 启动机器人代理
echo "正在启动机器人代理程序..."
if [ -f "smart_robot_agent.py" ]; then
    python3 smart_robot_agent.py
else
    echo "错误：找不到smart_robot_agent.py文件"
    exit 1
fi

echo "程序执行完成"