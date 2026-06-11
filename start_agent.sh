#!/bin/bash

# 1. 设置设备权限
# echo "正在设置设备权限..."
# echo "sunrise" | sudo -S chmod 777 /dev/ttyACM0

# # 检查权限设置是否成功
# if [ $? -ne 0 ]; then
#     echo "错误：设置设备权限失败"
#     exit 1
# fi

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# 2. 配置日志目录
data_name=$(date +"%Y%m%d_%H%M%S")
agent_log_dir="/userdata/roslog/agent"

if [ ! -d "${agent_log_dir}" ]; then
    mkdir -p "${agent_log_dir}"
fi

# 3. 配置 ROS2 环境。S100 使用 Humble；本地 overlay 只加载 local_setup，
# 避免 colcon 生成的 setup.bash 继续链式加载旧的 /opt/ros/foxy。
source /opt/ros/humble/setup.bash
if [ -f "install/local_setup.bash" ]; then
    source install/local_setup.bash
else
    echo "警告：未找到 install/local_setup.bash，仅使用系统 ROS Humble 环境" >&2
fi

# 4. 启动代理并记录日志（后台运行）
echo "正在启动机器人代理程序，日志: ${agent_log_dir}/agent_${data_name}.log"
nohup python3 smart_robot_agent.py >>"${agent_log_dir}/agent_${data_name}.log" 2>&1 &
echo "Agent 已启动，进程 ID: $!"
