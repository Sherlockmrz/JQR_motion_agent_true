#!/bin/bash

# 1. 设置设备权限
# echo "正在设置设备权限..."
# echo "sunrise" | sudo -S chmod 777 /dev/ttyACM0

# # 检查权限设置是否成功
# if [ $? -ne 0 ]; then
#     echo "错误：设置设备权限失败"
#     exit 1
# fi

# 2. 配置日志目录
data_name=`date +"%Y%m%d_%H%M%S"`
agent_log_dir="/userdata/roslog/agent"

if [ ! -d "${agent_log_dir}" ]; then
    mkdir -p ${agent_log_dir}
fi

# 3. 配置ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 4. 启动代理并记录日志（后台运行）
echo "正在启动机器人代理程序，日志: ${agent_log_dir}/agent_${data_name}.log"
nohup python3 smart_robot_agent.py >>${agent_log_dir}/agent_${data_name}.log 2>&1 &
echo "Agent 已启动，进程 ID: $!"
