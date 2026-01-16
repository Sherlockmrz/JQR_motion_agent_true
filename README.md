# Jrobot_agent

#### 介绍

{以下是 Gitee 平台说明，您可以替换此简介 Gitee 是 OSCHINA 推出的基于 Git 的代码托管平台。专为开发者提供稳定、高效、安全的云端软件开发协作平台 无论是个人、团队、或是企业，都能够用 Gitee 实现代码托管、项目管理、协作开发。}

#### 软件版本

v1.0.0
演示版本agent


#### 安装教程

将代码部署至s100上的某一位置（如/home/sunrise/motion_agent，s100 用户名  密码均为sunrise） 
前置项：将rk.rules文件 ，放置在s100 的 /etc/udev/rules.d/ 目录下
在代码目录终端，运行下列指令:
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo apt install ros-humble-nav2-common
source /opt/ros/humble/setup.bash
colcon build
sudo chmod +x start_agent.sh


#### 使用说明

./start_agent.sh

