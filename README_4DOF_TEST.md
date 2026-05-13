# 4自由度头颈控制测试脚本使用说明

## 概述

`test_4dof_head_control.py` 是一个适配agent中新的4自由度任务控制接口的测试脚本。它将 `test.py` 中的演示动作转换为 roll/pitch/yaw 三轴控制（移除底盘控制部分）。

## 坐标系说明

使用 ROS2 标准坐标系（头颈部正前方向）：

- **yaw（偏航角）**: 左右转头
  - 正值 = 左转
  - 负值 = 右转
  
- **pitch（俯仰角）**: 上下点头
  - 正值 = 低头
  - 负值 = 抬头
  
- **roll（翻滚角）**: 左右歪头
  - 正值 = 向左歪
  - 负值 = 向右歪

## 与原test.py的主要区别

1. **移除底盘控制**: 所有场景只控制头颈部，不包含底盘移动和旋转
2. **4自由度接口**: 使用新的 `set_four_combine_motor_control` 接口
3. **三轴控制**: 支持 yaw/pitch/roll 三个自由度的独立控制
4. **ROS2坐标系**: 严格遵循ROS2坐标系定义

## 测试场景

### 场景1: 用户移动位置时的视线跟踪
- 头部: pitch=30°(低头), yaw=30°(左转), roll=0°
- 适用场景: 用户从正前方走到侧面

### 场景2: 头部电机回归0位
- 头部: yaw=0°, pitch=0°, roll=0°
- 适用场景: 复位头部到初始位置

### 场景3: 左右摆头观察
- 动作序列: 左转80° → 右转80° → 回中0°
- 适用场景: 巡逻观察

### 场景10: 4自由度头颈手动控制
- 交互式输入 yaw/pitch/roll 角度
- 支持自定义任意角度组合

## 使用方法

### 1. 启动前提条件

确保以下服务已启动：
```bash
# 启动 SmartRobotAgent (WebSocket端口 8766)
python3 Jrobot_agent/smart_robot_agent.py
```

### 2. 运行测试脚本

```bash
cd /home/jungong3/vln/as/git_jqr_agent
python3 test_4dof_head_control.py
```

### 3. 选择测试场景

脚本会显示菜单，可以选择：
- 输入场景编号（1-10）运行单个场景
- 输入 0 运行所有场景
- 输入 q 退出

### 4. 手动控制示例

选择场景10后，可以手动输入角度：
```
yaw偏航角(度, 正=左转, 负=右转, 0=不控制): 45
pitch俯仰角(度, 正=低头, 负=抬头, 0=不控制): 20
roll翻滚角(度, 正=左歪, 负=右歪, 0=不控制): 0
速度档位(0=低速, 1=中速, 2=快速) [默认1]: 1
```

## 验证测试

### 测试1: 语法检查
```bash
python3 -m py_compile test_4dof_head_control.py
```

### 测试2: 基本功能测试
```bash
# 测试场景2（头部归零）
python3 test_4dof_head_control.py
# 选择: 2
```

### 测试3: 手动控制测试
```bash
# 测试场景10（手动控制）
python3 test_4dof_head_control.py
# 选择: 10
# 输入: yaw=30, pitch=15, roll=0, speed=1
```

## 接口说明

### 4自由度控制接口

```python
{
    "type": "set_four_combine_motor_control",
    "params": {
        "control_yaw": True,      # 是否控制偏航
        "yaw_angle": 0.524,       # 偏航角度（弧度）
        "control_pitch": True,    # 是否控制俯仰
        "pitch_angle": 0.524,     # 俯仰角度（弧度）
        "control_roll": False,    # 是否控制翻滚
        "roll_angle": 0.0,        # 翻滚角度（弧度）
        "speed_level": 1          # 速度档位 (0=低速, 1=中速, 2=快速)
    }
}
```

### 角度转换

脚本自动将度数转换为弧度：
```python
yaw_rad = math.radians(yaw_deg)
pitch_rad = math.radians(pitch_deg)
roll_rad = math.radians(roll_deg)
```

## 注意事项

1. **角度限制**: 请注意头部电机的物理限位
   - pitch: -30° ~ 30°
   - yaw: -110° ~ 110°
   - roll: 根据实际硬件限制

2. **速度档位**: 
   - 0 = 低速（约30°/s）
   - 1 = 中速（约60°/s）
   - 2 = 快速（约90°/s）

3. **底盘控制**: 如需底盘控制，请使用原 `test.py` 脚本

4. **WebSocket连接**: 确保agent的WebSocket服务器在 `localhost:8766` 运行

## 故障排查

### 问题1: 无法连接到WebSocket服务器
```
解决方案: 检查 SmartRobotAgent 是否已启动
```

### 问题2: 响应超时
```
解决方案: 检查电机节点是否正常运行，增加timeout参数
```

### 问题3: 角度超出限制
```
解决方案: 检查输入角度是否在物理限位范围内
```

## 开发者信息

- 脚本版本: 1.0
- 创建日期: 2026-05-12
- 基于: test.py (原组合电机控制测试脚本)
- 适配接口: publish_four_combine_motor_control (4自由度头颈运控)
