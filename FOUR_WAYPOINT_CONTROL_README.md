# 四自由度头颈运控多路点组合电机控制接口文档

## 概述

`four_combine_waypoint_control` 接口支持一次性下发多个路点，下游电机控制器按顺序执行，并上报执行进度和最终结果。相比单路点接口（`four_combine_motor_control`），多路点接口减少了通信次数，提高了执行效率，适合需要连续动作序列的场景。

## 话题定义

### 控制指令话题
- **话题名**: `/four_combine_waypoint_control`
- **消息类型**: `std_msgs/msg/Float32MultiArray`
- **数据格式**: 固定头部（3字段）+ N个路点（每个12字段）

### 结果反馈话题
- **话题名**: `/four_combine_waypoint_control_result`
- **消息类型**: `std_msgs/msg/Float32MultiArray`
- **数据格式**: `[task_id, result]`
  - `result` 为 101/102/103/104 表示最终结果
  - `result` 为 0~100 的浮点数表示路点完成进度百分比

## 数据格式详解

### 任务头部（固定3字段）
```
data[0]  task_id           # 任务ID，确保一个工作周期内唯一
data[1]  pose_mode         # 0=相对位姿 1=绝对位姿
data[2]  waypoint_count    # 路点数量N（必须 >= 1）
```

### 第i个路点（12字段，base = 3 + i*12）
```
data[base+0]   control_yaw           # 是否控制偏航（0.0/1.0）
data[base+1]   yaw_angle             # 偏航目标角度（弧度）
data[base+2]   control_roll          # 是否控制翻滚（0.0/1.0）
data[base+3]   roll_angle            # 翻滚目标角度（弧度）
data[base+4]   control_pitch         # 是否控制俯仰（0.0/1.0）
data[base+5]   pitch_angle           # 俯仰目标角度（弧度）
data[base+6]   control_chassis_move  # 是否控制底盘位移（0.0/1.0）
data[base+7]   chassis_offset        # 底盘位置偏移量（米，+前进/-后退）
data[base+8]   control_chassis_rotate # 是否控制底盘旋转（0.0/1.0）
data[base+9]   chassis_rotation      # 底盘旋转偏移量（弧度，+逆时针/-顺时针）
data[base+10]  speed_level           # 执行档位（0=低速 1=中速 2=快速）
data[base+11]  timeout               # 本路点超时时间（秒，0=无限制）
```

### 位姿模式说明
- **相对位姿（pose_mode=0）**: 每个路点的角度/位移是相对于上一个路点的增量
- **绝对位姿（pose_mode=1）**: 每个路点的角度/位移是绝对目标值

### 结果码定义
- `101`: SUCCESS - 执行成功
- `102`: ABORTED - 执行中止
- `103`: FAILED - 执行失败
- `104`: REJECTED - 拒绝执行

## Python Agent 接口使用

### 基本用法

```python
import math

# 定义路点序列
waypoints = [
    # 路点0: 头部偏航30°
    {
        'control_yaw': True,
        'yaw_angle': math.radians(30),
        'speed_level': 2,
        'timeout': 5.0
    },
    # 路点1: 头部俯仰-20°
    {
        'control_pitch': True,
        'pitch_angle': math.radians(-20),
        'speed_level': 1,
        'timeout': 5.0
    },
    # 路点2: 底盘前进0.3米 + 旋转45°
    {
        'control_chassis_move': True,
        'chassis_offset': 0.3,
        'control_chassis_rotate': True,
        'chassis_rotation': math.radians(45),
        'speed_level': 2,
        'timeout': 8.0
    }
]

# 调用接口（相对位姿模式）
result = await agent.ros2_interface.set_four_combine_waypoint_control(
    waypoints=waypoints,
    pose_mode=0,  # 0=相对位姿
    timeout=30.0  # 总超时时间
)

# 检查结果
if result['success']:
    print(f"执行成功，task_id={result['task_id']}")
    print(f"进度上报: {result.get('progress', [])}")
else:
    print(f"执行失败: {result.get('error_msg')}")
```

### 通过任务类型调用

```python
# 在 SmartRobotAgent 中使用
task = {
    "type": "set_four_combine_waypoint_control",
    "params": {
        "waypoints": waypoints,
        "pose_mode": 0,
        "timeout": 30.0
    }
}

result = await agent.execute_task(task)
```

## 测试验证

### 启动 Mock 节点

```bash
# 启动模拟电机控制器（进度模式）
python3 mock_four_waypoint_node.py --mode progress

# 其他模式
python3 mock_four_waypoint_node.py --mode success   # 成功模式
python3 mock_four_waypoint_node.py --mode fail      # 失败模式
python3 mock_four_waypoint_node.py --mode abort     # 中止模式
python3 mock_four_waypoint_node.py --mode reject    # 拒绝模式
python3 mock_four_waypoint_node.py --mode random    # 随机结果
```

### 运行测试用例

```bash
# 运行单个测试用例
python3 test_four_waypoint_control.py --case single

# 运行全部测试用例
python3 test_four_waypoint_control.py --case all

# 集成测试（自动启动 mock 节点）
./test_waypoint_integration.sh
./test_waypoint_integration.sh --case multi
```

## 应用场景示例

### 场景1: 巡逻扫视
```python
# 机器人巡逻中左右扫视观察环境
waypoints = [
    # 头部左转
    {'control_yaw': True, 'yaw_angle': math.radians(-60), 'speed_level': 1, 'timeout': 3.0},
    # 头部回正
    {'control_yaw': True, 'yaw_angle': math.radians(0), 'speed_level': 1, 'timeout': 3.0},
    # 头部右转
    {'control_yaw': True, 'yaw_angle': math.radians(60), 'speed_level': 1, 'timeout': 3.0},
    # 头部回正
    {'control_yaw': True, 'yaw_angle': math.radians(0), 'speed_level': 1, 'timeout': 3.0},
]
```

### 场景2: 检查桌面物品
```python
# 机器人靠近桌子，头部低头扫视桌面
waypoints = [
    # 底盘前进至桌边
    {'control_chassis_move': True, 'chassis_offset': 0.5, 'speed_level': 1, 'timeout': 5.0},
    # 头部低头看桌面
    {'control_pitch': True, 'pitch_angle': math.radians(30), 'speed_level': 2, 'timeout': 3.0},
    # 头部左扫
    {'control_yaw': True, 'yaw_angle': math.radians(-90), 'speed_level': 1, 'timeout': 4.0},
    # 头部右扫
    {'control_yaw': True, 'yaw_angle': math.radians(90), 'speed_level': 1, 'timeout': 4.0},
    # 头部回正
    {'control_yaw': True, 'yaw_angle': math.radians(0), 'speed_level': 1, 'timeout': 3.0},
    # 头部抬头回正
    {'control_pitch': True, 'pitch_angle': math.radians(0), 'speed_level': 1, 'timeout': 3.0},
]
```

### 场景3: 转身寻找
```python
# 机器人转身180度寻找目标
waypoints = [
    # 头部先转到极限角度
    {'control_yaw': True, 'yaw_angle': math.radians(110), 'speed_level': 2, 'timeout': 3.0},
    # 底盘旋转剩余角度
    {'control_chassis_rotate': True, 'chassis_rotation': math.radians(70), 'speed_level': 2, 'timeout': 5.0},
    # 头部回正
    {'control_yaw': True, 'yaw_angle': math.radians(0), 'speed_level': 2, 'timeout': 3.0},
]
```

### 场景4: 复杂导航动作
```python
# 边走边看 - 底盘前进同时头部左右观察
waypoints = [
    # 底盘前进 + 头部左转
    {
        'control_chassis_move': True, 'chassis_offset': 0.3,
        'control_yaw': True, 'yaw_angle': math.radians(-45),
        'speed_level': 1, 'timeout': 5.0
    },
    # 继续前进 + 头部右转
    {
        'control_chassis_move': True, 'chassis_offset': 0.3,
        'control_yaw': True, 'yaw_angle': math.radians(45),
        'speed_level': 1, 'timeout': 5.0
    },
    # 停止前进 + 头部回正
    {
        'control_yaw': True, 'yaw_angle': math.radians(0),
        'speed_level': 2, 'timeout': 3.0
    },
]
```

## 与单路点接口的对比

| 特性 | 单路点接口 | 多路点接口 |
|-----|-----------|-----------|
| 话题名 | `/four_combine_motor_control` | `/four_combine_waypoint_control` |
| 一次下发 | 1个路点 | N个路点 |
| 进度上报 | 无 | 支持（每个路点完成后上报进度百分比） |
| 通信次数 | N次（N个路点需要N次通信） | 1次（一次性下发全部路点） |
| 适用场景 | 单个动作、即时控制 | 连续动作序列、预定轨迹 |
| 执行效率 | 较低（需要等待每次反馈） | 较高（下游批量执行） |

## 注意事项

1. **超时时间设置**: 
   - 每个路点的 `timeout` 字段只约束该路点的执行时间
   - Agent 接口的 `timeout` 参数应设置为所有路点超时之和 + 余量

2. **任务ID唯一性**: 
   - 使用 `_next_motor_task_id()` 自动生成唯一ID
   - 同一工作周期内不要重复使用相同的 task_id

3. **位姿模式选择**:
   - 相对位姿：适合连续增量动作（如左右摆头、绕圈等）
   - 绝对位姿：适合多个独立目标位置（如依次看向不同方向）

4. **路点数量限制**:
   - 最少1个路点
   - 建议不超过20个路点（避免数据包过大）
   - 超长序列可以分批发送

5. **进度上报机制**:
   - 每完成一个路点，上报当前总进度百分比
   - 最后一个路点完成后，上报最终结果码（101/102/103/104）
   - Agent 可以实时接收进度更新

## 故障排查

### 问题1: 超时无反馈
- **原因**: 下游节点未启动或话题名称不匹配
- **解决**: 检查 `ros2 topic list` 确认话题存在，检查节点是否运行

### 问题2: 执行失败（result=103）
- **原因**: 路点参数超出电机物理极限，或电机故障
- **解决**: 检查角度/距离是否在安全范围内，检查电机状态

### 问题3: 拒绝执行（result=104）
- **原因**: 任务ID重复、数据格式错误、或电机繁忙
- **解决**: 确保生成唯一 task_id，检查数据长度是否正确

### 问题4: 中途中止（result=102）
- **原因**: 用户手动中断、紧急停止、或碰撞检测触发
- **解决**: 检查是否有急停信号，检查障碍物检测

## 版本历史

- **v1.0** (2025-01): 初始版本，支持多路点控制和进度上报
- 基于 `four_combine_motor_control` 单路点接口扩展

## 相关文件

- `mock_four_waypoint_node.py` - 模拟电机控制节点
- `test_four_waypoint_control.py` - 端到端测试脚本
- `test_waypoint_integration.sh` - 集成测试脚本
- `smart_robot_agent.py` - Agent 集成实现（第1126-1197行：监控，第1373-1486行：发布和等待，第3996-4065行：高层接口）
