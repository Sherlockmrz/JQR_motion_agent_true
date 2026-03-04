# WebSocket控制接口使用说明

## 概述

智能机器人Agent现已支持WebSocket控制接口,允许局域网内的其他终端通过WebSocket协议发送控制命令。该功能与现有的USB串口通信并行工作,互不干扰。

## 架构

```
外部终端 (Web/App/其他客户端)
        ↓ WebSocket连接
WebSocketControlServer (端口: 8766)
        ↓ 任务转发
SmartRobotAgent (复用现有执行逻辑)
        ↓ 任务执行
机器人响应 (通过WebSocket返回)
```

## 功能特性

- ✅ 支持多客户端同时连接
- ✅ 复用现有的任务执行逻辑
- ✅ 标准化的JSON通信协议
- ✅ 实时响应反馈
- ✅ 错误处理和异常捕获
- ✅ 与USB串口通信并行工作

## 通信协议

### 请求格式

```json
{
  "type": "命令类型",
  "params": {
    "参数名": "参数值"
  }
}
```

### 响应格式

```json
{
  "success": true/false,
  "error_msg": "错误信息(仅失败时)",
  "type": "命令类型",
  // ... 其他返回数据
}
```

### 支持的命令类型

#### 导航相关
- `go_to_door` - 导航到门口
- `back_to_last_position` - 返回初始位置
- `go_to_object` - 导航到指定物体
- `go_find_person` - 查找指定人员
- `follow_person` - 跟随人员
- `stop_move` - 停止移动
- `stop_navigate` - 停止导航
- `stop_follow` - 停止跟随

#### 状态查询
- `get_move_mode` - 获取运动模式
- `get_robot_rise_state` - 获取升降状态
- `get_robot_tilt_state` - 获取俯仰状态
- `get_screen_tilt_state` - 获取屏幕俯仰状态
- `get_medicine_box_state` - 获取药箱状态
- `get_laser_pointer_state` - 获取激光笔状态
- `get_rgb_light_strip_state` - 获取RGB灯状态

#### 控制命令
- `set_robot_rise_jqr` - 控制升降
  - 参数: `{"rise": true/false}`
- `set_robot_tilt_jqr` - 控制俯仰
  - 参数: `{"angle": 角度值}`
- `set_screen_tilt_jqr` - 控制屏幕俯仰
  - 参数: `{"angle": 角度值}`
- `set_medicine_box_switch` - 控制药箱
  - 参数: `{"switch": true/false, "speed_stage": 1/2}`
- `set_laser_pointer` - 控制激光笔
  - 参数: `{"laser_pointer": true/false}`
- `set_rgb` - 控制RGB灯
  - 参数: `{"switch": true/false, "color": "颜色", "mode": 模式}`

#### 其他
- `find_person` - 静态找人
- `delete_person` - 删除人脸数据

## 使用方法

### 方法1: 使用测试脚本

```bash
# 运行测试脚本
python test_websocket_control.py

# 选择测试模式:
# 1 - 自动测试模式(运行预设测试用例)
# 2 - 交互式测试模式(手动输入命令)
```

### 方法2: 使用HTML客户端

1. 用浏览器打开 `websocket_control_client.html`
2. 点击"连接"按钮连接到WebSocket服务器
3. 使用预设按钮或自定义命令进行控制

### 方法3: 使用Python客户端代码

```python
import asyncio
import json
import websockets

async def send_command():
    uri = "ws://localhost:8766"  # 或使用机器人的IP地址
    
    async with websockets.connect(uri) as websocket:
        # 构造命令
        command = {
            "type": "go_to_door",
            "params": {}
        }
        
        # 发送命令
        await websocket.send(json.dumps(command))
        
        # 接收响应
        response = await websocket.recv()
        result = json.loads(response)
        
        print(f"成功: {result['success']}")
        print(f"错误信息: {result.get('error_msg', '')}")

# 运行
asyncio.run(send_command())
```

### 方法4: 使用JavaScript/Web客户端

```javascript
// 连接到WebSocket服务器
const ws = new WebSocket('ws://localhost:8766');

// 连接成功
ws.onopen = function() {
    console.log('已连接');
    
    // 发送命令
    const command = {
        type: 'go_to_door',
        params: {}
    };
    ws.send(JSON.stringify(command));
};

// 接收响应
ws.onmessage = function(event) {
    const response = JSON.parse(event.data);
    console.log('响应:', response);
};

// 错误处理
ws.onerror = function(error) {
    console.error('WebSocket错误:', error);
};
```

## 配置

### 修改WebSocket端口

在 `smart_robot_agent.py` 中修改:

```python
self.websocket_server = WebSocketControlServer(
    agent=self,
    host="0.0.0.0",  # 监听所有网卡
    port=8766        # 修改为您想要的端口
)
```

### 连接远程机器人

如果机器人运行在另一台机器上,使用机器人的IP地址连接:

```javascript
const ws = new WebSocket('ws://192.168.1.100:8766');  // 替换为机器人实际IP
```

## 示例

### 示例1: 导航到门口

**请求:**
```json
{
  "type": "go_to_door",
  "params": {}
}
```

**成功响应:**
```json
{
  "success": true,
  "error_msg": "",
  "type": "go_to_door"
}
```

**失败响应:**
```json
{
  "success": false,
  "error_msg": "导航未响应",
  "type": "go_to_door"
}
```

### 示例2: 控制RGB灯

**请求:**
```json
{
  "type": "set_rgb",
  "params": {
    "switch": true,
    "color": "green",
    "mode": 0
  }
}
```

**响应:**
```json
{
  "success": true,
  "error_msg": "",
  "type": "set_rgb"
}
```

### 示例3: 获取机器人状态

**请求:**
```json
{
  "type": "get_robot_rise_state",
  "params": {}
}
```

**响应:**
```json
{
  "success": true,
  "state": true,
  "description": "升起状态",
  "type": "get_robot_rise_state"
}
```

## 故障排查

### 无法连接到WebSocket服务器

1. **检查Agent是否启动**: 确保SmartRobotAgent正在运行
2. **检查端口占用**: 确认端口8766未被其他程序占用
   ```bash
   netstat -tulpn | grep 8766
   ```
3. **检查防火墙**: 确认防火墙允许该端口的连接
   ```bash
   sudo ufw allow 8766
   ```
4. **查看日志**: 检查Agent的启动日志,确认WebSocket服务器已启动

### 命令执行失败

1. **检查命令格式**: 确保JSON格式正确
2. **检查命令类型**: 确认使用的是支持的命令类型
3. **检查参数**: 确认参数格式和类型正确
4. **查看Agent日志**: 检查具体的错误信息

### 响应超时

某些命令(如导航)可能需要较长时间执行,请适当增加超时时间:

```python
response = await asyncio.wait_for(websocket.recv(), timeout=60.0)
```

## 性能考虑

- WebSocket服务器在独立线程中运行,不影响主事件循环
- 每个客户端连接都会创建独立的处理协程
- 任务执行复用现有的并发控制机制
- 建议单个客户端不要发送过于频繁的命令

## 安全建议

- 在生产环境中,建议添加身份认证机制
- 可以限制只监听localhost (127.0.0.1) 而不是 0.0.0.0
- 可以添加SSL/TLS加密 (wss://)
- 可以实现命令白名单机制

## 版本历史

- v1.0.0 (2024-03-04)
  - 初始版本
  - 支持基本的命令执行和响应
  - 支持多客户端连接
  - 提供测试工具和HTML客户端

## 技术支持

如有问题或建议,请联系开发团队或查看项目文档。
