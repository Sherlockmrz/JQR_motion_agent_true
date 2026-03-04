# -*- coding: utf-8 -*-
"""WebSocket控制服务器 - 支持局域网终端控制"""
import asyncio
import json
import logging
import threading
import time
from datetime import datetime
from typing import Dict, Any, Set, Optional
import websockets
from websockets.server import WebSocketServerProtocol

logger = logging.getLogger(__name__)


class WebSocketControlServer:
    """WebSocket控制服务器
    
    允许局域网内的其他终端通过WebSocket发送控制命令
    通信协议:
    入参: {"type": "go_to_door", "params": {}}
    返回: {"success": true/false, "error_msg": "..."}
    """
    
    def __init__(self, agent, host: str = "0.0.0.0", port: int = 8766):
        """初始化WebSocket控制服务器
        
        Args:
            agent: SmartRobotAgent实例
            host (str): 监听地址，默认0.0.0.0监听所有网卡
            port (int): 监听端口，默认8766
        """
        self.agent = agent
        self.host = host
        self.port = port
        
        # 连接管理
        self.connected_clients: Set[WebSocketServerProtocol] = set()
        self.clients_lock = threading.Lock()
        
        # 服务器状态
        self.server = None
        self.server_thread = None
        self.running = False
        
        # 统计信息
        self.total_messages = 0
        self.total_errors = 0
        
    def start(self) -> bool:
        """启动WebSocket服务器（在独立线程中运行）
        
        Returns:
            bool: 启动是否成功
        """
        try:
            if self.running:
                logger.warning(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] WebSocket控制服务器已在运行")
                return True
            
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 正在启动WebSocket控制服务器 ws://{self.host}:{self.port}")
            
            # 创建新的事件循环和线程
            self.running = True
            self.server_thread = threading.Thread(
                target=self._run_server,
                daemon=True,
                name="WebSocketControlServer"
            )
            self.server_thread.start()
            
            # 等待服务器启动
            time.sleep(0.5)
            
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] WebSocket控制服务器已启动，监听端口: {self.port}")
            return True
            
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 启动WebSocket控制服务器失败: {e}")
            self.running = False
            return False
    
    def _run_server(self):
        """在独立线程中运行WebSocket服务器"""
        try:
            # 创建新的事件循环
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            # 启动服务器
            start_server = websockets.serve(
                self._handle_client,
                self.host,
                self.port,
                ping_interval=20,
                ping_timeout=60
            )
            
            self.server = loop.run_until_complete(start_server)
            
            # 保持运行
            loop.run_forever()
            
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] WebSocket服务器运行异常: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.running = False
    
    async def _handle_client(self, websocket: WebSocketServerProtocol, path: str):
        """处理客户端连接
        
        Args:
            websocket: WebSocket连接对象
            path: 请求路径
        """
        client_addr = websocket.remote_address
        logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 新的WebSocket客户端连接: {client_addr}")
        
        # 添加到连接列表
        with self.clients_lock:
            self.connected_clients.add(websocket)
        
        try:
            # 持续接收消息
            async for message in websocket:
                try:
                    await self._handle_message(websocket, message)
                except Exception as e:
                    logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 处理消息异常: {e}")
                    self.total_errors += 1
                    # 发送错误响应
                    error_response = {
                        "success": False,
                        "error_msg": f"处理消息失败: {str(e)}"
                    }
                    await websocket.send(json.dumps(error_response, ensure_ascii=False))
        
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] WebSocket客户端断开连接: {client_addr}")
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] WebSocket连接异常: {e}")
        finally:
            # 从连接列表移除
            with self.clients_lock:
                self.connected_clients.discard(websocket)
    
    async def _handle_message(self, websocket: WebSocketServerProtocol, message: str):
        """处理接收到的消息
        
        Args:
            websocket: WebSocket连接对象
            message: 消息内容
        """
        self.total_messages += 1
        start_time = time.time()
        
        try:
            # 解析消息
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] JSON解析失败: {message}")
                response = {
                    "success": False,
                    "error_msg": "无效的JSON格式"
                }
                await websocket.send(json.dumps(response, ensure_ascii=False))
                return
            
            # 验证消息格式
            if not isinstance(data, dict):
                response = {
                    "success": False,
                    "error_msg": "消息必须是JSON对象格式"
                }
                await websocket.send(json.dumps(response, ensure_ascii=False))
                return
            
            # 检查是否有type字段
            if "type" not in data:
                response = {
                    "success": False,
                    "error_msg": "消息必须包含type字段"
                }
                await websocket.send(json.dumps(response, ensure_ascii=False))
                return
            
            task_type = data.get("type")
            task_params = data.get("params", {})
            
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] [WebSocket] 收到控制命令: type={task_type}, params={task_params}")
            
            # 调用agent执行任务
            task = {
                "type": task_type,
                "params": task_params
            }
            
            # 检查agent是否可用
            if not self.agent:
                response = {
                    "success": False,
                    "error_msg": "Agent不可用"
                }
                await websocket.send(json.dumps(response, ensure_ascii=False))
                return
            
            # 执行任务（使用agent的execute_task方法）
            try:
                result = await self.agent.execute_task(task)
                
                # 构造响应（按照协议格式）
                response = {
                    "success": result.get("success", False),
                    "error_msg": result.get("error_msg", "") if not result.get("success") else ""
                }
                
                # 如果有额外的数据字段，也添加到响应中
                for key in ["result", "description", "data"]:
                    if key in result:
                        response[key] = result[key]
                
                # 保持type字段用于识别
                response["type"] = task_type
                
            except Exception as e:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 任务执行异常: {e}")
                response = {
                    "success": False,
                    "error_msg": f"任务执行失败: {str(e)}",
                    "type": task_type
                }
            
            # 发送响应
            elapsed = time.time() - start_time
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] [WebSocket] 任务执行完成: type={task_type}, success={response.get('success')}, 耗时={elapsed:.2f}s")
            await websocket.send(json.dumps(response, ensure_ascii=False))
            
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 处理消息失败: {e}")
            self.total_errors += 1
            response = {
                "success": False,
                "error_msg": f"处理失败: {str(e)}"
            }
            try:
                await websocket.send(json.dumps(response, ensure_ascii=False))
            except Exception:
                pass
    
    def stop(self):
        """停止WebSocket服务器"""
        try:
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 正在停止WebSocket控制服务器...")
            
            self.running = False
            
            # 关闭所有客户端连接
            with self.clients_lock:
                for client in list(self.connected_clients):
                    try:
                        asyncio.run(client.close())
                    except Exception:
                        pass
                self.connected_clients.clear()
            
            # 关闭服务器
            if self.server:
                try:
                    self.server.close()
                    asyncio.run(self.server.wait_closed())
                except Exception:
                    pass
            
            logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] WebSocket控制服务器已停止，统计: 消息总数={self.total_messages}, 错误总数={self.total_errors}")
            
        except Exception as e:
            logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 停止WebSocket控制服务器失败: {e}")
    
    def get_stats(self) -> Dict[str, Any]:
        """获取服务器统计信息
        
        Returns:
            Dict[str, Any]: 统计信息
        """
        with self.clients_lock:
            client_count = len(self.connected_clients)
        
        return {
            "running": self.running,
            "host": self.host,
            "port": self.port,
            "connected_clients": client_count,
            "total_messages": self.total_messages,
            "total_errors": self.total_errors
        }
    
    async def broadcast_message(self, message: Dict[str, Any]) -> int:
        """向所有连接的客户端广播消息
        
        Args:
            message: 要广播的消息
            
        Returns:
            int: 成功发送的客户端数量
        """
        if not self.running:
            return 0
        
        message_str = json.dumps(message, ensure_ascii=False)
        success_count = 0
        
        with self.clients_lock:
            clients = list(self.connected_clients)
        
        for client in clients:
            try:
                await client.send(message_str)
                success_count += 1
            except Exception as e:
                logger.error(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 广播消息失败: {e}")
        
        return success_count
