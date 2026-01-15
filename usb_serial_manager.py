#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""USB串口通信管理器"""

import logging
import threading
import time
import json
from typing import Optional, Callable, Dict, Any, List, Tuple
from protocol_parser import ProtocolParser, CommandType, ParseResult

logger = logging.getLogger(__name__)

class SerialManager:
    """串口管理器 - 负责USB串口通信"""
    
    def __init__(self, port: str = "/dev/rk", baudrate: int = 115200):
        """初始化串口管理器
        
        Args:
            port: 串口设备路径
            baudrate: 波特率
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.parser = ProtocolParser()
        
        # 通信控制
        self.is_running = False
        self.receive_thread = None
        
        # 回调函数列表
        self.message_callbacks: List[Callable[[Dict[Any, Any]], None]] = []

        # 任务响应管理
        self.task_responses: Dict[str, List[Dict[Any, Any]]] = {}

        # 活跃任务类型管理 - 用于防止同一类型任务并发执行
        # 修改：使用细粒度锁，不同类型任务可以并发执行
        self.active_task_types: set = set()  # 存储正在执行的任务类型
        self.task_type_locks: Dict[str, threading.Lock] = {}  # 每种任务类型一个独立的锁

        # 自发自收过滤
        self.sent_messages: List[str] = []  # 存储最近发送的消息的JSON字符串
        self.max_sent_messages = 3  # 最多存储3条最近发送的消息

        # 线程锁
        self.lock = threading.Lock()
        
        # logger.info(f"串口管理器初始化完成: {port}@{baudrate}")
    
    def add_callback(self, callback: Callable[[Dict[Any, Any]], None]):
        """添加消息回调函数
        
        Args:
            callback: 接收解析后的JSON消息的回调函数
        """
        with self.lock:
            self.message_callbacks.append(callback)
        logger.info(f"添加消息回调函数: {callback.__name__}")
    
    def remove_callback(self, callback: Callable[[Dict[Any, Any]], None]):
        """移除消息回调函数"""
        with self.lock:
            if callback in self.message_callbacks:
                self.message_callbacks.remove(callback)
                logger.info(f"移除消息回调函数: {callback.__name__}")
    
    async def connect(self) -> bool:
        """连接到串口设备"""
        try:
            # 这里需要导入pyserial
            import serial
            
            # 尝试连接多个可能的串口设备
            possible_ports = self.port
            # logger.info(f"即将连接到串口: {possible_ports}")
            
            try:
                self.serial_port = serial.Serial(
                    port=possible_ports,
                    baudrate=self.baudrate,
                    timeout=0.1,  # 非阻塞读取
                    write_timeout=1.0
                )
                self.port = possible_ports
                logger.info(f"成功连接到串口: {possible_ports}")
                return True
            except (serial.SerialException, OSError):
                return False
            
        except ImportError as e:
            logger.error(f"pyserial库未安装: {e}")
            logger.info("使用虚拟串口模式进行测试")
            self.serial_port = VirtualSerialPort()
            return True
        except Exception as e:
            logger.error(f"连接串口失败: {e}")
            return False
    
    def start_receiving(self):
        """开始接收数据线程"""
        if self.is_running:
            logger.warning("接收线程已在运行")
            return
            
        self.is_running = True
        self.receive_thread = threading.Thread(
            target=self._receive_loop,
            name="SerialReceiver",
            daemon=True
        )
        self.receive_thread.start()
        logger.info("串口接收线程已启动")
    
    def stop_receiving(self):
        """停止接收数据线程"""
        self.is_running = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)
        
        if self.serial_port and hasattr(self.serial_port, 'close'):
            self.serial_port.close()
        
        logger.info("串口接收线程已停止")
    
    def _receive_loop(self):
        """接收数据循环"""
        # logger.info("串口接收循环开始")
        
        while self.is_running:
            try:
                if self.serial_port and hasattr(self.serial_port, 'in_waiting') and self.serial_port.in_waiting > 0:
                    # 读取可用数据
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    if data:
                        self._process_received_data(data)
                
                elif isinstance(self.serial_port, VirtualSerialPort):
                    # 虚拟串口模式
                    data = self.serial_port.read()
                    if data:
                        self._process_received_data(data)
                
                else:
                    time.sleep(0.01)  # 短暂休眠，避免CPU占用过高
                    
            except Exception as e:
                logger.error(f"接收数据时出错: {e}")
                time.sleep(0.1)  # 出错后稍长休眠
        
        logger.info("串口接收循环结束")
    
    def _process_received_data(self, data: bytes):
        """处理接收到的数据"""
        try:
            # logger.info(f"[RAW_DATA] 接收到原始数据: {len(data)}字节, 内容: {data.hex()}")
            
            # 解析协议数据
            result = self.parser.parse_buffer(data)
            # logger.info(f"[PARSER_RESULT] 解析结果: {result}")
            
            if result == ParseResult.PARSE_OK:
                # logger.debug(f"成功解析协议帧: {self.parser.buffer.hex()}")
                
                # 提取JSON数据
                json_data = self.parser.extract_json_data()
                if json_data:
                    # logger.info(f"[PARSER_DEBUG] 解析到JSON: {json_data}")
                    self._handle_received_message(json_data)
                
                # 重置解析器准备下一帧
                self.parser.reset()
                
            elif result == ParseResult.PARSE_ERROR_HEADER:
                logger.warning("协议帧头错误")
                self.parser.reset()
                
            elif result == ParseResult.PARSE_ERROR_TAIL:
                logger.warning("协议帧尾错误")
                self.parser.reset()
                
            elif result == ParseResult.PARSE_ERROR_LENGTH:
                logger.warning("协议长度错误")
                self.parser.reset()
                
        except Exception as e:
            logger.error(f"[RAW_DATA] 处理接收数据时出错: {e}")
            self.parser.reset()
    
    def _handle_received_message(self, message: Dict[Any, Any]):
        """处理接收到的消息"""
        try:
            # logger.info(f"处理接收到的消息: {message}")
            
            # 启用自发自收过滤
            if self._is_self_sent_message(message):
                logger.debug(f"过滤自发自收消息: {message}")
                return
            
            # 调用所有回调函数
            with self.lock:
                for callback in self.message_callbacks:
                    try:
                        # logger.info(f"调用回调函数: {callback.__name__}")
                        callback(message)
                    except Exception as e:
                        logger.error(f"回调函数执行失败: {e}")
            
            # 处理任务响应
            self._handle_task_response(message)
            
        except Exception as e:
            logger.error(f"处理消息时出错: {e}")
    
    def _handle_task_response(self, message: Dict[Any, Any]):
        """处理任务响应 - 仅存储响应，不阻塞任务执行"""
        try:
            # 从消息中提取任务信息
            task_type = message.get("type")
            task_id = message.get("task_id")

            if not task_type:
                return

            # 生成任务唯一标识
            if not task_id:
                # 从活跃任务集合中查找匹配的任务
                task_id = self._find_matching_task_id(task_type)

            if not task_id:
                return

            # 存储响应
            if task_id not in self.task_responses:
                self.task_responses[task_id] = []

            self.task_responses[task_id].append(message)

            # 检查是否为最终结果，如果则清除任务类型的活跃状态
            if self._is_final_result(message):
                with self.lock:
                    # 清除任务类型的活跃状态（通过消息响应完成的任务）
                    if task_type and task_type in self.active_task_types:
                        self.active_task_types.remove(task_type)
                        logger.info(f"任务类型 '{task_type}' 已执行完成 (通过消息响应)")

        except Exception as e:
            logger.error(f"处理任务响应时出错: {e}")
    


    def _find_matching_task_id(self, task_type: str) -> Optional[str]:
        """查找匹配的任务ID"""
        # 由于不再使用pending_tasks，这里返回None
        # task_responses存储历史响应，不需要按类型查找
        return None
    
    def _is_final_result(self, message: Dict[Any, Any]) -> bool:
        """判断是否为最终结果"""
        # 检查是否包含result或success字段（但排除command字段）
        if ("result" in message or "success" in message) and "command" not in message:
            return True
        
        # 检查是否包含results字段且包含成功或错误信息
        if "results" in message:
            results = message["results"]
            if any("success" in str(r).lower() or "error" in str(r).lower() for r in results):
                return True
        
        return False
    
    def send_message(self, message: Dict[Any, Any]) -> bool:
        """发送消息

        Args:
            message: 要发送的JSON消息

        Returns:
            bool: 发送是否成功
        """
        try:
            if not self.serial_port:
                logger.error("串口未连接")
                return False

            # 转换为JSON字符串
            json_str = json.dumps(message, ensure_ascii=False)

            # 记录发送的消息用于过滤自发自收
            self._record_sent_message(json_str)

            # 创建协议帧 (地瓜S100应答使用0x81)
            frame = self.parser.create_response_frame(CommandType.CMD_JSON_RESPONSE, json_str)
            # logger.info(f"创建协议帧: {frame.hex()}")
            # 发送数据
            if hasattr(self.serial_port, 'write'):
                bytes_written = self.serial_port.write(frame)
                if hasattr(self.serial_port, 'flush'):
                    self.serial_port.flush()

                # logger.info(f"已发送USB消息: {json_str} ({bytes_written} bytes)")
                return True
            else:
                logger.error("串口对象不支持写入操作")
                return False

        except Exception as e:
            logger.error(f"发送消息失败: {e}")
            return False
    
    def register_task(self, task_id: str, task_type: Optional[str] = None) -> Tuple[bool, str]:
        """注册任务，立即返回不等待响应

        Args:
            task_id: 任务ID
            task_type: 任务类型，用于并发控制

        Returns:
            Tuple[bool, str]: (是否成功, 错误消息)
        """
        # 初始化响应列表
        with self.lock:
            if task_id not in self.task_responses:
                self.task_responses[task_id] = []

        logger.info(f"注册任务: {task_id} (type: {task_type})")
        return True, ""
    
    def acquire_task_type_lock(self, task_type: str) -> Tuple[bool, str]:
        """尝试获取任务类型锁（支持并发）

        Args:
            task_type: 任务类型

        Returns:
            Tuple[bool, str]: (是否成功获取, 错误消息)
        """
        # 获取或创建该任务类型的专用锁
        with self.lock:
            if task_type not in self.task_type_locks:
                self.task_type_locks[task_type] = threading.Lock()
                # logger.info(f"[锁管理] 为任务类型 '{task_type}' 创建专用锁")

            task_lock = self.task_type_locks[task_type]

            # 检查该类型是否已在执行中
            if task_type in self.active_task_types:
                error_msg = f"任务类型 '{task_type}' 正在执行中，请等待当前任务完成"
                logger.warning(error_msg)
                return False, error_msg

            # 标记任务类型为活跃
            self.active_task_types.add(task_type)
            # logger.info(f"[锁管理] 任务类型 '{task_type}' 获取锁成功")

        # 尝试获取该任务类型的专用锁（非阻塞方式）
        # 使用超时避免死锁
        acquired = task_lock.acquire(blocking=False)
        if not acquired:
            # 理论上不应该到达这里，因为已经检查了active_task_types
            # 但为了安全起见，如果获取锁失败，从active_task_types中移除
            with self.lock:
                self.active_task_types.discard(task_type)
            error_msg = f"任务类型 '{task_type}' 锁获取失败"
            logger.warning(error_msg)
            return False, error_msg

        return True, ""
    
    def release_task_type_lock(self, task_type: str):
        """释放任务类型锁（支持并发）

        Args:
            task_type: 任务类型
        """
        with self.lock:
            # 从活跃任务集合中移除
            self.active_task_types.discard(task_type)

            # 释放该任务类型的专用锁
            if task_type in self.task_type_locks:
                self.task_type_locks[task_type].release()
                # logger.info(f"[锁管理] 任务类型 '{task_type}' 释放锁成功")
    
    def get_task_responses(self, task_id: str) -> List[Dict[Any, Any]]:
        """获取任务的所有响应"""
        return self.task_responses.get(task_id, [])
    
    def clear_task_responses(self, task_id: str, task_type: Optional[str] = None):
        """清除任务响应"""
        if task_id in self.task_responses:
            del self.task_responses[task_id]

        with self.lock:
            # 清除任务类型的活跃状态
            if task_type and task_type in self.active_task_types:
                self.active_task_types.remove(task_type)
                logger.info(f"任务类型 '{task_type}' 已执行完成")
    
    def _record_sent_message(self, json_str: str):
        """记录发送的消息用于自发自收过滤"""
        with self.lock:
            self.sent_messages.append(json_str)
            # 保持最多保存max_sent_messages条消息
            if len(self.sent_messages) > self.max_sent_messages:
                self.sent_messages.pop(0)
    
    def _is_self_sent_message(self, message: Dict[Any, Any]) -> bool:
        """检查是否是自己发送的消息"""
        try:
            # 暂时禁用自发自收过滤，确保测试程序能收到所有响应
            # 只检查完全相同的消息，不过滤响应类型
            message_json = json.dumps(message, sort_keys=True)
            with self.lock:
                for sent_msg in self.sent_messages:
                    try:
                        if json.dumps(json.loads(sent_msg), sort_keys=True) == message_json:
                            return True
                    except (json.JSONDecodeError, TypeError):
                        continue
            return False
        except Exception:
            return False
    
    def _is_agent_response(self, message: Dict[Any, Any]) -> bool:
        """检查消息是否是agent的响应"""
        try:
            # 暂时禁用消息过滤，允许所有消息通过
            # 这是为了调试通信问题，确保测试程序能收到agent的响应
            return False
            
            # 原有的过滤逻辑被注释掉
            # 检查常见的响应模式
            # response_patterns = [
            #     # 标准成功响应
            #     lambda msg: "success" in msg and "message" in msg and "收到命令" in msg.get("message", ""),
            #     # 任务执行结果
            #     lambda msg: "type" in msg and ("success" in msg or "error_msg" in msg),
            #     # 包含response字段的响应
            #     lambda msg: "response" in msg and ("command" in msg.get("response", {}) or "timestamp" in msg.get("response", {})),
            #     # 移动相关的响应
            #     lambda msg: "type" in msg and msg["type"] in ["get_move_mode", "stop_move"] and ("success" in msg or "error_msg" in msg),
            # ]
            
            # # 如果消息匹配任何响应模式，则认为是agent的响应
            # for pattern in response_patterns:
            #     if pattern(message):
            #         # 排除客户端发送的原始任务消息
            #         if "task" in message and "params" in message.get("task", {}):
            #             return False
            #         return True
            
            # return False
        except Exception:
            return False
    



class VirtualSerialPort:
    """虚拟串口类，用于测试"""
    
    def __init__(self):
        self.in_waiting = 0
        self._buffer = []
        self._index = 0
    
    def read(self, size: Optional[int] = None) -> bytes:
        """模拟读取数据"""
        if not self._buffer:
            # 生成测试数据
            self._generate_test_data()
        
        if self._index < len(self._buffer):
            data = self._buffer[self._index]
            self._index += 1
            return data
        else:
            self._buffer.clear()
            self._index = 0
            return b''
    
    def write(self, data: bytes) -> int:
        """模拟写入数据"""
        logger.info(f"虚拟串口写入: {data.hex()}")
        return len(data)
    
    def flush(self):
        """模拟刷新缓冲区"""
        pass
    
    def close(self):
        """模拟关闭串口"""
        pass
    
    def _generate_test_data(self):
        """生成测试数据"""
        import time
        
        # 创建一些测试消息
        test_messages = [
            {"type": "test", "message": ""},
            {"type": "status", "value": "ok"},
            {"command": "test_command", "result": "success"},
        ]
        
        for message in test_messages:
            json_str = json.dumps(message)
            
            # 创建协议帧
            parser = ProtocolParser()
            frame = parser.create_response_frame(CommandType.CMD_JSON_DATA, json_str)
            self._buffer.append(frame)
        
        self.in_waiting = sum(len(data) for data in self._buffer)