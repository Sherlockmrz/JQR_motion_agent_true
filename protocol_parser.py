#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""协议解析器Python版本"""

import json
import logging
from enum import IntEnum
from typing import Optional, Dict, Any, Tuple, List, Tuple

logger = logging.getLogger(__name__)

class CommandType(IntEnum):
    """命令字定义"""
    CMD_JSON_DATA = 0x01
    CMD_JSON_RESPONSE = 0x81
    CMD_STATUS_QUERY = 0x02
    CMD_PARAM_SET = 0x03

class ParserState(IntEnum):
    """解析器状态"""
    STATE_WAIT_HEADER1 = 0
    STATE_WAIT_HEADER2 = 1
    STATE_WAIT_LENGTH = 2
    STATE_WAIT_CMD = 3
    STATE_WAIT_DATA = 4
    STATE_WAIT_TAIL1 = 5
    STATE_WAIT_TAIL2 = 6

class ParseResult(IntEnum):
    """解析结果"""
    PARSE_OK = 0
    PARSE_INCOMPLETE = 1
    PARSE_ERROR_HEADER = 2
    PARSE_ERROR_LENGTH = 3
    PARSE_ERROR_TAIL = 4
    PARSE_ERROR_CMD = 5
    PARSE_ERROR_CRC = 6

class FrameData:
    """帧数据结构"""
    def __init__(self):
        self.data_length = 0
        self.command = 0
        self.data = bytearray()
        self.data_received = 0
    
    def reset(self):
        """重置帧数据"""
        self.data_length = 0
        self.command = 0
        self.data.clear()
        self.data_received = 0

class ProtocolParser:
    """协议解析器"""
    
    # 协议常量
    FRAME_HEADER_1 = 0xAA
    FRAME_HEADER_2 = 0x55
    FRAME_TAIL_1 = 0x0D
    FRAME_TAIL_2 = 0x0A
    
    MAX_FRAME_SIZE = 260  # 2+1+1+255+2
    MAX_JSON_SIZE = 1024
    
    def __init__(self):
        """初始化解析器"""
        self.state = ParserState.STATE_WAIT_HEADER1
        self.current_frame = FrameData()
        self.buffer = bytearray()
    
    def reset(self):
        """重置解析器状态"""
        self.state = ParserState.STATE_WAIT_HEADER1
        self.current_frame.reset()
        self.buffer.clear()
    
    def parse_byte(self, byte: int) -> ParseResult:
        """解析单个字节"""
        try:
            if self.state == ParserState.STATE_WAIT_HEADER1:
                if byte == self.FRAME_HEADER_1:
                    self.state = ParserState.STATE_WAIT_HEADER2
                    self.buffer.clear()
                    self.buffer.append(byte)
                    logger.debug(f"收到帧头1: 0x{byte:02X}")
                else:
                    # 跳过无效字节
                    pass
                    
            elif self.state == ParserState.STATE_WAIT_HEADER2:
                if byte == self.FRAME_HEADER_2:
                    self.state = ParserState.STATE_WAIT_LENGTH
                    self.buffer.append(byte)
                    logger.debug(f"收到帧头2: 0x{byte:02X}")
                else:
                    self.state = ParserState.STATE_WAIT_HEADER1
                    return ParseResult.PARSE_ERROR_HEADER
                    
            elif self.state == ParserState.STATE_WAIT_LENGTH:
                self.current_frame.data_length = byte
                self.current_frame.data_received = 0
                self.state = ParserState.STATE_WAIT_CMD
                self.buffer.append(byte)
                logger.debug(f"数据长度: {byte}")
                
            elif self.state == ParserState.STATE_WAIT_CMD:
                self.current_frame.command = byte
                self.buffer.append(byte)
                if self.current_frame.data_length > 0:
                    self.state = ParserState.STATE_WAIT_DATA
                else:
                    self.state = ParserState.STATE_WAIT_TAIL1
                logger.debug(f"命令字: 0x{byte:02X}")
                
            elif self.state == ParserState.STATE_WAIT_DATA:
                if self.current_frame.data_received < self.current_frame.data_length:
                    self.current_frame.data.append(byte)
                    self.current_frame.data_received += 1
                    self.buffer.append(byte)
                    
                    if self.current_frame.data_received >= self.current_frame.data_length:
                        self.state = ParserState.STATE_WAIT_TAIL1
                        logger.debug("数据接收完成")
                else:
                    self.state = ParserState.STATE_WAIT_HEADER1
                    return ParseResult.PARSE_ERROR_LENGTH
                    
            elif self.state == ParserState.STATE_WAIT_TAIL1:
                if byte == self.FRAME_TAIL_1:
                    self.state = ParserState.STATE_WAIT_TAIL2
                    self.buffer.append(byte)
                    logger.debug(f"收到帧尾1: 0x{byte:02X}")
                else:
                    self.state = ParserState.STATE_WAIT_HEADER1
                    return ParseResult.PARSE_ERROR_TAIL
                    
            elif self.state == ParserState.STATE_WAIT_TAIL2:
                if byte == self.FRAME_TAIL_2:
                    self.buffer.append(byte)
                    self.state = ParserState.STATE_WAIT_HEADER1
                    
                    # 验证数据长度
                    if self.current_frame.data_received != self.current_frame.data_length:
                        return ParseResult.PARSE_ERROR_LENGTH
                    
                    logger.debug(f"收到帧尾2: 0x{byte:02X}，帧解析完成")
                    return ParseResult.PARSE_OK
                else:
                    self.state = ParserState.STATE_WAIT_HEADER1
                    return ParseResult.PARSE_ERROR_TAIL
                    
            return ParseResult.PARSE_INCOMPLETE
            
        except Exception as e:
            logger.error(f"解析字节时出错: {e}")
            self.reset()
            return ParseResult.PARSE_ERROR_HEADER
    
    def parse_buffer(self, data: bytes) -> Tuple[ParseResult, List[Dict[Any, Any]]]:
        """解析缓冲区数据，返回解析结果和解析到的所有JSON消息列表

        Args:
            data: 接收到的原始数据

        Returns:
            Tuple[ParseResult, List[Dict]]: (最终解析结果, JSON消息列表)
        """
        result = ParseResult.PARSE_INCOMPLETE
        json_messages = []

        for byte in data:
            result = self.parse_byte(byte)

            # 当解析完一帧后，提取JSON消息并重置状态继续解析下一帧
            if result == ParseResult.PARSE_OK:
                # 提取JSON数据
                json_data = self.extract_json_data()
                if json_data:
                    json_messages.append(json_data)

                # 重置解析器准备下一帧（关键修复：不返回，继续处理剩余字节）
                self.reset()
                result = ParseResult.PARSE_INCOMPLETE

            elif result in [ParseResult.PARSE_ERROR_HEADER,
                          ParseResult.PARSE_ERROR_TAIL,
                          ParseResult.PARSE_ERROR_LENGTH]:
                # 解析出错时重置状态，继续处理剩余字节
                self.reset()
                result = ParseResult.PARSE_INCOMPLETE

        return result, json_messages
    
    def extract_json_from_frame(self) -> Optional[str]:
        """从帧中提取JSON数据"""
        if self.current_frame.command != CommandType.CMD_JSON_DATA and self.current_frame.command != CommandType.CMD_JSON_RESPONSE:
            logger.warning(f"不是JSON命令，命令字: 0x{self.current_frame.command:02X}")
            return None
        
        if not self.current_frame.data:
            return None
            
        try:
            json_str = self.current_frame.data.decode('utf-8')
            logger.debug(f"提取JSON: {json_str}")
            return json_str
        except UnicodeDecodeError as e:
            logger.error(f"JSON解码错误: {e}")
            return None
    
    def extract_json_data(self) -> Optional[Dict[Any, Any]]:
        """提取JSON数据并解析为字典"""
        json_str = self.extract_json_from_frame()
        if json_str is None:
            return None
            
        try:
            return json.loads(json_str)
        except json.JSONDecodeError as e:
            logger.error(f"JSON解析错误: {e}")
            return None
    
    def create_response_frame(self, command: CommandType, json_data: str) -> bytes:
        """创建响应帧"""
        json_bytes = json_data.encode('utf-8')
        json_length = len(json_bytes)
        
        if json_length > self.MAX_JSON_SIZE:
            raise ValueError(f"JSON数据过长: {json_length} > {self.MAX_JSON_SIZE}")
        
        # 构建帧: 帧头(2) + 长度(1) + 命令(1) + 数据 + 帧尾(2)
        frame = bytearray()
        frame.append(self.FRAME_HEADER_1)
        frame.append(self.FRAME_HEADER_2)
        frame.append(json_length)
        frame.append(command)
        frame.extend(json_bytes)
        frame.append(self.FRAME_TAIL_1)
        frame.append(self.FRAME_TAIL_2)
        
        return bytes(frame)
    
    def print_frame_info(self):
        """打印帧信息"""
        frame = self.current_frame
        print(f"=== Frame Information ===")
        print(f"Data Length: {frame.data_length} bytes")
        print(f"Command: 0x{frame.command:02X}")
        
        if frame.command == CommandType.CMD_JSON_DATA:
            print("Command Type: JSON Data")
        elif frame.command == CommandType.CMD_STATUS_QUERY:
            print("Command Type: Status Query")
        elif frame.command == CommandType.CMD_PARAM_SET:
            print("Command Type: Parameter Set")
        else:
            print("Command Type: Unknown")
        
        print("Data (hex): ", end="")
        for i, byte in enumerate(frame.data[:32]):  # 只显示前32字节
            print(f"{byte:02X} ", end="")
        if len(frame.data) > 32:
            print("...")
        else:
            print()