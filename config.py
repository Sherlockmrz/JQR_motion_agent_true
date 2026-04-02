import os
from dataclasses import dataclass, field


@dataclass
class AgentConfig:
    """Agent配置类 - 从环境变量读取，带合理默认值"""

    # 版本
    AGENT_VERSION: str = "1.0.8"

    # 本地模型服务配置
    LOCAL_MODEL_URI: str = field(
        default_factory=lambda: os.getenv(
            "LOCAL_MODEL_URI", "ws://192.168.31.43:8000/ws/navigate"
        )
    )

    # OpenAI兼容客户端配置
    OPENAI_API_KEY: str = field(
        default_factory=lambda: os.getenv("OPENAI_API_KEY", "0")
    )
    OPENAI_BASE_URL: str = field(
        default_factory=lambda: os.getenv(
            "OPENAI_BASE_URL", "http://192.168.31.43:9000/v1"
        )
    )
    OPENAI_MODEL: str = field(
        default_factory=lambda: os.getenv(
            "OPENAI_MODEL", "Qwen3-VL-30B-A3B-Instruct"
        )
    )

    # USB串口配置
    USB_SERIAL_PORT: str = field(
        default_factory=lambda: os.getenv("USB_SERIAL_PORT", "/dev/ttyACM0")
    )
    USB_SERIAL_BAUDRATE: int = field(
        default_factory=lambda: int(os.getenv("USB_SERIAL_BAUDRATE", "115200"))
    )

    # 数据库和文件路径
    DB_PATH: str = field(
        default_factory=lambda: os.getenv("DB_PATH", "history.db")
    )
    ASM_JSON_PATH: str = field(
        default_factory=lambda: os.getenv("ASM_JSON_PATH", "asm_data.json")
    )
    VIDEO_BASE_DIR: str = field(
        default_factory=lambda: os.getenv("VIDEO_BASE_DIR", "videos")
    )

    # WebSocket配置
    WEBSOCKET_HOST: str = field(
        default_factory=lambda: os.getenv("WEBSOCKET_HOST", "127.0.0.1")
    )
    WEBSOCKET_PORT: int = field(
        default_factory=lambda: int(os.getenv("WEBSOCKET_PORT", "8766"))
    )

    # 导航位置文件路径
    WELCOME_POSITION_FILE: str = field(
        default_factory=lambda: os.getenv("WELCOME_POSITION_FILE", "/home/sunrise/welcome_position.txt")
    )

    # USB串口开关（设为false可禁用串口，仅用WebSocket通信）
    USB_SERIAL_ENABLED: bool = field(
        default_factory=lambda: os.getenv("USB_SERIAL_ENABLED", "false").lower() in ("true", "1", "yes")
    )

    # ReAct框架配置
    MAX_REACT_ITERATIONS: int = 8
    AGENT_MEMORY_SIZE: int = 50


config = AgentConfig()
