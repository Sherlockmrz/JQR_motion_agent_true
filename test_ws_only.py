import asyncio
import websockets

URIS = [
    "ws://192.168.31.43:8000/ws/navigate",
    "ws://192.168.31.43:8000/ws/navigate?token=DISABLE",
]

async def test_uri(uri):
    print("\n只测试 WebSocket 握手：", uri)
    try:
        async with websockets.connect(
            uri,
            ping_interval=None,
            ping_timeout=None,
            open_timeout=20
        ):
            print("WebSocket 握手成功：", uri)
    except Exception as e:
        print("WebSocket 握手失败：", repr(e))

async def main():
    for uri in URIS:
        await test_uri(uri)

asyncio.run(main())