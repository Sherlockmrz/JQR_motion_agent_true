import asyncio
import json
import websockets

URI = "ws://192.168.31.9:18000/ws/navigate?token=DISABLE"

async def main():
    print("连接真实导航服务：", URI)

    async with websockets.connect(
        URI,
        ping_interval=None,
        ping_timeout=None,
        open_timeout=10
    ) as ws:
        print("连接成功，发送 stop")

        msg = {
            "type": "stop"
        }

        await ws.send(json.dumps(msg, ensure_ascii=False))
        print("已发送：", msg)

        try:
            while True:
                resp = await asyncio.wait_for(ws.recv(), timeout=20)
                print("收到返回：", resp)
        except asyncio.TimeoutError:
            print("20秒没有新返回，停止测试")

asyncio.run(main())