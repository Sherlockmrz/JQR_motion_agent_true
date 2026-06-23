import asyncio
import json
import websockets

AGENT_URI = "ws://127.0.0.1:8766"

async def main():
    print("准备连接 Agent：", AGENT_URI)

    async with websockets.connect(
        AGENT_URI,
        ping_interval=None,
        ping_timeout=None,
        open_timeout=10
    ) as ws:
        print("连接 Agent 成功！")

        msg = {
            "type": "stop_navigate",
            "params": {}
        }

        print("准备发送给 Agent：", msg)
        await ws.send(json.dumps(msg, ensure_ascii=False))

        print("已经发送，开始等待 Agent 返回...")

        try:
            while True:
                resp = await asyncio.wait_for(ws.recv(), timeout=30)
                print("Agent 返回：", resp)

                try:
                    data = json.loads(resp)
                    if "success" in data or data.get("type") == "stop_navigate":
                        print("收到有效返回，测试结束")
                        break
                except Exception:
                    pass

        except asyncio.TimeoutError:
            print("30秒没有收到新返回，测试结束")

asyncio.run(main())
