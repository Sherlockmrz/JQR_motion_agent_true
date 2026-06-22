import asyncio
import json
import websockets
import time

AGENT_URI = "ws://127.0.0.1:8766"

async def main():
    print("准备连接 Agent：", AGENT_URI)

    try:
        async with websockets.connect(
            AGENT_URI,
            ping_interval=None,
            ping_timeout=None,
            open_timeout=10
        ) as ws:
            print("连接 Agent 成功！")

            msg = {
                "type": "go_to_object",
                "params": {
                    "obj_name": "水杯",
                    "pixel_position": None
                }
            }

            print("准备发送给 Agent：", msg)
            await ws.send(json.dumps(msg, ensure_ascii=False))
            print("已经发送，开始等待 Agent 返回...")

            start = time.time()

            while True:
                try:
                    resp = await asyncio.wait_for(ws.recv(), timeout=180)
                except asyncio.TimeoutError:
                    print("180秒没有收到 Agent 返回，测试结束")
                    break

                print("Agent 返回：", resp)

                try:
                    data = json.loads(resp)

                    if "success" in data:
                        print("收到 Agent 最终返回，测试结束")
                        break

                except Exception as e:
                    print("返回解析失败：", e)

                if time.time() - start > 240:
                    print("总等待超过240秒，主动结束")
                    break

    except Exception as e:
        print("连接 Agent 失败：", repr(e))

asyncio.run(main())