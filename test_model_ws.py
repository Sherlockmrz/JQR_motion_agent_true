import asyncio
import json
import websockets
import time

URI = "ws://192.168.31.9:18000/ws/navigate?token=DISABLE"

async def main():
    print("准备连接模型服务：", URI)

    try:
        async with websockets.connect(
            URI,
            ping_interval=None,
            ping_timeout=None,
            open_timeout=10
        ) as ws:
            print("连接成功！")

            msg = {
                "type": "go_to_object",
                "user_prompt": "去找水杯",
                "params": {
                    "obj_name": "水杯",
                    "pixel_position": None
                }
            }

            print("准备发送：", msg)
            await ws.send(json.dumps(msg, ensure_ascii=False))
            print("已经发送，开始等待返回...")

            start = time.time()

            while True:
                try:
                    resp = await asyncio.wait_for(ws.recv(), timeout=180)
                except asyncio.TimeoutError:
                    print("180秒没有收到新的返回，测试结束")
                    break

                print("收到模型返回：", resp)

                try:
                    data = json.loads(resp)

                    # 最终结果判断
                    if ("success" in data or "result" in data or "answer" in data) and "command" not in data:
                        print("收到最终结果，测试结束：", data)
                        break

                    # 中间状态
                    if data.get("type") == "status":
                        print("这是中间状态，不是最终结果，继续等...")

                except Exception as e:
                    print("返回不是标准 JSON 或解析失败：", e)

                if time.time() - start > 240:
                    print("总等待超过240秒，主动结束")
                    break

    except Exception as e:
        print("连接或通信失败：", repr(e))

asyncio.run(main())