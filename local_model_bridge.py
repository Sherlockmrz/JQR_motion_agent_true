import asyncio
import json
import websockets
from openai import OpenAI


HTTP_BASE_URL = "http://192.168.31.43:8000/v1"
MODEL = "Vln_Qwen3-VL-8B-Instruct-500-202602031754"

HOST = "127.0.0.1"
PORT = 9001


client = OpenAI(
    base_url=HTTP_BASE_URL,
    api_key="EMPTY"
)


def call_http_llm(prompt: str) -> str:
    """
    调用 mentor 给的 HTTP 大模型服务。
    """
    resp = client.chat.completions.create(
        model=MODEL,
        messages=[
            {
                "role": "system",
                "content": "你是机器人任务模拟执行助手。请根据输入任务，给出简短执行说明。"
            },
            {
                "role": "user",
                "content": prompt
            }
        ],
        max_tokens=256,
        temperature=0.0
    )

    return resp.choices[0].message.content


async def handle_client(websocket):
    print("Agent 已连接到本地模型适配器")

    async for message in websocket:
        print("\n收到 Agent 发来的消息：")
        print(message)

        try:
            data = json.loads(message)
            task_type = data.get("type", "unknown")

            # 先返回一条中间状态，模拟真实导航模型启动任务
            status_msg = {
                "type": "status",
                "message": f"任务 '{task_type}' 已由本地适配器接收，开始模拟执行。"
            }
            await websocket.send(json.dumps(status_msg, ensure_ascii=False))
            print("已发送中间状态：", status_msg)

            # 处理 Agent 内部 LLM 分析用的 talk
            if task_type == "talk":
                raw_message = data.get("message", "")
                prompt = f"请根据以下消息进行机器人任务分析，并尽量返回 JSON：\n{raw_message}"

                llm_result = call_http_llm(prompt)

                final_msg = {
                    "success": True,
                    "result": llm_result,
                    "error_msg": ""
                }

            # 处理 go_to_object
            elif task_type == "go_to_object":
                params = data.get("params", {})
                obj_name = params.get("obj_name", "未知物体")
                user_prompt = data.get("user_prompt", f"去找{obj_name}")

                llm_text = call_http_llm(
                    f"机器人收到任务：{user_prompt}。请用一句话模拟说明机器人已经理解并准备执行。"
                )

                final_msg = {
                    "success": True,
                    "error_msg": "",
                    "result": f"模拟完成 go_to_object：{obj_name}",
                    "description": llm_text
                }

            # 处理 go_to_person
            elif task_type == "go_to_person":
                person_id = data.get("person_id", "未知人员")
                user_prompt = data.get("user_prompt", f"寻找{person_id}")

                llm_text = call_http_llm(
                    f"机器人收到任务：{user_prompt}。请用一句话模拟说明机器人已经理解并准备执行。"
                )

                final_msg = {
                    "success": True,
                    "error_msg": "",
                    "result": f"模拟完成 go_to_person：{person_id}",
                    "description": llm_text
                }

            # 处理 follow_person
            elif task_type == "follow_person":
                llm_text = call_http_llm(
                    "机器人收到跟随人员任务。请用一句话模拟说明机器人已经进入跟随状态。"
                )

                final_msg = {
                    "success": True,
                    "error_msg": "",
                    "result": "模拟完成 follow_person",
                    "description": llm_text
                }

            # 处理 stop
            elif task_type == "stop":
                final_msg = {
                    "success": True,
                    "error_msg": "",
                    "result": "模拟停止任务成功"
                }

            # 其他任务统一模拟成功
            else:
                llm_text = call_http_llm(
                    f"机器人收到任务类型：{task_type}，参数：{json.dumps(data, ensure_ascii=False)}。请用一句话模拟执行结果。"
                )

                final_msg = {
                    "success": True,
                    "error_msg": "",
                    "result": f"模拟完成任务：{task_type}",
                    "description": llm_text
                }

            # 稍微等一下，模拟模型执行耗时
            await asyncio.sleep(1)

            await websocket.send(json.dumps(final_msg, ensure_ascii=False))
            print("已发送最终结果：")
            print(json.dumps(final_msg, ensure_ascii=False, indent=2))

        except Exception as e:
            error_msg = {
                "success": False,
                "error_msg": str(e)
            }
            await websocket.send(json.dumps(error_msg, ensure_ascii=False))
            print("处理失败：", e)


async def main():
    print(f"本地模型适配器启动：ws://{HOST}:{PORT}/ws/navigate")
    print(f"HTTP LLM 服务：{HTTP_BASE_URL}")
    print(f"模型：{MODEL}")

    async with websockets.serve(
        handle_client,
        HOST,
        PORT,
        ping_interval=None,
        ping_timeout=None
    ):
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())