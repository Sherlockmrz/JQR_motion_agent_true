from openai import OpenAI

BASE_URL = "http://192.168.31.43:8000/v1"
MODEL = "Vln_Qwen3-VL-8B-Instruct-500-202602031754"

client = OpenAI(
    base_url=BASE_URL,
    api_key="EMPTY"
)

resp = client.chat.completions.create(
    model=MODEL,
    messages=[
        {
            "role": "user",
            "content": "你好，请用一句话告诉我你是否正常工作。"
        }
    ],
    max_tokens=100,
    temperature=0.0
)

print(resp.choices[0].message.content)