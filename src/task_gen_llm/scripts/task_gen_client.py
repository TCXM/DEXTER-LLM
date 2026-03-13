from openai import OpenAI
import requests
import json
from datetime import datetime
import rospy
from std_msgs.msg import Empty
import os

BASE_URL = ""
MODEL = ""
API_KEY = ""


class TaskGenClient:
    def __init__(self, log_dir: str = "src/task_gen_llm/logs/"):
        self.client = OpenAI(
            api_key=API_KEY,
            base_url=BASE_URL,
        )
        self.log_dir = log_dir
        self.llm_continue = False

    def call_llm(self, prompt: str) -> str:
        print("\033[94m[INFO]: Calling LLM ...\033[0m")
        response = self.client.chat.completions.create(
            model=MODEL,
            messages=[{"role": "user", "content": prompt}],
            stream=False,
            response_format={"type": "json_object"},
        )
        print("\033[94m[INFO]: LLM Response Received\033[0m")
        result = response.choices[0].message.content

        # Get the usage of the chat
        # hit_usage = response.usage.prompt_cache_hit_tokens
        # miss_usage = response.usage.prompt_cache_miss_tokens
        # print(f"Hit Usage: {hit_usage}, Miss Usage: {miss_usage}")

        # Log the input and output
        if self.log_dir != "None":
            prompt = json.loads(prompt)
            result = json.loads(result)
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            with open(f"{self.log_dir}{timestamp}_call_llm_log.json", "w") as log_file:
                json.dump({"prompt": prompt, "result": result}, log_file, indent=4)

        return json.dumps(result)

    def call_llm_manual(self, prompt: str) -> str:
        if not hasattr(self, "trigger_sub") or not self.trigger_sub:
            self.trigger_sub = rospy.Subscriber(
                "/llm_continue", Empty, self._llm_continue_cb
            )

        if self.log_dir == "None":
            raise ValueError("Log directory must be set in 'manual' mode")
        prompt_file_dir = f"{self.log_dir}llm_manual_prompt.json"
        result_file_dir = f"{self.log_dir}llm_manual_result.json"
        with open(prompt_file_dir, "w") as prompt_file:
            json.dump(json.loads(prompt), prompt_file, indent=4)
        with open(result_file_dir, "w") as result_file:
            result_file.write("")

        while not self.llm_continue:
            rospy.sleep(1)
        self.llm_continue = False

        with open(result_file_dir, "r") as result_file:
            result = result_file.read()

        os.remove(prompt_file_dir)
        os.remove(result_file_dir)

        # Log the input and output
        prompt = json.loads(prompt)
        result = json.loads(result)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        with open(f"{self.log_dir}llm_{timestamp}_log.json", "w") as log_file:
            json.dump({"prompt": prompt, "result": result}, log_file, indent=4)

        return json.dumps(result)

    def _llm_continue_cb(self, msg: Empty):
        self.llm_continue = True

    def get_balance(self):
        url = f"{BASE_URL}/user/balance"
        payload = {}
        headers = {
            "Accept": "application/json",
            "Authorization": f"Bearer {API_KEY}",
        }
        response = requests.request("GET", url, headers=headers, data=payload)
        return json.loads(response.text)["balance_infos"][0]["total_balance"]


if __name__ == "__main__":
    client = TaskGenClient()
    print(
        client.call_llm(
            json.dumps({"question": "What is the capital of France?", "format": "json"})
        )
    )
    print(client.get_balance())
