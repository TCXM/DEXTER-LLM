#!/usr/bin/env python3
import rospy, json, copy, pprint
from task_gen_client import TaskGenClient
from task_gen_llm.srv import TaskGenSrv, TaskGenSrvResponse, TaskGenSrvRequest
from typing import List, Dict


class TaskGenManager:
    def __init__(self):
        log_dir = rospy.get_param("~log_dir", "None")
        self.mode = rospy.get_param("~mode", "auto")
        self.client = TaskGenClient(log_dir)

        # Task Generation Service
        promp_template_path = rospy.get_param("~prompt_template_path")
        self.prompt_template = json.load(open(promp_template_path))
        self.meta_policy = {}
        self.gen_tasks_server = rospy.Service(
            "/gen_tasks", TaskGenSrv, self.gen_tasks_cb
        )

    def gen_tasks_cb(self, req: TaskGenSrvRequest):
        """
        req template:
        {
            "Resources": [
                "...",
                "..."
            ],
            "Task": ""
        }
        """
        rospy.loginfo("Received a task generation request with situation:")
        tg_situation = json.loads(req.tg_situation_json)
        pprint.pprint(tg_situation)
        resources = tg_situation["Resources"]
        task = tg_situation["Task"]

        llm_result = self._gen_tasks(resources, task)

        return TaskGenSrvResponse(json.dumps(llm_result), True)

    def _gen_tasks(self, resources: List[str], task: str) -> dict:
        prompt = copy.deepcopy(self.prompt_template)
        prompt["Resources"] = resources
        prompt["Task"] = task
        prompt["Original Meta Policy"] = self.meta_policy
        if self.mode == "auto":
            result = json.loads(self.client.call_llm(json.dumps(prompt)))
            self.meta_policy = result["Fine Tuned Meta Policy"]
        elif self.mode == "manual":
            result = json.loads(self.client.call_llm_manual(json.dumps(prompt)))
            self.meta_policy = result["Fine Tuned Meta Policy"]
        elif self.mode == "preset":
            if not hasattr(self, "preset"):
                preset_file_path = rospy.get_param("~preset_file_path", "None")
                if preset_file_path == "None":
                    raise ValueError("Preset file path must be set in 'preset' mode")
                self.preset: Dict = json.load(open(preset_file_path))
            results: List[Dict] = self.preset[task]
            for result in results:
                if all([r in resources for r in result["Resources"]]):
                    return result
        else:
            raise ValueError(f"Invalid mode: {self.mode}")

        return result


if __name__ == "__main__":
    rospy.init_node("task_gen_manager")
    TaskGenManager()
    rospy.spin()
