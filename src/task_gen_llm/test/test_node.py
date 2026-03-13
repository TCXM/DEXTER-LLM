#! /usr/bin/python

import rospy, json, pprint
from task_gen_llm.srv import TaskGenSrv, TaskGenSrvResponse, TaskGenSrvRequest


class TestNode:
    def __init__(self):
        self.gen_task_client = rospy.ServiceProxy("/gen_tasks", TaskGenSrv)
        self.gen_task_client.wait_for_service()
        situation_dict = {}
        situation_dict["Resources"] = [
            "There is a water pull at (10, 10, 1)"
        ]
        situation_dict["Task"] = "Extinguish flame_1 at (2, 2, 1)"
        resp: TaskGenSrvResponse = self.gen_task_client(
            TaskGenSrvRequest(json.dumps(situation_dict))
        )
        pprint.pprint(json.loads(resp.tg_result_json))


if __name__ == "__main__":
    rospy.init_node("task_gen_manager")
    TestNode()
    rospy.spin()
