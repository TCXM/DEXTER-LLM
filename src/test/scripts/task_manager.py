#!/usr/bin/env python3
from task_gen_llm.srv import TaskGenSrv, TaskGenSrvRequest, TaskGenSrvResponse
from task_alloc_milp.srv import TaskAllocSrv, TaskAllocSrvRequest, TaskAllocSrvResponse
from std_msgs.msg import String
from task_database import (
    TaskDatabase,
    WorldRobot,
    TaskInstance,
    WorldObject,
    TaskType,
    SubTaskType,
    SubTaskInstance,
)
import json, rospy, copy
from typing import List, Dict, Tuple
from scripts_player import ScriptsPlayer
from std_msgs.msg import Empty
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from visualization_msgs.msg import Marker
from marker_factory import MarkerFactory as MF
from jsk_rviz_plugins.msg import OverlayText


class TaskManager:
    def __init__(self):
        self.task_gen_client = rospy.ServiceProxy("/gen_tasks", TaskGenSrv)
        self.task_alloc_client = rospy.ServiceProxy("/alloc_tasks", TaskAllocSrv)
        self.r_s_sub = rospy.Subscriber("/robot_state", String, self._robot_state_cb)
        self.assign_tasks_pub = rospy.Publisher("/assign_tasks", String, queue_size=10)
        self.assign_tasks_timer = rospy.Timer(rospy.Duration(2), self._assign_tasks_cb)
        self.log_dir = rospy.get_param("~log_dir")
        self.task_db = TaskDatabase()
        self.has_robot_state = False
        self.task_update_lock = False
        self.global_id = 0
        self.global_color_idx = 0

        # scripts
        self.scripts_buffer = []
        self.start_play_pub = rospy.Publisher("/start_play_script", Empty, queue_size=1)
        self.scripts_sub = rospy.Subscriber("/script", String, self._script_cb)
        self.handle_script_timer = rospy.Timer(
            rospy.Duration(3), self._handle_script_cb
        )

        # visualization
        self.gantt_server = rospy.Service("/robots_gantt", Trigger, self._gantt_cb)
        self.vis_pub = rospy.Publisher("/extended_vis", Marker, queue_size=50)
        self.robot_overlay_pub = rospy.Publisher(
            "/robot_overlay_text", OverlayText, queue_size=10
        )
        self.task_overlay_pub = rospy.Publisher(
            "/task_overlay_text", OverlayText, queue_size=10
        )
        self.map_name = rospy.get_param("~map_name")
        for _ in range(3):
            self.vis_pub.publish(MF.gen_delete_marker())
            rospy.sleep(0.2)
        self.vis_timer = rospy.Timer(rospy.Duration(1), self._vis_cb)

    def _gantt_cb(self, req: TriggerRequest) -> TriggerResponse:
        if not self.has_robot_state:
            return TriggerResponse(success=False, message="No robot state available")
        else:
            gantt_dict = self.task_db.gen_full_gantt_chart()
            gantt_json = json.dumps(gantt_dict)
            return TriggerResponse(success=True, message=gantt_json)

    def _pub_overlay(self):
        robot_overlay_dict = {}
        for robot_id, robot in self.task_db.robots.items():
            robot_overlay_dict[robot_id] = f"({robot.state}): "
            for task_id in self.task_db.get_robot_schedule(robot_id, ("doing")):
                task = self.task_db.sub_task_instances[task_id]
                robot_overlay_dict[robot_id] += f"{task.action}"
        self.robot_overlay_pub.publish(MF.gen_robot_overlay(robot_overlay_dict))

        all_tasks = self.task_db.get_task_ids(("todo", "doing", "done"))
        todo_tasks = self.task_db.get_task_ids(("todo"))
        done_tasks = self.task_db.get_task_ids(("done"))
        task_overlay_list: List[Tuple[str, str]] = []
        for task_id in all_tasks:
            task = self.task_db.task_instances[task_id]
            if task_id in todo_tasks:
                state = "Todo"
            elif task_id in done_tasks:
                state = "Done"
            else:
                state = "In Progress"
            task_overlay_list.append((task.description, state, task.color))
        self.task_overlay_pub.publish(MF.gen_task_overlay(task_overlay_list))

    def _vis_cb(self, event):
        objs = copy.deepcopy(self.task_db.objects)
        for obj_id, obj in objs.items():
            obj_marker, text_marker = MF.gen_obj_marker(obj_id, obj)
            self.vis_pub.publish(obj_marker)
        # legends
        if self.map_name == "Ice":
            legends = {
                10000: WorldObject("{algae}", "", (-20, 30, 1), "active"),
                10001: WorldObject("{injured_animal}", "", (-10, 30, 1), "active"),
                10002: WorldObject("{rescue_station}", "", (0, 30, 1), "active"),
                10003: WorldObject("{snow_heap}", "", (10, 30, 1), "active"),
                10004: WorldObject("{expedition_base}", "", (20, 30, 1), "active"),
            }
        elif self.map_name == "Factory":
            legends = {
                10000: WorldObject("{flame}", "", (-25, 22, 1), "active"),
                10001: WorldObject("{injured_person}", "", (-15, 22, 1), "active"),
                10002: WorldObject("{rescue_station}", "", (-5, 22, 1), "active"),
                10003: WorldObject("{water_source}", "", (5, 22, 1), "active"),
                10004: WorldObject("{sand_heap}", "", (15, 22, 1), "active"),
                10005: WorldObject("{fire_extinguisher}", "", (25, 22, 1), "active"),
            }
        for obj_id, obj in legends.items():
            obj_marker, text_marker = MF.gen_obj_marker(obj_id, obj)
            self.vis_pub.publish(obj_marker)
            self.vis_pub.publish(text_marker)
        self._pub_overlay()

    def _handle_script_cb(self, event):
        if not self.has_robot_state or self.task_update_lock or not self.scripts_buffer:
            return
        msg: str = self.scripts_buffer.pop(0)
        script = msg.split(": ")
        if "task" in script[0]:
            if script[0].startswith("task-p"):
                priority = int(script[0].split("-p")[1])
            else:
                priority = 0
            task_type = script[1]
            remap_strs = script[2].split(" & ")
            remaps = {}
            for remap_str in remap_strs:
                remap = remap_str.split("=")
                location = tuple(map(float, remap[1].split(",")))
                remaps[remap[0]] = location
            self.task_input(task_type, remaps, priority)
        elif script[0] == "resource":
            resource_description = script[1]
            remap = script[2].split("=")
            resource_type = remap[0]
            location = tuple(map(float, remap[1].split(",")))
            self.resource_input(resource_description, resource_type, location)

    def _script_cb(self, msg: String):
        self.scripts_buffer.append(msg.data)

    def task_input(self, task_type: str, remaps: Dict[str, tuple], priority: int = 0):
        """
        @param task_type: extinguish {flame}
        @param remaps: {'{flame}':(1,1,1), '{stone}':(2,2,2)}
        """
        self.task_update_lock = True
        # 1. 判断任务类型是否存在
        is_exist: bool = self.task_db.check_task_type_existence(task_type)

        # 1.1 如果任务类型不存在，则是新的任务，需要通过大模型生成方案
        if not is_exist:
            # 1.2 生成任务类型
            self.task_db.task_types[self.global_id] = TaskType(task_type, [])
            # 1.3 生成方案
            self.gen_scheme([self.global_id])
            self.global_id += 1

        # 2. 不论是不是新任务，都需要生成任务实例

        # 2.1 生成抽象任务实例
        color = MF.id_2_color_10_light(self.global_color_idx)
        self.global_color_idx += 1
        self.task_db.task_instances[self.global_id] = TaskInstance(
            task_type,
            {},
            [],
            self.task_db.get_task_ids(("todo", "doing")),
            priority,
            color,
        )
        task_id = self.global_id
        self.global_id += 1

        # 2.2 生成 object 实例并加入 remap 到任务实例中
        for obj_type, location in remaps.items():
            self.task_db.objects[self.global_id] = WorldObject(
                obj_type, obj_type, location, "active"
            )
            self.task_db.task_instances[task_id].remaps[obj_type] = self.global_id
            self.global_id += 1

        # 3. 为抽象任务实例分配子任务实例序列
        self.alloc_task(task_id)

        if priority > 0:
            # 4. 更新所有 todo 状态的任务实例，考虑新任务的优先级
            todo_task_ids = self.task_db.get_task_ids(("todo"))
            for task_id in todo_task_ids:
                task_instance = self.task_db.task_instances[task_id]
                # 4.1 删除任务实例的所有子任务实例
                for sub_task_instance_id in task_instance.sub_task_instances:
                    del self.task_db.sub_task_instances[sub_task_instance_id]
                task_instance.sub_task_instances.clear()
                task_instance.start_after.clear()
            # 4.2 按照优先级顺序重新分配子任务实例
            todo_task_ids = sorted(
                todo_task_ids,
                key=lambda x: self.task_db.task_instances[x].priority,
                reverse=True,
            )
            for task_id in todo_task_ids:
                task_instance = self.task_db.task_instances[task_id]
                task_instance.start_after = self.task_db.get_task_ids(("todo", "doing"))
                self.alloc_task(task_id)

        self.task_db.log(self.log_dir)
        self.task_update_lock = False

    def resource_input(self, description: str, object_type: str, location: tuple):
        """
        @param description: We have detected a {rescue stations} to save injured person
        @param object_type: {rescue stations}
        @param location: (1,1,1)
        """
        self.task_update_lock = True
        # 1. 判断资源类型是否存在
        is_exist: bool = self.task_db.check_object_type_existence(object_type)

        # 2. 不论是不是新资源，都需要生成 object 实例
        self.task_db.objects[self.global_id] = WorldObject(
            object_type, description, location, "active"
        )
        self.global_id += 1

        # 3. 如果资源类型不存在，则是新的资源，需要通过大模型对所有任务类型生成新的方案
        if not is_exist:
            self.gen_scheme([key for key in self.task_db.task_types.keys()])

        # 4. 更新所有 todo 状态的任务实例，应用新的资源和方案
        todo_task_ids = self.task_db.get_task_ids(("todo"))
        for task_id in todo_task_ids:
            task_instance = self.task_db.task_instances[task_id]
            # 4.1 删除任务实例的所有子任务实例
            for sub_task_instance_id in task_instance.sub_task_instances:
                del self.task_db.sub_task_instances[sub_task_instance_id]
            task_instance.sub_task_instances.clear()
            task_instance.start_after.clear()
        # 4.2 按照优先级顺序重新分配子任务实例
        todo_task_ids = sorted(
            todo_task_ids,
            key=lambda x: self.task_db.task_instances[x].priority,
            reverse=True,
        )
        for task_id in todo_task_ids:
            task_instance = self.task_db.task_instances[task_id]
            task_instance.start_after = self.task_db.get_task_ids(("todo", "doing"))
            self.alloc_task(task_id)
        self.task_db.log(self.log_dir)
        self.task_update_lock = False

    def gen_scheme(self, task_type_ids: List[int]):
        """通过大模型生成子任务类型序列，整合为多个任务类型"""
        for task_type_id in task_type_ids:
            # 1. 生成候选子任务类型序列
            sub_task_sequences = self._call_llm(task_type_id)
            self.task_db.task_types[task_type_id].sub_task_types_seqs.clear()

            # 2. 对于每一个候选序列，生成并存储子任务类型
            for sub_task_sequence in sub_task_sequences:
                sub_task_types = []
                temp_id_remap = {"None": "None"}
                for sub_task_id, sub_task in sub_task_sequence.items():
                    self.task_db.sub_task_types[self.global_id] = SubTaskType(
                        task_type_id,
                        sub_task["robot_type"],
                        sub_task["action"],
                        sub_task["action"].split(" ")[1],
                        [
                            temp_id_remap[local_id]
                            for local_id in sub_task["dependencies"]
                        ],
                        temp_id_remap[sub_task["done_by_same_robot_as"]],
                    )
                    temp_id_remap[sub_task_id] = self.global_id
                    sub_task_types.append(self.global_id)
                    self.global_id += 1
                self.task_db.task_types[task_type_id].sub_task_types_seqs.append(
                    sub_task_types
                )
        self.task_db.log(self.log_dir)

    def alloc_task(self, task_id: int):
        """根据环境资源、remap和方案, 为抽象任务实例分配子任务实例序列"""
        # 1. 获取抽象任务实例并找到对应的任务类型
        task_instance = copy.deepcopy(self.task_db.task_instances[task_id])
        del self.task_db.task_instances[task_id]
        scheme_id, scheme = self.task_db.get_scheme(task_instance.task_type)
        scheme: TaskType

        # 2. 根据排列组合生成临时任务实例和子任务实例序列
        temp_task_instances: Dict[int, TaskInstance] = {}
        temp_sub_task_instances: Dict[int, SubTaskInstance] = {}

        # 2.1 先根据方案创建对应的多个任务实例和子任务实例
        for sub_task_types_seq in scheme.sub_task_types_seqs:
            task_id = self.global_id
            self.global_id += 1
            temp_task_instances[task_id] = TaskInstance(
                task_instance.task_type,
                task_instance.remaps,
                [],
                task_instance.start_after,
                task_instance.priority,
                task_instance.color,
            )
            temp_id_remap = {"None": "None"}
            for sub_task_type_id in sub_task_types_seq:
                sub_task_type = self.task_db.sub_task_types[sub_task_type_id]

                sub_task_instance_id = self.global_id
                self.global_id += 1

                temp_id_remap[sub_task_type_id] = sub_task_instance_id
                temp_sub_task_instances[sub_task_instance_id] = SubTaskInstance(
                    task_id,
                    sub_task_type.required_robot_type,
                    "",
                    sub_task_type.action,
                    sub_task_type.target_type,
                    [temp_id_remap[type_id] for type_id in sub_task_type.dependencies],
                    temp_id_remap[sub_task_type.done_by_same_robot_as],
                    "todo",
                )
                temp_task_instances[task_id].sub_task_instances.append(
                    sub_task_instance_id
                )

        # 2.2 根据资源和 remap 为子任务实例分配资源，进行排列组合，生成更多子任务实例
        for task_id, task_instance in temp_task_instances.items():
            for sub_task_instance_id in task_instance.sub_task_instances:
                sub_task_instance = temp_sub_task_instances[sub_task_instance_id]
                target_type = sub_task_instance.target

                if target_type in task_instance.remaps.keys():
                    # 2.2.1 先查找 remap 中是否有对应的 target_type
                    # Task 的执行对象必须在 remap 中，例如 {flame}
                    sub_task_instance.target = task_instance.remaps[target_type]
                else:
                    # 2.2.2 如果没有，查找环境资源中对应的 target_type
                    obj_ids = self.task_db.get_active_object_ids(target_type)
                    if len(obj_ids) == 0:
                        rospy.logerr(
                            f"Skip task allocation: no active object of type {target_type} available"
                        )
                        return
                    elif len(obj_ids) == 1:  # 只有一个，直接分配
                        sub_task_instance.target = obj_ids[0]  # type: ignore
                    else:  # 多个，先存为列表
                        sub_task_instance.target = obj_ids  # type: ignore

        # 2.2.3 进行排列组合，生成所有的任务和子任务实例
        # Expand task instances by handling subtasks whose target is a list.
        # 对每一个原始的task instance，根据各subtaskinstance target（可能为list）生成所有组合，
        # 每一种组合对应一个新的task instance（以及对应的新的subtask instances），并更新global id。
        new_temp_task_instances = {}
        new_temp_sub_task_instances = {}

        for orig_task_id, task_instance in temp_task_instances.items():
            subtask_ids = task_instance.sub_task_instances

            # 递归生成所有组合：对每个subtask instance，若其target为list，则枚举每个可能值，
            # 否则就直接取当前target。
            assignments = []

            def helper(idx: int, current_assignment: Dict):
                if idx == len(subtask_ids):
                    assignments.append(current_assignment.copy())
                    return
                stid = subtask_ids[idx]
                sub_task = temp_sub_task_instances[stid]
                if isinstance(sub_task.target, list):
                    for option in sub_task.target:
                        current_assignment[stid] = option
                        helper(idx + 1, current_assignment)
                else:
                    current_assignment[stid] = sub_task.target
                    helper(idx + 1, current_assignment)

            helper(0, {})

            # 对于每一种组合，复制task instance和对应的subtask instances，更新target为组合选择的值
            for assign in assignments:
                new_task_id = self.global_id
                self.global_id += 1
                # 复制task instance，注意这里复制remaps以防引用问题
                new_task_instance = TaskInstance(
                    task_instance.task_type,
                    task_instance.remaps.copy(),
                    [],
                    task_instance.start_after,
                    task_instance.priority,
                    task_instance.color,
                )
                new_id_map = {"None": "None"}
                for old_stid in subtask_ids:
                    old_sub_task = temp_sub_task_instances[old_stid]
                    new_stid = self.global_id
                    self.global_id += 1
                    new_id_map[old_stid] = new_stid
                    # 构造新的subtask instance，更新target为assign选中的值
                    new_target = assign[old_stid]
                    new_sub_task = SubTaskInstance(
                        new_task_id,
                        old_sub_task.required_robot_type,
                        old_sub_task.allocated_robot,
                        old_sub_task.action,
                        new_target,
                        [new_id_map[dep] for dep in old_sub_task.dependencies],
                        new_id_map[old_sub_task.done_by_same_robot_as],
                        old_sub_task.state,
                    )
                    new_task_instance.sub_task_instances.append(new_stid)
                    new_temp_sub_task_instances[new_stid] = new_sub_task
                new_temp_task_instances[new_task_id] = new_task_instance

        # 替换原来的临时字典，用于后续MILP求解的调度
        temp_task_instances.clear()
        temp_task_instances.update(new_temp_task_instances)
        temp_sub_task_instances.clear()
        temp_sub_task_instances.update(new_temp_sub_task_instances)

        # 3. 对每一个任务实例进行求解，选出最优方案
        robot_state = self.task_db.get_all_robot_state_for_milp()
        candidate_alloc_solutions: List[Dict] = []
        for task_id, task_instance in temp_task_instances.items():

            # 3.1 构建 MILP 求解器需要的数据结构
            meta_tasks = []
            task_dependencies = []
            task_consistency = []
            task_simultaneity = []  # ignore
            task_exclusivity = []  # ignore
            for sub_task_instance_id in task_instance.sub_task_instances:
                sub_task_instance = temp_sub_task_instances[sub_task_instance_id]
                meta_task = {
                    "id": sub_task_instance_id,
                    "location": self.task_db.objects[sub_task_instance.target].location,
                    "required_robot_type": sub_task_instance.required_robot_type,
                    "action": sub_task_instance.action,
                    "duration": sub_task_instance.duration,
                    "belong_to": task_id,
                }
                meta_tasks.append(meta_task)
                task_dependencies.extend(
                    [
                        [dep, sub_task_instance_id]
                        for dep in sub_task_instance.dependencies
                    ]
                )
                if sub_task_instance.done_by_same_robot_as != "None":
                    task_consistency.append(
                        [sub_task_instance_id, sub_task_instance.done_by_same_robot_as]
                    )

            # 3.2 调用 MILP 求解器
            self.task_alloc_client.wait_for_service()
            resp: TaskAllocSrvResponse = self.task_alloc_client(
                json.dumps(
                    {
                        "meta_tasks": meta_tasks,
                        "robots": robot_state,
                        "task_dependencies": task_dependencies,
                        "task_consistency": task_consistency,
                        "task_simultaneity": task_simultaneity,
                        "task_exclusivity": task_exclusivity,
                    }
                )
            )
            candidate_alloc_solutions.append(json.loads(resp.ta_solution_json))

        # 3.3 选出最优方案并更新任务实例到数据库
        best_solution = min(candidate_alloc_solutions, key=lambda x: x["T_max"])
        best_task_id = None
        for robot_id, meta_tasks in best_solution["robot_schedule"].items():
            last_meta_task = None
            for m in meta_tasks:
                sub_task_instance = temp_sub_task_instances[m["id"]]
                sub_task_instance.allocated_robot = int(robot_id)
                best_task_id = m["belong_to"]
                if last_meta_task:
                    sub_task_instance.dependencies.append(last_meta_task)
                last_meta_task = m["id"]
        if best_task_id is None:
            rospy.logerr("No valid task allocation solution found")
            return
        best_task = temp_task_instances[best_task_id]
        self.task_db.task_instances[best_task_id] = best_task
        for sub_task_instance_id in best_task.sub_task_instances:
            self.task_db.sub_task_instances[sub_task_instance_id] = (
                temp_sub_task_instances[sub_task_instance_id]
            )

    def _robot_state_cb(self, msg: String):
        """
        Robot state message format:
        [
            {"id": 0, "type": "1", "initial_location": [0, 10], "speed": 1.0, "state": "idle"},
            {"id": 1, "type": "1", "initial_location": [-10, 10], "speed": 1.0, "state": "idle"},
        ]
        """
        msg = json.loads(msg.data)
        for robot in msg:
            if int(robot["id"]) not in self.task_db.robots:
                self.task_db.robots[int(robot["id"])] = WorldRobot(
                    robot["type"],
                    robot["initial_location"],
                    robot["state"],
                    robot["speed"],
                )
            else:
                world_robot = self.task_db.robots[int(robot["id"])]
                world_robot.location = robot["initial_location"]
                world_robot.state = robot["state"]
        self.task_db.log(self.log_dir)
        if not self.has_robot_state:
            self.start_play_pub.publish(Empty())
            self.global_start_time = rospy.Time.now().to_sec()
            self.task_db.global_start_time = self.global_start_time
        self.has_robot_state = True

    def _assign_tasks_cb(self, event):
        if not self.has_robot_state or self.task_update_lock:
            return
        for robot_id, robot in self.task_db.robots.items():
            if robot.state == "idle":
                todo_and_doing_tasks = self.task_db.get_robot_schedule(robot_id)
                if todo_and_doing_tasks:
                    next_task = self.task_db.sub_task_instances[todo_and_doing_tasks[0]]
                    if next_task.state == "todo" and self.task_db.check_dependencies(
                        todo_and_doing_tasks[0]
                    ):
                        pub_dict = {
                            str(robot_id): [
                                {
                                    "id": todo_and_doing_tasks[0],
                                    "location": self.task_db.objects[
                                        next_task.target
                                    ].location,
                                    "action": next_task.action,
                                    "duration": next_task.duration,
                                }
                            ]
                        }
                        self.assign_tasks_pub.publish(json.dumps(pub_dict))
                        next_task.state = "doing"

                        # predict travel time
                        start = self.task_db.robots[robot_id].location
                        goal = self.task_db.objects[next_task.target].location
                        distance = (
                            (start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2
                        ) ** 0.5
                        travel_time = distance / self.task_db.robots[robot_id].speed

                        next_task.start_time = (
                            rospy.Time.now().to_sec()
                            - self.global_start_time
                            + travel_time
                        )
                    elif next_task.state == "doing":
                        next_task.state = "done"
                        # set object state
                        target: WorldObject = self.task_db.objects[next_task.target]
                        if target.object_type in (
                            "{flame}",
                            "{injured_person}",
                            "{algae}",
                            "{injured_animal}",
                        ):
                            target.state = "inactive"
        self.task_db.log(self.log_dir)

    def _call_llm(self, task_type_id) -> List[Dict]:
        self.task_alloc_client.wait_for_service()
        task_gen_resp: TaskGenSrvResponse = self.task_gen_client(
            json.dumps(
                {
                    "Task": self.task_db.task_types[task_type_id].task_type,
                    "Resources": self.task_db.get_resources_description(),
                }
            )
        )
        llm_result = json.loads(task_gen_resp.tg_result_json)
        candidate_sub_task_sequences: Dict = llm_result["Candidate Sub-Task Sequences"]
        return [seq for seq in candidate_sub_task_sequences.values()]


if __name__ == "__main__":
    rospy.init_node("main_manager")
    TaskManager()
    ScriptsPlayer()
    rospy.spin()
