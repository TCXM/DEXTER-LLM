from dataclasses import dataclass
from typing import Dict, List
from collections import defaultdict, deque
import rospy
from marker_factory import MarkerFactory as MF


@dataclass
class WorldObject:
    object_type: str
    description: str  # description of the object
    location: tuple  # (x, y, z)
    state: str  # state of the object, e.g. active, inactive


@dataclass
class WorldRobot:
    robot_type: str
    location: tuple  # (x, y, z)
    state: str  # state of the robot, e.g. idle, moving, performing
    speed: float  # speed of the robot


@dataclass
class SubTaskInstance:
    parent_task: int  # parent task id
    required_robot_type: str  # robot type required to perform the task
    allocated_robot: int  # robot allocated to perform the task
    action: str  # action to be performed
    target: str  # target of the action, should be the key of an object
    dependencies: list  # list of sub task instance ids that need to be completed before this task can be started
    done_by_same_robot_as: str  # sub task instance that must be done by the same robot
    state: str  # state of the task, e.g. todo, in_progress, done
    start_time: float = -1.0  # time when the task is started
    duration: float = 10.0  # duration of the task


@dataclass
class TaskInstance:
    task_type: str  # type of the task, e.g. extinguish {flame}
    remaps: dict  # local remap, e.g. {flame}=world_object_id
    sub_task_instances: List[int]  # list of keys to sub task instances
    start_after: List[int]  # task instances that must be start before this task
    priority: int = 0  # priority of the task, higher value means higher priority
    color: str = "#000000"  # color of the task in the gantt chart

    @property
    def description(self):
        task_description = self.task_type
        for key, value in self.remaps.items():
            task_description = task_description.replace(key, f"{key[1:-1]}_{value}")
        if self.priority > 0:
            task_description = f"{task_description} (serious)"
        return task_description


@dataclass
class SubTaskType:
    parent_task_type: str  # id of parent task type
    required_robot_type: str  # robot type required to perform the task
    action: str  # action to be performed
    target_type: str  # type of the target, should be an object type
    dependencies: list  # list of sub task types ids that need to be completed before this task can be started
    done_by_same_robot_as: int  # sub task type that must be done by the same robot


@dataclass
class TaskType:
    task_type: str  # type of the task, e.g. extinguish {flame}
    sub_task_types_seqs: List[List[int]]  # list of seq of keys to sub task types


class TaskDatabase:
    def __init__(self):
        self.objects: Dict[int, WorldObject] = {}
        self.robots: Dict[int, WorldRobot] = {}
        self.sub_task_instances: Dict[int, SubTaskInstance] = {}
        self.task_instances: Dict[int, TaskInstance] = {}
        self.sub_task_types: Dict[int, SubTaskType] = {}
        self.task_types: Dict[int, TaskType] = {}
        self.global_start_time: float = -1.0

    def get_task_ids(self, state: tuple = ("todo")) -> List[int]:
        return [
            task_id
            for task_id, task in self.task_instances.items()
            if task.sub_task_instances
            and all(
                self.sub_task_instances[sub_task_id].state in state
                for sub_task_id in task.sub_task_instances
            )
        ]

    def check_dependencies(self, sub_task_instance_id: int) -> bool:
        sub_task_instance = self.sub_task_instances[sub_task_instance_id]
        return all(
            self.sub_task_instances[dependency].state == "done"
            for dependency in sub_task_instance.dependencies
        )

    def check_task_type_existence(self, task_type: str) -> bool:
        for task in self.task_types.values():
            task: TaskType
            if task.task_type == task_type:
                return True
        return False

    def check_object_type_existence(self, object_type: str) -> bool:
        for obj in self.objects.values():
            obj: WorldObject
            if obj.object_type == object_type:
                return True
        return False

    def get_resources_description(self) -> List[str]:
        """Get the description of all object types for LLM input"""
        resources_list = []
        for obj in self.objects.values():
            obj: WorldObject
            if obj.state == "active" and obj.description not in resources_list:
                resources_list.append(obj.description)
        return resources_list

    def get_scheme(self, task_type: str) -> int:
        """Get the scheme for a task type"""
        for scheme_id, scheme in self.task_types.items():
            scheme: TaskType
            if scheme.task_type == task_type:
                return scheme_id, scheme
        raise ValueError(f"Task type {task_type} does not exist in the database")

    def get_active_object_ids(self, object_type: str) -> List[int]:
        """Get the object id for an object type"""
        object_ids = []
        for obj_id, obj in self.objects.items():
            obj: WorldObject
            if obj.object_type == object_type and obj.state == "active":
                object_ids.append(obj_id)
        return object_ids

    def get_all_robot_state_for_milp(self) -> List[Dict]:
        """Get the state of all robots for LLM input"""
        robot_state = []
        for robot_id, robot in self.robots.items():
            robot: WorldRobot
            robot_state.append(
                {
                    "id": robot_id,
                    "type": robot.robot_type,
                    "initial_location": self.predict_robot_free_location(robot_id),
                    "speed": robot.speed,
                    "free_time": self.predict_robot_free_time(robot_id),
                }
            )
        return robot_state

    def get_robot_schedule(self, robot_id: int, state=("todo", "doing")) -> List[int]:
        """Get the schedule of a robot"""
        # Create a graph and in-degree count for the sub-tasks
        graph = defaultdict(list)
        in_degree = defaultdict(int)
        robot_sub_task_ids = []

        # Find all sub-tasks assigned to the robot
        for sub_task_id, sub_task in self.sub_task_instances.items():
            if sub_task.allocated_robot == robot_id and sub_task.state in state:
                robot_sub_task_ids.append(sub_task_id)
        for sub_task_id in robot_sub_task_ids:
            sub_task = self.sub_task_instances[sub_task_id]
            for dependency in sub_task.dependencies:
                if dependency in robot_sub_task_ids:
                    graph[dependency].append(sub_task_id)
                    in_degree[sub_task_id] += 1
            start_after_task_ids = self.task_instances[sub_task.parent_task].start_after
            for robot_sub_task_id in robot_sub_task_ids:
                parent_task_id = self.sub_task_instances[robot_sub_task_id].parent_task
                # 如果有子任务的父任务需要在当前任务之前，那么作为依赖加入
                if parent_task_id in start_after_task_ids:
                    graph[robot_sub_task_id].append(sub_task_id)
                    in_degree[sub_task_id] += 1

        # Topological sort (Kahn's algorithm)
        queue = deque([task for task in robot_sub_task_ids if in_degree[task] == 0])
        sorted_tasks = []

        while queue:
            current = queue.popleft()
            sorted_tasks.append(current)
            for neighbor in graph[current]:
                in_degree[neighbor] -= 1
                if in_degree[neighbor] == 0:
                    queue.append(neighbor)

        return sorted_tasks

    def predict_robot_free_time(self, robot_id: int) -> float:
        """Predict the free time of a robot after all todo tasks are done"""
        predicted_schedule = self.gen_robot_gantt_chart(robot_id)
        if predicted_schedule:
            current_time = max(entry["end"] for entry in predicted_schedule)
        else:
            current_time = rospy.Time.now().to_sec() - self.global_start_time
        return current_time

    def predict_robot_free_location(self, robot_id: int) -> tuple:
        """Predict the free location of a robot after all todo tasks are done"""
        robot_schedule = self.get_robot_schedule(robot_id)
        if not robot_schedule:
            return self.robots[robot_id].location
        last_sub_task = self.sub_task_instances[robot_schedule[-1]]
        last_sub_task: SubTaskInstance
        target = self.objects[last_sub_task.target]
        return target.location

    def gen_robot_gantt_chart(self, robot_id: int, check_dep=True):
        past_robot_schedule = self.get_robot_schedule(robot_id, ("doing", "done"))
        past_robot_schedule.sort(key=lambda sub_task_id: self.sub_task_instances[sub_task_id].start_time)
        todo_robot_schedule = self.get_robot_schedule(robot_id, ("todo"))
        robot = self.robots[robot_id]
        current_time = rospy.Time.now().to_sec() - self.global_start_time
        current_pos = robot.location
        predicted_robot_schedule = []
        for sub_task_id in past_robot_schedule:
            sub_task = self.sub_task_instances[sub_task_id]
            sub_task: SubTaskInstance
            task = self.task_instances[sub_task.parent_task]
            task: TaskInstance
            sub_task_concise = sub_task.action.split("_")[0][0].upper() + " " + sub_task.action.split("_")[1][0].upper()
            predicted_robot_schedule.append(
                {
                    "start": sub_task.start_time,
                    "end": sub_task.start_time + sub_task.duration,
                    "sub_task": sub_task_concise,
                    "task": task.description,
                    "robot": sub_task.allocated_robot,
                    "parent_task": sub_task.parent_task,
                    "id": sub_task_id,
                    "priority": task.priority,
                    "color": task.color
                }
            )
            current_pos = self.objects[sub_task.target].location
            current_time = sub_task.start_time + sub_task.duration
        if current_time < rospy.Time.now().to_sec() - self.global_start_time:
            current_time = rospy.Time.now().to_sec() - self.global_start_time
        for sub_task_id in todo_robot_schedule:
            sub_task = self.sub_task_instances[sub_task_id]
            sub_task: SubTaskInstance
            task = self.task_instances[sub_task.parent_task]
            task: TaskInstance
            target = self.objects[sub_task.target]
            distance = (
                sum((x - y) ** 2 for x, y in zip(current_pos, target.location))
                ** 0.5
            )
            time_to_move = distance / robot.speed
            current_time += time_to_move
            if check_dep:
                # check dependencies
                for dep in sub_task.dependencies:
                    sub_task_dep = self.sub_task_instances[dep]
                    if sub_task_dep.allocated_robot == robot_id:
                        continue
                    if sub_task_dep.state in ("done", "doing"):
                        end_time = sub_task_dep.start_time + sub_task_dep.duration
                        if end_time > current_time:
                            current_time = end_time
                    else:
                        predicted_dep_robot_schedule = self.gen_robot_gantt_chart(sub_task_dep.allocated_robot, False)
                        dep_schedule = next((t for t in predicted_dep_robot_schedule if t["id"] == dep), None)
                        if dep_schedule and dep_schedule["end"] > current_time:
                            current_time = dep_schedule["end"]
                
            sub_task_concise = sub_task.action.split("_")[0][0].upper() + " " + sub_task.action.split("_")[1][0].upper()
            predicted_robot_schedule.append(
                {
                    "start": current_time,
                    "end": current_time + sub_task.duration,
                    "sub_task": sub_task_concise,
                    "task": task.description,
                    "robot": sub_task.allocated_robot,
                    "parent_task": sub_task.parent_task,
                    "id": sub_task_id,
                    "priority": task.priority,
                    "color": task.color
                }
            )
            current_pos = target.location
            current_time += sub_task.duration
        return predicted_robot_schedule

    def gen_full_gantt_chart(self):
        gantt_chart = []
        for robot_id in self.robots:
            gantt_chart.extend(self.gen_robot_gantt_chart(robot_id))

        for _ in range(len(gantt_chart)):
            sub_task = gantt_chart.pop(0)
            robot_id = sub_task["robot"]
            robot = self.robots[robot_id]
            transfromed_sub_task = {
                "start": sub_task["start"],
                "end": sub_task["end"],
                "label": f"{robot_id}: {robot.robot_type}",
                "legend": sub_task["task"],
                "name": sub_task["sub_task"],
                "name_color": "#bd0000" if sub_task["priority"] > 0 else "black",
                "color": sub_task["color"],
                "label_color": MF.id_2_color_8(robot_id),
                "hatch": "///" if "injured" in sub_task["task"] else "",
            }
            gantt_chart.append(transfromed_sub_task)

        for robot_id in self.robots:
            robot = self.robots[robot_id]
            init_task = {
                "start": 0,
                "end": 0.1,
                "label": f"{robot_id}: {robot.robot_type}",
                "name": "",
                "label_color": MF.id_2_color_8(robot_id)
            }
            gantt_chart.append(init_task)
        for _ in range(len(self.robots)):
            gantt_chart.insert(0, gantt_chart.pop())
            
        current_time = rospy.Time.now().to_sec() - self.global_start_time
        max_end = max(current_time, max(task["end"] for task in gantt_chart))

        # Define the number of ticks you want on the x-axis
        num_ticks = 10
        # Avoid division by zero, if no tasks are scheduled or max_end is zero
        interval = max_end / (num_ticks - 1) if max_end > 0 else 1
        xticks = [round(i * interval, 2) for i in range(num_ticks)]

        gantt_chart_dict = {
            "current_time": current_time,
            "packages": gantt_chart,
            "title": "Robot Schedule",
            "xticks": xticks,
        }
        return gantt_chart_dict

    def log(self, log_dir: str):
        with open(f"{log_dir}/db_objects_log.txt", "w") as f:
            f.truncate(0)
            for id, obj in self.objects.items():
                f.write(f"{id}: {obj}\n")

        with open(f"{log_dir}/db_robots_log.txt", "w") as f:
            f.truncate(0)
            for id, robot in self.robots.items():
                f.write(f"{id}: {robot}\n")

        with open(f"{log_dir}/db_sub_task_instances_log.txt", "w") as f:
            f.truncate(0)
            for id, sub_task_instance in self.sub_task_instances.items():
                f.write(f"{id}: {sub_task_instance}\n")

        with open(f"{log_dir}/db_task_instances_log.txt", "w") as f:
            f.truncate(0)
            for id, task_instance in self.task_instances.items():
                f.write(f"{id}: {task_instance}\n")

        with open(f"{log_dir}/db_sub_task_types_log.txt", "w") as f:
            f.truncate(0)
            for id, sub_task_type in self.sub_task_types.items():
                f.write(f"{id}: {sub_task_type}\n")

        with open(f"{log_dir}/db_task_types_log.txt", "w") as f:
            f.truncate(0)
            for id, task_type in self.task_types.items():
                f.write(f"{id}: {task_type}\n")

        with open(f"{log_dir}/AAA_robot_shedule_log.txt", "w") as f:
            f.truncate(0)
            for robot_id in self.robots:
                robot_state = self.robots[robot_id].state
                f.write(
                    f"{robot_id}: ({robot_state}): {self.get_robot_schedule(robot_id)}\n"
                )


if __name__ == "__main__":
    task_db = TaskDatabase()

    task_db.objects[1] = WorldObject("fire", "A blazing fire", (0, 0, 0), "active")
    task_db.objects[2] = WorldObject("water", "A pool of water", (1, 1, 1), "inactive")

    task_db.robots[1] = WorldRobot("firefighter", (0, 0, 0), "idle", 1.0)
    task_db.robots[2] = WorldRobot("watercarrier", (1, 1, 1), "moving", 0.5)

    task_db.sub_task_instances[1] = SubTaskInstance(
        "task1", "firefighter", 1, "extinguish", 1, [], None, "todo"
    )
    task_db.sub_task_instances[2] = SubTaskInstance(
        "task1", "watercarrier", 1, "fetch", 2, [3], None, "todo"
    )
    task_db.sub_task_instances[3] = SubTaskInstance(
        "task1", "watercarrier", 1, "fetch", 2, [1], None, "todo"
    )
    task_db.sub_task_instances[4] = SubTaskInstance(
        "task1", "watercarrier", 2, "fetch", 2, [5], None, "todo"
    )
    task_db.sub_task_instances[5] = SubTaskInstance(
        "task1", "watercarrier", 2, "fetch", 2, [1], None, "todo"
    )

    task_db.task_instances[1] = TaskInstance("extinguish {fire}", {"fire": 1}, [1, 2])

    task_db.sub_task_types[1] = SubTaskType(
        "extinguish {fire}", "firefighter", "extinguish", "fire", [], None
    )
    task_db.sub_task_types[2] = SubTaskType(
        "extinguish {fire}", "watercarrier", "fetch", "water", [1], None
    )

    task_db.task_types[1] = TaskType("extinguish {fire}", [1, 2])

    print(task_db.get_task_ids())
    print(task_db.check_dependencies(2))
    print(task_db.check_task_type_existence("extinguish {fire}"))
    print(task_db.check_object_type_existence("fire"))
    print(task_db.get_resources_description())
    print(task_db.get_scheme("extinguish {fire}"))
    print(task_db.get_active_object_ids("fire"))
    print(task_db.get_all_robot_state_for_milp())
    print(task_db.get_robot_schedule(1))
    print(task_db.get_robot_schedule(2))
    # print(task_db.predict_robot_free_time(1))
