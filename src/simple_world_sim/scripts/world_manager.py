#!/usr/bin/env python3
import rospy
from world_classes import *
from std_msgs.msg import String
from typing import Dict
import json


class WorldManager:
    def __init__(self, world_config_file, odom_topic="odom"):
        self.world_objects: Dict[int, WorldObject] = {}
        self.world_agents: Dict[int, WorldAgent] = {}

        self.detect_region_sub = rospy.Subscriber(
            "/detect_region", String, self._detect_region_cb
        )
        self.assign_tasks_sub = rospy.Subscriber(
            "/assign_tasks", String, self._assign_tasks_cb
        )
        self.action_sub = rospy.Subscriber("/action", String, self._action_cb)
        self.vis_pub = rospy.Publisher("/extended_vis", Marker, queue_size=10)

        self.feedback_pub = rospy.Publisher("/simulator_feedback", String, queue_size=1)
        self.robot_state_pub = rospy.Publisher("/robot_state", String, queue_size=1)
        self.robot_state_pub_timer = rospy.Timer(
            rospy.Duration(1), self._publish_robot_state
        )
        rospy.sleep(0.5)
        self._load_world(world_config_file, odom_topic)

    def _detect_region_cb(self, msg: String):
        '''msg: "x,y~x,y" or "x,y,z~x,y,z"'''
        min_pos, max_pos = msg.data.split("~")
        min_pos = tuple(map(float, min_pos.split(",")))
        max_pos = tuple(map(float, max_pos.split(",")))

        detected_objects = []
        for name, obj in self.world_objects.items():
            obj: WorldObject
            if obj.state != "unknown":
                continue
            detected_objects.append(obj)
            for i in range(len(min_pos)):
                if not min_pos[i] <= obj.position[i] <= max_pos[i]:
                    detected_objects.pop()
                    break
        for obj in detected_objects:
            obj.change_state("detected")
            rospy.loginfo(f"We have detected {obj.name} at {obj.position}")
            self.feedback_pub.publish(f"We have detected {obj.name} at {obj.position}")

    def _assign_tasks_cb(self, msg: String):
        """
        {"1": [{"action": "action2",
                "duration": 2,
                "id": 1,
                "location": [10, 10],
                "required_robot_type": "1"}]}
        """
        rospy.loginfo("Assigning tasks")
        rospy.loginfo(msg.data)
        robot_schedule = json.loads(msg.data)
        for robot_id, action_list in robot_schedule.items():
            for action_str in action_list:
                method_name = action_str["action"].split()[0]
                method_target = action_str["action"].split()[1]
                action = Action(
                    id=action_str["id"],
                    method_name=method_name,
                    method_target=method_target,
                    location=tuple(action_str["location"]),
                    duration=action_str["duration"],
                )
                self.world_agents[robot_id].add_action(action)
    
    def _action_cb(self, msg: String):
        '''msg: {"action name","action target"}'''
        for obj in self.world_objects.values():
            obj: WorldObject
            if obj.state != "detected":
                continue
            if obj.name == msg.data.split(",")[1]:
                obj.change_state("finished")
                rospy.loginfo(f"{obj.name} has been completely handled")
                self.feedback_pub.publish(f"{obj.name} has been completely handled")

    def _load_world(self, file, odom_topic):
        world_config = json.load(open(file))
        for obj in world_config["objects"]:
            self.world_objects[obj["name"]] = WorldObject(
                obj["name"], tuple(obj["location"]), self.vis_pub
            )
        for agent in world_config["agents"]:
            if "model" in  agent.keys():
                self.model_path = rospy.get_param("~model_path")
                marker = MF.get_model_marker(agent["id"], self.model_path + agent["model"])
            else:
                marker = None
            self.world_agents[agent["id"]] = WorldAgent(
                agent["id"], agent["type"], odom_topic, max_speed=1, marker=marker
            )
        rospy.loginfo("World loaded")
        rospy.loginfo(self.world_objects)
        rospy.loginfo(self.world_agents)

    def _publish_robot_state(self, event):
        '''
        Robot state message format:
        [
            {"id": 0, "type": "1", "initial_location": [0, 10], "speed": 1.0, "state": "idle"},
            {"id": 1, "type": "1", "initial_location": [-10, 10], "speed": 1.0, "state": "idle"},
        ],
        '''
        if not all([agent.has_odom for agent in self.world_agents.values()]):
            return
        state_list = []
        for agent in self.world_agents.values():
            agent: WorldAgent
            state_list.append(
                {
                    "id": agent.id,
                    "type": agent.type,
                    "initial_location": agent.position,
                    "speed": agent.max_speed,
                    "state": agent.state,
                }
            )
        self.robot_state_pub.publish(json.dumps(state_list))


if __name__ == "__main__":
    rospy.init_node("world_manager")
    world_config_file = rospy.get_param("~world_config_file")
    odom_topic = rospy.get_param("~odom_topic", "odom")
    WorldManager(world_config_file, odom_topic)
    rospy.spin()
