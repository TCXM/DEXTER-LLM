from typing import *
from visualization_msgs.msg import Marker
import rospy
from dataclasses import dataclass
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_msgs.msg import String
from marker_factory import MarkerFactory as MF
import random


class WorldObject:
    def __init__(
        self, name: str, position: Tuple[float, float, float], vis_pub: rospy.Publisher
    ):
        self.name = name
        self.position = position
        self.state = "unknown"
        self.vis_pub = vis_pub

        self.marker = MF.create_marker(name, position)
        self.marker.color.a = 0.2
        self.vis_pub.publish(self.marker)
        rospy.loginfo(f"WorldObject {name} created at {position}")

    def change_state(self, state: str):
        self.state = state
        rospy.loginfo(f"{self.name} change state to {state}")
        if state == "detected":
            self.marker.color.a = 1
            self.vis_pub.publish(self.marker)
        elif state == "finished":
            self.marker.color.a = 0
            self.vis_pub.publish(self.marker)


@dataclass
class Action:
    id: int
    method_name: str
    method_target: str
    location: Tuple
    duration: float


class WorldAgent:
    def __init__(
        self,
        id: int,
        type: str,
        odom_topic: str,
        max_speed: float = 1.0,
        marker: Marker = None,
    ):
        self.id = id
        self.type = type
        self.has_odom = False
        self.position = (0, 0, 0)
        self.max_speed = max_speed
        self.state = "idle"
        self.actions: List[Action] = []
        self.current_action = None
        self.timer = rospy.Timer(rospy.Duration(1), self._timer_cb)
        self.waypoint_pub = rospy.Publisher(
            f"/drone_{self.id}_planning/waypoint", PoseStamped, queue_size=10
        )
        self.odom_sub = rospy.Subscriber(
            f"/drone_{self.id}_{odom_topic}", Odometry, self._odom_cb
        )
        self.action_pub = rospy.Publisher(f"/action", String, queue_size=1)
        self.performing_timer = None

        # Visualization
        if marker != None:
            self.marker = marker
            self.vis_pub = rospy.Publisher(f"/extended_vis", Marker, queue_size=10)
            self.vis_timer = rospy.Timer(rospy.Duration(0.1), self._vis_timer_cb)

    def _vis_timer_cb(self, event):
        if self.has_odom == False:
            return
        self.marker.pose = self.pose
        self.vis_pub.publish(self.marker)

    def add_action(self, action: Action):
        self.actions.append(action)

    def _timer_cb(self, event):
        if self.state == "idle":
            if len(self.actions) == 0:
                return
            a = self.actions.pop(0)
            self.current_action = a
            if self._distance(self.position, a.location) < 0.5:
                self._change_state("waiting")
                return
            else:
                self._change_state("moving")
                return
        elif self.state == "moving":
            a = self.current_action
            noisy_location = tuple(coord + random.uniform(-0.3, 0.3) for coord in a.location)
            destination = self._creat_destination(noisy_location)
            self.waypoint_pub.publish(destination)
            if self._distance(self.position, a.location) < 0.5:
                self._change_state("waiting")
                return
        elif self.state == "waiting":
            if True:
                self._change_state("performing")
            return
        elif self.state == "performing":
            if self.current_action.duration == 0:
                self._change_state("idle")
            elif self.performing_timer == None:
                self.performing_timer = rospy.Timer(
                    rospy.Duration(self.current_action.duration),
                    self._perfroming_timer_cb,
                    oneshot=True,
                )

    def _perfroming_timer_cb(self, event):
        self.action_pub.publish(
            f"{self.current_action.method_name},{self.current_action.method_target}"
        )
        self._change_state("idle")
        self.performing_timer = None

    def _change_state(self, state: str):
        self.state = state
        rospy.loginfo(f"Robot {self.id} change state to {state}")

    def _odom_cb(self, msg: Odometry):
        self.has_odom = True
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        self.pose = msg.pose.pose

    def _distance(self, a, b) -> float:
        return sum((a[i] - b[i]) ** 2 for i in range(len(b))) ** 0.5

    def _creat_destination(self, position: Tuple) -> PoseStamped:
        return PoseStamped(
            header=Header(frame_id="world"),
            pose=Pose(
                position=(Point(x=position[0], y=position[1], z=position[2])),
                orientation=Quaternion(w=1),
            ),
        )
