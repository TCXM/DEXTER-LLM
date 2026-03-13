#! /usr/bin/python
import rospy
from std_msgs.msg import String

EXAMPLE = """
{
    "1": [
        {
            "action": "move_to(flame_1)",
            "duration": 0,
            "id": 1,
            "location": [1, 1, 1],
            "required_robot_type": "1"
        },
        {
            "action": "spray_water_to(flame_1)",
            "duration": 3,
            "id": 2,
            "location": [1, 1, 1],
            "required_robot_type": "1"
        }
    ]
}
"""
class TestNode:
    def __init__(self):
        self.detect_region_pub = rospy.Publisher("/detect_region", String, queue_size=1)
        self.assign_tasks_pub = rospy.Publisher("/assign_tasks", String, queue_size=1)
        self.detect_region_pub.publish("0,0-2,2")
        rospy.sleep(3)
        self.assign_tasks_pub.publish(EXAMPLE)

if __name__ == "__main__":
    rospy.init_node("world_manager")
    test_node = TestNode()
    rospy.spin()
