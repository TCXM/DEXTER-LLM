#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Empty

class ScriptsPlayer:
    def __init__(self):
        self.start_play_sub = rospy.Subscriber(
            "/start_play_script", Empty, self._start_play_cb
        )

    def _start_play_cb(self, msg):
        self._script_timer_cb(None)

    def _script_timer_cb(self, event):
        if not hasattr(self, "script"):
            file_path = rospy.get_param("~script_file")
            with open(file_path) as f:
                self.script = f.readlines()
        if not hasattr(self, "script_pub"):
            self.script_pub = rospy.Publisher("/script", String, queue_size=1)
        if self.script:
            s = self.script.pop(0)
            if "sleep" in s:
                self.script_timer = rospy.Timer(
                    rospy.Duration(float(s.split()[1])),
                    self._script_timer_cb,
                    oneshot=True,
                )
            else:
                self.script_pub.publish(String(s))
                rospy.loginfo("\033[35m" + s + "\033[0m")
                self._script_timer_cb(None)