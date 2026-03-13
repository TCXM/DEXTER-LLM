#!/usr/bin/env python3
# pylint: disable=R0902, R0903, C0103
"""
Gantt.py is a simple class to render Gantt charts, as commonly used in
"""

import os
import json
import platform
from operator import sub

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
import rospy
from std_srvs.srv import Trigger, TriggerResponse
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# TeX support: on Linux assume TeX in /usr/bin, on OSX check for texlive
if (platform.system() == "Darwin") and "tex" in os.getenv("PATH"):
    LATEX = True
elif (platform.system() == "Linux") and os.path.isfile("/usr/bin/latex"):
    LATEX = True
else:
    LATEX = False

# setup pyplot w/ tex support
if LATEX:
    rc("text", usetex=True)


class Package:
    """Encapsulation of a work package

    A work package is instantiated from a dictionary. It **has to have**
    a label, astart and an end. Optionally it may contain milestones
    and a color

    :arg str pkg: dictionary w/ package data name
    """

    def __init__(self, pkg):

        DEFCOLOR = "#32AEE0"

        self.label = pkg["label"]
        self.start = pkg["start"]
        self.end = pkg["end"]

        if self.start < 0 or self.end < 0:
            raise ValueError("Package cannot begin at t < 0")
        if self.start > self.end:
            raise ValueError("Cannot end before started")

        try:
            self.milestones = pkg["milestones"]
        except KeyError:
            pass

        try:
            self.color = pkg["color"]
        except KeyError:
            self.color = DEFCOLOR

        try:
            self.name_color = pkg["name_color"]
        except KeyError:
            self.name_color = "black"

        try:
            self.legend = pkg["legend"]
        except KeyError:
            self.legend = None

        try:
            self.name = pkg["name"]
        except KeyError:
            self.name = None

        try:
            self.label_color = pkg["label_color"]
        except KeyError:
            self.label_color = "black"

        try:
            self.hatch = pkg["hatch"]
        except KeyError:
            self.hatch = None


class Gantt:
    """Gantt
    Class to render a simple Gantt chart, with optional milestones
    """

    def __init__(self):
        self.gantt_client = rospy.ServiceProxy("/robots_gantt", Trigger)
        self.gantt_client.wait_for_service()
        self.plot_mode = rospy.get_param("~plot_mode", "window")
        if self.plot_mode == "window":
            plt.ion()
            # init figure
            self.fig, self.ax = plt.subplots(figsize=(15, 4))
            self.ax.xaxis.grid(True)
            self.ax.yaxis.grid(False)
            while not rospy.is_shutdown():
                data: TriggerResponse = self.gantt_client()
                if data.success:
                    plt.cla()
                    self.set_gantt(data.message)
                    plt.pause(1)
                else:
                    plt.pause(1)
        elif self.plot_mode == "publish":
            image_pub = rospy.Publisher("/gantt_image", Image, queue_size=10)
            self.log_dir = rospy.get_param("~log_dir", "")
            if self.log_dir == "":
                raise ValueError("log_dir must be set in 'publish' mode")
            bridge = CvBridge()
            # init figure
            self.fig, self.ax = plt.subplots(figsize=(15, 3))
            self.ax.xaxis.grid(True)
            self.ax.yaxis.grid(False)
            while not rospy.is_shutdown():
                data: TriggerResponse = self.gantt_client()
                if data.success:
                    plt.cla()
                    self.set_gantt(data.message)
                    save_path = f"{self.log_dir}/GANTT.png"
                    Gantt.save(save_path)
                    cv_image = cv2.imread(save_path)
                    ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                    image_pub.publish(ros_image)
                rospy.sleep(1)

    def set_gantt(self, dataFile):
        """Instantiation

        Create a new Gantt using the data in the file provided
        or the sample data that came along with the script

        :arg str dataFile: file holding Gantt data
        """
        self.dataFile = dataFile

        # some lists needed
        self.packages = []
        self.labels = []

        self._loadData()
        self._procData()
        self.render()

    def _loadData(self):
        """Load data from a JSON file that has to have the keys:
        packages & title. Packages is an array of objects with
        a label, start and end property and optional milesstones
        and color specs.
        """

        # load data
        data = json.loads(self.dataFile)

        # must-haves
        self.title = data["title"]

        for pkg in data["packages"]:
            self.packages.append(Package(pkg))

        self.labels = [pkg["label"] for pkg in data["packages"]]

        # optionals
        self.milestones = {}
        for pkg in self.packages:
            try:
                self.milestones[pkg.label] = pkg.milestones
            except AttributeError:
                pass

        try:
            self.xlabel = data["xlabel"]
        except KeyError:
            self.xlabel = ""
        try:
            self.xticks = data["xticks"]
        except KeyError:
            self.xticks = ""

        try:
            self.current_time = data["current_time"]
        except KeyError:
            self.current_time = None

    def _procData(self):
        """Process data to have all values needed for plotting"""
        # parameters for bars
        self.nPackages = len(self.labels)
        self.start = [None] * self.nPackages
        self.end = [None] * self.nPackages

        for pkg in self.packages:
            idx = self.labels.index(pkg.label)
            self.start[idx] = pkg.start
            self.end[idx] = pkg.end

        self.durations = map(sub, self.end, self.start)
        self.yPos = np.arange(self.nPackages, 0, -1)

    def format(self):
        """Format various aspect of the plot, such as labels,ticks, BBox
        :todo: Refactor to use a settings object
        """
        # format axis
        plt.tick_params(
            axis="both",  # format x and y
            which="both",  # major and minor ticks affected
            bottom="on",  # bottom edge ticks are on
            top="off",  # top, left and right edge ticks are off
            left="off",
            right="off",
        )

        # tighten axis but give a little room from bar height
        plt.xlim(0, max(self.end))
        plt.ylim(0.5, self.nPackages + 0.5)

        # add title and package names
        plt.yticks(self.yPos, self.labels)
        plt.title(self.title)

        if self.xlabel:
            plt.xlabel(self.xlabel)

        if self.xticks:
            plt.xticks(self.xticks, map(str, self.xticks))

    def render(self, frame=0):
        """Prepare data for plotting, supporting multiple blocks per line"""

        # 获取所有包的 label，并保留顺序
        unique_labels = list(dict.fromkeys([pkg.label for pkg in self.packages]))
        # 设置 y 坐标，倒序排列
        y_coords = np.arange(len(unique_labels), 0, -1)
        label_to_y = dict(zip(unique_labels, y_coords))

        # 绘制每个包，多个包共用同一 label 的 y 坐标
        self.barlist = []
        for pkg in self.packages:
            pkg: Package
            y = label_to_y[pkg.label]
            width = pkg.end - pkg.start
            bar = self.ax.barh(
                y,
                width,
                left=pkg.start,
                height=0.5,
                color=pkg.color,
                align="center",
                hatch=pkg.hatch,
                edgecolor="gray",
                linewidth=0
            )
            self.barlist.append(bar)
            # Add task_name text
            self.ax.text(
                pkg.start + width / 2,
                y,
                pkg.name,
                va="center",
                ha="center",
                color=pkg.name_color,
                fontsize=10,
                fontweight="bold",
            )

        # 调整坐标轴
        self.ax.set_xlim(
            0, max(max(pkg.end for pkg in self.packages), self.current_time)
        )
        self.ax.set_ylim(0.5, len(unique_labels) + 0.5)
        self.ax.set_yticks(list(label_to_y.values()))
        self.ax.set_yticklabels(list(label_to_y.keys()))
        for label in self.ax.get_yticklabels():
            label.set_color(
                self.packages[unique_labels.index(label.get_text())].label_color
            )
        self.ax.set_title(self.title)
        if self.xlabel:
            self.ax.set_xlabel(self.xlabel)
        if self.xticks:
            self.ax.set_xticks(self.xticks)
            self.ax.set_xticklabels(list(map(str, self.xticks)))

        # 添加里程碑与图例
        self.add_milestones(label_to_y)
        self.add_legend(label_to_y)
        self.add_current_time_line()

    def add_milestones(self, label_to_y):
        """Add milestones to Gantt chart.
        The milestones are drawn as yellow diamonds.
        """
        if not self.milestones:
            return

        xs = []
        ys = []
        for label, milestones in self.milestones.items():
            if label in label_to_y:
                for m in milestones:
                    xs.append(m)
                    ys.append(label_to_y[label])
        self.ax.scatter(
            xs, ys, s=120, marker="D", color="yellow", edgecolor="black", zorder=3
        )

    def add_legend(self, label_to_y):
        """Add a legend to the plot if any package defines a legend entry."""
        # 用一个字典避免重复图例项
        handles = {}
        for pkg, bar in zip(self.packages, self.barlist):
            if pkg.legend and pkg.legend not in handles:
                handles[pkg.legend] = bar[0]  # barh 返回一个容器，取第一个 patch

        if handles:
            handles = {k: v for k, v in reversed(list(handles.items()))}
            if len(handles) > 10:
                handles = dict(list(handles.items())[:10])
            self.ax.legend(
                handles.values(),
                handles.keys(),
                shadow=False,
                ncol=1,
                fontsize="medium",
                loc="upper left",
                bbox_to_anchor=(0, 1),
            )

    def add_current_time_line(self):
        """Add a red line to indicate the current time."""
        if self.current_time is not None:
            self.ax.axvline(
                self.current_time,
                color="red",
                linewidth=2,
                linestyle="--",
                label="Current Time",
            )

    @staticmethod
    def show():
        """Show the plot"""
        plt.show()

    @staticmethod
    def save(saveFile="img/GANTT.png"):
        """Save the plot to a file. It defaults to `img/GANTT.png`.

        :arg str saveFile: file to save to
        """
        plt.savefig(saveFile, bbox_inches="tight")


if __name__ == "__main__":
    rospy.init_node("gantt", anonymous=True)
    Gantt()
