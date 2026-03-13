from visualization_msgs.msg import Marker, MarkerArray
import rospy
from jsk_rviz_plugins.msg import OverlayText
from typing import Dict, List, Tuple
from matplotlib.colors import to_rgb


class MarkerFactory:
    @staticmethod
    def gen_obj_marker(id: int, obj):
        obj_marker = Marker()
        obj_marker.header.frame_id = "world"
        obj_marker.header.stamp = rospy.Time.now()
        obj_marker.id = id
        obj_marker.action = Marker.ADD
        obj_marker.pose.position.x = obj.location[0]
        obj_marker.pose.position.y = obj.location[1]
        obj_marker.pose.position.z = obj.location[2]
        obj_marker.pose.orientation.x = 0
        obj_marker.pose.orientation.y = 0
        obj_marker.pose.orientation.z = 0
        obj_marker.pose.orientation.w = 1
        obj_marker.scale.x = 1
        obj_marker.scale.y = 1
        obj_marker.scale.z = 1
        if obj.state == "active":
            obj_marker.color.a = 0.7
        else:
            obj_marker.color.a = 0
        if obj.object_type == "{flame}" or obj.object_type == "{algae}":
            color = to_rgb("#ff5900")
            marker_type = Marker.SPHERE
        elif obj.object_type == "{injured_person}" or obj.object_type == "{injured_animal}":
            color = to_rgb("#aa00ff")
            marker_type = Marker.SPHERE
        elif obj.object_type == "{rescue_station}":
            color = to_rgb("#00ff1a")
            marker_type = Marker.CUBE
        elif obj.object_type == "{water_source}" or obj.object_type == "{snow_heap}":
            color = to_rgb("#00ccff")
            marker_type = Marker.CUBE
        elif obj.object_type == "{sand_heap}" or obj.object_type == "{expedition_base}":
            color = to_rgb("#ffc800")
            marker_type = Marker.CUBE
        elif obj.object_type == "{fire_extinguisher}":
            color = to_rgb("#ff2525")
            marker_type = Marker.CYLINDER
        else:
            raise ValueError(f"Unknown object type {obj.object_type}")
        obj_marker.type = marker_type
        obj_marker.color.r = color[0]
        obj_marker.color.g = color[1]
        obj_marker.color.b = color[2]
        
        text_marker = Marker()
        text_marker.header.frame_id = "world"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.id = id + 1000
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = obj.location[0]
        text_marker.pose.position.y = obj.location[1]
        text_marker.pose.position.z = obj.location[2] + 2
        text_marker.pose.orientation.x = 0
        text_marker.pose.orientation.y = 0
        text_marker.pose.orientation.z = 0
        text_marker.pose.orientation.w = 1
        text_marker.scale.x = 1.5
        text_marker.scale.y = 1.5
        text_marker.scale.z = 1.5
        text_marker.type = Marker.TEXT_VIEW_FACING
        if obj.state == "active":
            text_marker.color.a = 1
        else:
            text_marker.color.a = 0
        text_marker.color.r = 0
        text_marker.color.g = 0
        text_marker.color.b = 0
        text_marker.text = obj.object_type[1:-1]
        
        return obj_marker, text_marker

    @staticmethod
    def id_2_color_8(i: int):
        color_palette = [
            "#ff3e00",
            "#ff8000",
            "#edce00",
            "#8aff00",
            "#00ffc0",
            "#00c2ff",
            "#5c00ff",
            "#d800ff",
        ]
        return color_palette[i % len(color_palette)]

    @staticmethod
    def id_2_color_10_light(i: int):
        color_palette = [
            "#ff8f70",
            "#ffe371",
            "#7bff76",
            "#75d9ff",
            "#c079ff",
            "#ffbf70",
            "#edff71",
            "#76ffcb",
            "#7585ff",
            "#ff79d9",
        ]
        return color_palette[i % len(color_palette)]

    @staticmethod
    def gen_robot_overlay(dict_text: Dict[int, str]) -> OverlayText:
        overlay_text = "Robot States\n\n"
        for robot_id, text in dict_text.items():
            color = MarkerFactory.id_2_color_8(robot_id)
            overlay_text += (
                f'<span style="color: {color};">{robot_id}: </span>{text}\n'
            )
        overlay = OverlayText()
        overlay.text = overlay_text
        overlay.width = 600
        overlay.height = 600
        overlay.left = 10
        overlay.top = 10
        overlay.text_size = 14
        overlay.action = OverlayText.ADD
        overlay.bg_color.a = 0
        overlay.fg_color.r = 0
        overlay.fg_color.g = 0
        overlay.fg_color.b = 0
        overlay.fg_color.a = 1
        return overlay

    @staticmethod
    def gen_task_overlay(task_list: List[Tuple[str, str, str]]) -> OverlayText:
        text = "Task States\n\n"
        todo_list = []
        in_progress_list = []
        done_list = []
        for i in range(len(task_list)):
            task, state, color = task_list[i]
            if state == "Todo":
                todo_list.append(f'<span style="color: {color};">■■■ </span>{task}')
            elif state == "In Progress":
                in_progress_list.append(f'<span style="color: {color};">■■■ </span>{task}')
            elif state == "Done":
                done_list.insert(0, f'<span style="color: {color};">■■■ </span>{task}')
            else:
                raise ValueError(f"Unknown task state {state}")
        text += f"Todo: {len(todo_list)}\n"
        text += "\n".join(todo_list)
        text += f"\n\nIn Progress: {len(in_progress_list)}\n"
        text += "\n".join(in_progress_list)
        text += f"\n\nDone: {len(done_list)}\n"
        text += "\n".join(done_list)
        overlay = OverlayText()
        overlay.text = text
        overlay.width = 600
        overlay.height = 600
        overlay.left = 1150
        overlay.top = 10
        overlay.text_size = 14
        overlay.action = OverlayText.ADD
        overlay.bg_color.a = 0
        overlay.fg_color.r = 0
        overlay.fg_color.g = 0
        overlay.fg_color.b = 0
        overlay.fg_color.a = 1
        return overlay

    def gen_delete_marker():
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.DELETEALL
        return marker
