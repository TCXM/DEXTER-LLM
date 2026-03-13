from visualization_msgs.msg import Marker, MarkerArray
import rospy


class MarkerFactory:
    @staticmethod
    def create_marker(name, location):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = name
        marker.id = 0
        marker.action = Marker.ADD
        marker.pose.position.x = location[0]
        marker.pose.position.y = location[1]
        marker.pose.position.z = location[2]
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 1
        if "flame" in name:
            color = [1, 0, 0]
            marker_type = Marker.SPHERE
        elif "person" in name:
            color = [0, 0, 1]
            marker_type = Marker.CUBE
        marker.type = marker_type
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        return marker

    @staticmethod
    def get_model_marker(name, model_path):
        marker = Marker()
        marker.id = 0
        marker.ns = name
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "world"
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.color.a = 1.0
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 0
        marker.mesh_use_embedded_materials = True
        marker.mesh_resource = model_path
        return marker
