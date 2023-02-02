
import rospy
from std_msgs.msg import Bool

class CmvdRosInterfaceBoolPublisher():
    def __init__(self, topic_name, latch=False):
        self.topic_name = topic_name
        self.msg = Bool()
        self.pub = rospy.Publisher(topic_name, Bool,queue_size=1,latch=latch)

    def set_value(self, data):
        self.msg.data = data
        self.pub.publish(self.msg)

class CmvdRosInterface():
    def __init__(self, cm_plugin_rosnamespace='/ambf/volumetric_drilling'):
        self.ros_nh = rospy.get_node_uri()  
        self.cable_control_mode = CmvdRosInterfaceBoolPublisher(cm_plugin_rosnamespace + "/setCableControlMode", latch=True)
        self.drill_control_mode = CmvdRosInterfaceBoolPublisher(cm_plugin_rosnamespace + "/setDrillControlMode", latch=True)
        self.physics_paused = CmvdRosInterfaceBoolPublisher(cm_plugin_rosnamespace + "/setPhysicsPaused", latch=True)
        self.show_tool_cursors = CmvdRosInterfaceBoolPublisher(cm_plugin_rosnamespace + "/setShowToolCursors", latch=True)
        self.volume_collisions_enabled = CmvdRosInterfaceBoolPublisher(cm_plugin_rosnamespace + "/setVolumeCollisionsEnabled", latch=True)
        self.init_tool_cursors = CmvdRosInterfaceBoolPublisher(cm_plugin_rosnamespace + "/initToolCursors", latch=True)
        self.reset_voxels = CmvdRosInterfaceBoolPublisher(cm_plugin_rosnamespace + "/resetVoxels", latch=True)
        self.burr_on = CmvdRosInterfaceBoolPublisher(cm_plugin_rosnamespace + "/setBurrOn", latch=True)
        self.toggle_trace_collect = CmvdRosInterfaceBoolPublisher(cm_plugin_rosnamespace + "/set_body_trace_collect", latch=True)
        self.toggle_trace_visible = CmvdRosInterfaceBoolPublisher(cm_plugin_rosnamespace + "/set_body_trace_visible", latch=True)

class AmbfTraceRosInterface():
    def __init__(self, trace_plugin_rosnamespace='/ambf/trace_plugin'):
        self.ros_nh = rospy.get_node_uri()  
        self.toggle_trace_collect = CmvdRosInterfaceBoolPublisher(trace_plugin_rosnamespace + "/set_body_trace_collect")
        self.toggle_trace_visible = CmvdRosInterfaceBoolPublisher(trace_plugin_rosnamespace + "/set_body_trace_visible")