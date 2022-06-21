#!/usr/bin/env python3
import time
import os
import argparse
import sys
import numpy as np

import rospy
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

from ambf_msgs.msg import RigidBodyCmd



def main():
    rospy.init_node("slow_move_volume")
    pub = rospy.Publisher("/ambf/env/Sphere/Command", RigidBodyCmd,queue_size=1)
    # pub = rospy.Publisher("/ambf/env/snake_stick/Command", RigidBodyCmd,queue_size=1)


    rate = rospy.Rate(1000)
    cmd = RigidBodyCmd()
    while not rospy.is_shutdown():
        cmd.cartesian_cmd_type=1
        cmd.pose.position.x = 0.22
        cmd.pose.position.y = 1.51
        cmd.pose.position.z = -1.23 - 0.064 * np.sin(0.01*time.time()+np.pi/3)
        cmd.pose.orientation.w = 1.0
        pub.publish(cmd)
        rate.sleep()


if (__name__ == "__main__"):
    main()
