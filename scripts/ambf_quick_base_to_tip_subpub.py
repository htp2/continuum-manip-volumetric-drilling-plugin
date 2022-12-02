#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import TransformStamped
from ambf_msgs.msg import RigidBodyState
import numpy as np
import tf.transformations as tr
import time
import os
 
class ambf_quick_base_to_tip_subpub:


    def __init__(self, rate_hz):

        rospy.init_node('ambf_quick_base_to_tip_subpub', anonymous=True)
        base_marker_sub = rospy.Subscriber('ambf/env/snake_stick/State', RigidBodyState, self.ambf_base_marker_sub_callback)
        tip_marker_sub  = rospy.Subscriber('ambf/env/seg27/State', RigidBodyState, self.ambf_tip_marker_sub_callback)
        self.base_to_tip_pub = rospy.Publisher('/ambf/volumetric_drilling/base_to_tip/', TransformStamped)
        self.base_transform = TransformStamped()
        self.tip_transform = TransformStamped()
        self.rate_hz = rate_hz

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            base_to_tip_transform = compose_TransformStamped_invAxB(self.base_transform, self.tip_transform)
            self.base_to_tip_pub.publish(base_to_tip_transform)
            rate.sleep()

    def base_marker_sub_callback(self,transform):
        self.base_transform = transform

    def tip_marker_sub_callback(self,transform):
        self.tip_transform = transform

    def ambf_base_marker_sub_callback(self,ambf_rigid_body_state):
        self.base_marker_sub_callback(ambf_rigid_body_state_to_transform_stamped(ambf_rigid_body_state))
  
    def ambf_tip_marker_sub_callback(self,ambf_rigid_body_state):
        self.tip_marker_sub_callback(ambf_rigid_body_state_to_transform_stamped(ambf_rigid_body_state))


def ambf_rigid_body_state_to_transform_stamped(ambf_rigid_body_state):
    transform = TransformStamped()
    transform.transform.translation.x = ambf_rigid_body_state.pose.position.x
    transform.transform.translation.y = ambf_rigid_body_state.pose.position.y
    transform.transform.translation.z = ambf_rigid_body_state.pose.position.z
    transform.transform.rotation.x = ambf_rigid_body_state.pose.orientation.x
    transform.transform.rotation.y = ambf_rigid_body_state.pose.orientation.y
    transform.transform.rotation.z = ambf_rigid_body_state.pose.orientation.z
    transform.transform.rotation.w = ambf_rigid_body_state.pose.orientation.w
    return transform

def TransformStamped_to_mat4x4(transform_stamped):
    R = transform_stamped.transform.rotation
    p = transform_stamped.transform.translation
    q = np.array([R.x,R.y,R.z,R.w])
    q = q / np.linalg.norm(q)
    g = tr.quaternion_matrix(q)
    g[0:3,-1] = np.array([p.x,p.y,p.z])
    return g

def max4x4_to_TransformStamped(g):
    out = TransformStamped()
    q = tr.quaternion_from_matrix(g)
    p = tr.translation_from_matrix(g)
    R = out.transform.rotation
    t = out.transform.translation
    R.x, R.y, R.z, R.w = q
    t.x, t.y, t.z = p
    return out

def compose_TransformStamped_invAxB(A, B):
    g_A = TransformStamped_to_mat4x4(A)
    g_B = TransformStamped_to_mat4x4(B)
    g_out = np.linalg.inv(g_A) @ g_B
    return max4x4_to_TransformStamped(g_out)




if __name__ == '__main__':
    try:
        node = ambf_quick_base_to_tip_subpub(20)
        node.run()

    except rospy.ROSInterruptException:
        pass

