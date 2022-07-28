#!/usr/bin/env python2

import rospy

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
from ambf_msgs.msg import RigidBodyState
from ambf_msgs.msg import RigidBodyCmd
import numpy as np
import matplotlib.pyplot as plt
import tf.transformations as tf
import tf2_geometry_msgs
import time
import os
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize, LinearConstraint, Bounds
import PyKDL

class cm_insert_bend_controller:


    def __init__(self):
        rospy.init_node('cm_insert_bend_controller', anonymous=True)
        base_position_sub = rospy.Subscriber('ambf/env/snake_stick/State', RigidBodyState, self.ambf_base_marker_sub_callback)
        self.base_position_pub = rospy.Publisher('ambf/env/snake_stick/Command', RigidBodyCmd, queue_size=1)
        tip_position_sub  = rospy.Subscriber('ambf/env/Burr/State', RigidBodyState, self.ambf_tip_marker_sub_callback)
        
        bend_sub = rospy.Subscriber('/ambf/volumetric_drilling/bend_motor/measured_js/', Float32, self.bend_sub_callback)
        self.bend_pub = rospy.Publisher('/ambf/volumetric_drilling/bend_motor/move_jp/', Float32)
        self.collect = False
        self.bend_motor_pos = 0
        self.ms = 0.001
        self.sec = 1.0
        self.base_transform = TransformStamped()
        self.tip_transform = TransformStamped()
        self.base_command = RigidBodyCmd()
        self.bend_command = Float32()
        self.insertion_vel = 0
        self.bend_vel = 0
        self.ros_node_rate_hz = 20
        self.node_rate = rospy.Rate(self.ros_node_rate_hz)
        self.goal_points = np.genfromtxt("/home/henry/snake_registration/simulation/data_process/goal_points.csv",delimiter=',')
        self.num_goals = self.goal_points.shape[0]

    def dist_to_goal(self, v, goal_position_world, current_position_world, J1, J2, dt=0.05):
        # print("v",v)
        insertion = v[0]
        bend = v[1]
        test_scale = 5.0
        pred_new_pos = current_position_world.p + J1*insertion*dt + test_scale*J2*bend*dt
        err = pred_new_pos - goal_position_world.p
        return err.Norm()

    def set_goal_to_ith_goalpoint(self, goal_num):
        x,y,z = self.goal_points[goal_num, :]
        goalp=PyKDL.Vector(x,y,z)
        self.T_current_goal = PyKDL.Frame( PyKDL.Rotation(),goalp)


    def run(self):
        goal_num = 0
        self.set_goal_to_ith_goalpoint(goal_num)
        while True:
            T_tip = tf2_geometry_msgs.transform_to_kdl(self.tip_transform)
            # T_tip.p = T_tip * PyKDL.Vector(0,0.0065/2*10,0)
            T_base = tf2_geometry_msgs.transform_to_kdl(self.base_transform)
            T_pos = T_base.Inverse()*T_tip
            # print("goal: ", self.T_current_goal)
            # print("pos: ", T_pos)
            # print("tip: ", T_tip)
            # print(T_tip)
            # print("x: ",T_pos.M.UnitX()) 
            # print("y: ",T_pos.M.UnitY()) 
            # print("z: ",T_pos.M.UnitZ())
            print(f"T_current_goal.p: {self.T_current_goal.p}, T_tip.p: {T_tip.p}") 
            err_norm = (self.T_current_goal.p - T_tip.p).Norm()
            print(f"err_norm: {err_norm}")
            if err_norm < 0.005:
                goal_num += 1
                if goal_num < self.num_goals:
                    self.set_goal_to_ith_goalpoint(goal_num)
                    print(f"Set to new goal {goal_num}")
                else:
                    print("Reached final goal point")
                    return 

            # This is the positional jacobian direction of insertion
            tip_dir_bendplane = 1.0*T_pos.M.UnitY()
            # This is the positional jacobian direction of bending
            bend_dir_bendplane = -1.0*T_pos.M.UnitX()

            insert_dir = T_base.M.UnitY()

            bend_max = 0.032
            bend_min = -0.032
            max_cable_vel_bound_to_keep_in_range = max(0.0,(bend_max - self.bend_command.data)*self.ros_node_rate_hz)
            # only goes negative if current pos bigger than max
            min_cable_vel_bound_to_keep_in_range = min(0.0,(bend_min - self.bend_command.data)*self.ros_node_rate_hz)


            v = [0.0,0.0]
            vel_bounds = Bounds(lb=[-0.01,max(-0.0005,min_cable_vel_bound_to_keep_in_range)], ub=[0.01, min(0.0005, max_cable_vel_bound_to_keep_in_range) ])
            # vel_constraint = LinearConstraint(np.eye(2), lb=[-0.01,-0.0001], ub=[0.01,0.0001])
            # res = minimize(self.dist_to_goal, v, args=(T_goal, T_pos, tip_dir_bendplane, bend_dir_bendplane))
            res = minimize(self.dist_to_goal, v, args=(self.T_current_goal, T_tip, tip_dir_bendplane, bend_dir_bendplane), bounds=vel_bounds)

            v_in, v_bend = res.x
            print(f"v_in: {v_in}, v_bend: {v_bend}")
            # currently assume that we want no change in orientation
            # th1 - insertion
            # th2 - bend

            # goal is to minimize error to goal position through selection of th1, th2



            self.base_command.cartesian_cmd_type = 2
            (self.base_command.twist.linear.x, self.base_command.twist.linear.y, self.base_command.twist.linear.z) = insert_dir*v_in
            # (self.base_command.pose.position.x, self.base_command.pose.position.y, self.base_command.pose.position.z) = tip_dir_bendplane*v_bend

            self.base_position_pub.publish(self.base_command)

            self.bend_command.data += (v_bend*1/self.ros_node_rate_hz)
            # print(f"self.bend_motor_pos: {self.bend_motor_pos}, v_bend: {v_bend}")
            # print(f"self.bend_command.data: {self.bend_command.data}, v_bend: {v_bend}")


            self.bend_pub.publish(self.bend_command)

            self.node_rate.sleep()
       

                 

    def base_marker_sub_callback(self,transform):
        self.base_transform = transform
        if self.collect:
            self.base_transforms_sublist.append(transform)

    def tip_marker_sub_callback(self,transform):
        self.tip_transform = transform
        if self.collect:
            self.tip_transforms_sublist.append(transform)

    def ambf_base_marker_sub_callback(self,ambf_rigid_body_state):
        self.base_marker_sub_callback(ambf_rigid_body_state_to_transform_stamped(ambf_rigid_body_state))
  
    def ambf_tip_marker_sub_callback(self,ambf_rigid_body_state):
        self.tip_marker_sub_callback(ambf_rigid_body_state_to_transform_stamped(ambf_rigid_body_state))

    def bend_sub_callback(self,motor_pos):
        self.bend_motor_pos = motor_pos.data
        if self.collect:
            self.bend_motor_pos_sublist.append(motor_pos)

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

if __name__ == '__main__':
    try:
        node = cm_insert_bend_controller()
        node.run()

    except rospy.ROSInterruptException:
        pass

