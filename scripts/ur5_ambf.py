#!/usr/bin/env python3

import numpy as np
from ambf_client import Client
import rospy
from argparse import ArgumentParser

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import time
from scipy.spatial.transform import Rotation


class UR5_AMBF:
    def __init__(self, client, name, use_simul_pos_for_vel=False):
        self.client = client
        self.name = name
        self.base = self.client.get_obj_handle(name + '/base_link')
        self.use_simul_pos_for_vel = use_simul_pos_for_vel
        time.sleep(0.5)
        self.rate_hz = 120
        self.rate = rospy.Rate(self.rate_hz)

        self._T_b_w = None
        self._T_w_b = None
        self._base_pose_updated = False
        self._num_joints = 6
        self.servo_jv_cmd = [0, 0, 0, 0, 0, 0]
        self.servo_jp_cmd = [0, 0, 0, 0, 0, 0]
        self.servo_jp_flag = False
        self.pub_measured_js = rospy.Publisher(
            "/ambf/env/"+name+"/measured_js", JointState, queue_size=1)
        self.sub_servo_jp = rospy.Subscriber(
            "/ambf/env/"+name+"/servo_jp", JointState, self.sub_servo_jp_callback)
        self.sub_servo_jv = rospy.Subscriber(
            "/ambf/env/"+name+"/servo_jv", JointState, self.sub_servo_jv_callback)
        self.pub_measured_cp = rospy.Publisher(
            "/ambf/env/"+name+"/measured_cp", PoseStamped, queue_size=1)
        self.pub_jacobian = rospy.Publisher(
            "/ambf/env/"+name+"/jacobian", Float64MultiArray, queue_size=1)
        # self.run_once = False
        self.set_dh("UR5")
    # def set_home_pose(self, pose):
    # 	self.T_t_b_home = pose

    def is_present(self):
        if self.base is None:
            return False
        else:
            return True

    # def get_ik_solution(self):
    #     return self._ik_solution

    # def get_T_b_w(self):
    # 	self._update_base_pose()
    # 	return self._T_b_w

    # def get_T_w_b(self):
    # 	self._update_base_pose()
    # 	return self._T_w_b

    # def _update_base_pose(self):
    # 	if not self._base_pose_updated:
    # 		p = self.base.get_pos()
    # 		q = self.base.get_rot()
    # 		P_b_w = Vector(p.x, p.y, p.z)
    # 		R_b_w = Rotation.Quaternion(q.x, q.y, q.z, q.w)
    # 		self._T_b_w = Frame(R_b_w, P_b_w)
    # 		self._T_w_b = self._T_b_w.Inverse()
    # 		self._base_pose_updated = True

    # def servo_cp(self, T_t_b):
    #     if type(T_t_b) in [np.matrix, np.array]:
    #         T_t_b = convert_mat_to_frame(T_t_b)

    #     ik_solution = compute_IK(T_t_b)
    #     self._ik_solution = enforce_limits(ik_solution)
    #     self.servo_jp(self._ik_solution)

    # def servo_cv(self, twist):
    #     pass

    def servo_jp(self, jp):
        if (not self.is_present()):
            return
        # jp = self._joint_error_model.add_to_joints(jp, self._joints_error_mask)
        self.base.set_joint_pos(0, jp[0])
        self.base.set_joint_pos(1, jp[1])
        self.base.set_joint_pos(2, jp[2])
        self.base.set_joint_pos(3, jp[3])
        self.base.set_joint_pos(4, jp[4])
        self.base.set_joint_pos(5, jp[5])

    def servo_jv(self, jv):
        if self.use_simul_pos_for_vel:
            # js = self.measured_js()
            # for i in range(6):
                # print("p: ", js[i])
                # print("v: ", jv[i])
                # print("jp: ", js[i] + jv[i]/self.rate_hz)
            jp = [p + v/self.rate_hz for p,v in zip(self.measured_js(),jv)]
            self.servo_jp(jp)
            return
        if (not self.is_present()):
            return
        self.base.set_joint_vel(0, jv[0])
        self.base.set_joint_vel(1, jv[1])
        self.base.set_joint_vel(2, jv[2])
        self.base.set_joint_vel(3, jv[3])
        self.base.set_joint_vel(4, jv[4])
        self.base.set_joint_vel(5, jv[5])

    # def measured_cp(self):
    #     jp = self.measured_js()
    #     jp.append(0.0)
    #     return compute_FK(jp, 7)

    def measured_js(self):
        j0 = self.base.get_joint_pos(0)
        j1 = self.base.get_joint_pos(1)
        j2 = self.base.get_joint_pos(2)
        j3 = self.base.get_joint_pos(3)
        j4 = self.base.get_joint_pos(4)
        j5 = self.base.get_joint_pos(5)
        q = [j0, j1, j2, j3, j4, j5]
        self.js = q
        # q = self._joint_error_model.remove_from_joints(q, self._joints_error_mask)
        return q

    def measured_jv(self):
        j0 = self.base.get_joint_vel(0)
        j1 = self.base.get_joint_vel(1)
        j2 = self.base.get_joint_vel(2)
        j3 = self.base.get_joint_vel(3)
        j4 = self.base.get_joint_vel(4)
        j5 = self.base.get_joint_vel(5)
        return [j0, j1, j2, j3, j4, j5]

    def get_joint_names(self):
        return self.base.get_joint_names()

    def sub_servo_jv_callback(self, msg):  # JointState
        self.servo_jv_cmd = msg.velocity
    # self.servo_jv(msg.velocity)
        # if self.run_once:
        # 	return
        # # self.run_once = True
        # # jp=[0,0,0,0,0,0]# jp = self.measured_js()
        # delp = [v*1.0/self.rate_hz for v in msg.velocity]
        # new_js = [(x+y) for x,y in zip(self.js, delp)]
        # new_js[0]+=np.pi
        # self.servo_jp(new_js)

    def sub_servo_jp_callback(self, msg):  # JointState
        self.servo_jp_cmd = msg.position
        self.servo_jp_flag = True
    # self.servo_jp(msg.position)

    def publish_measured_js(self):
        msg = JointState()
        msg.name = 'ur5_ambf'
        msg.position = self.measured_js()
        msg.header.stamp = rospy.Time.now()
        self.pub_measured_js.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            if not self.is_present():
                self.base = self.client.get_obj_handle(
                    self.name + '/base_link')
                print("Tried to reconnect")
            self.publish_measured_js()
            self.FK(self.measured_js())
            if self.servo_jp_flag:
                self.servo_jp(self.servo_jp_cmd)
                self.servo_jp_flag = False
            else:
                self.servo_jv(self.servo_jv_cmd)

            self.rate.sleep()

    # Below is from https://github.com/ShahriarSefati/robot-constrained-control/blob/master/cmc_kinematics.py
    # Would be best to just do this straight from AMBF or defn files using pykdl or others. However, I beleive jacobian defns are different

    def set_dh(self, robot_type):
        if robot_type == 'UR5':
            # HTP - changes to DH here go to end effector, instead of to the origin of wrist_3_link
            self.d = np.array([0.089159, 0, 0, 0.10915, 0.09465, 0.0823])
            # self.d = np.array([0.089159, 0, 0, 0.10915, 0.09465, 0.0823+0.0750998])
            self.a = np.array([0, -0.425, -0.39225, 0, 0, 0])
            self.alph = np.array([np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0])
            # self.alph = np.array(
            #     [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, np.pi/2])

        if robot_type == 'UR10':
            self.d = np.array(
                [0.1273, 0, 0, 0.163941, 0.1157, 0.0922])              # UR10
            self.a = np.array([0, -0.612, -0.5723, 0, 0, 0]
                              )                         # UR10
            self.alph = np.array(
                [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0])                 # UR10

    def AH(self, n, th):
        T_a = np.identity(4)
        T_a[0, 3] = self.a[n]
        T_d = np.identity(4)
        T_d[2, 3] = self.d[n]

        Rzt = np.array([[np.cos(th[n]), -np.sin(th[n]), 0, 0],
                        [np.sin(th[n]), np.cos(th[n]), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        Rxa = np.array([[1, 0, 0, 0],
                        [0, np.cos(self.alph[n]), -np.sin(self.alph[n]), 0],
                        [0, np.sin(self.alph[n]), np.cos(self.alph[n]), 0],
                        [0, 0, 0, 1]])

        return T_d @ Rzt @ T_a @ Rxa

    def FK(self, th):
        th[0] -= np.pi  # This accounts for an offset in the definitions
        FK_T = np.eye(4)  # Starting the transformations
        Origins_T = []         # Storing the transformations from base to each joint

        for (i, t) in enumerate(th):
            Origins_T.append(FK_T)
            FK_T = FK_T @ self.AH(i, th)

        # compute Jacobian
        jac = np.zeros((6, 6))
        tip = FK_T[0:3, 3]

        for (k, origin) in enumerate(Origins_T):
            z_joint = origin[0:3, 2]  # 3rd column of the rotation part
            # 4th column of the transformation (translation component)
            o_joint = origin[0:3, 3]

            jac[0:3, k] = np.cross(z_joint, (tip - o_joint))
            jac[3:6, k] = z_joint

        fk_msg = PoseStamped()
        fk_msg.header.stamp = rospy.Time.now()
        # fk_msg.pose.position = FK_T[0:3,3]
        (fk_msg.pose.position.x, fk_msg.pose.position.y,
         fk_msg.pose.position.z) = FK_T[0:3, 3]

        q = Rotation.from_matrix(FK_T[0:3, 0:3]).as_quat()  # x,y,z,w

        (fk_msg.pose.orientation.x, fk_msg.pose.orientation.y,
         fk_msg.pose.orientation.z, fk_msg.pose.orientation.w) = q
        self.pub_measured_cp.publish(fk_msg)

        jac_msg = Float64MultiArray()
        jac_msg.layout
        jac_msg.layout.dim.append(
            MultiArrayDimension(label="rows", size=6, stride=1))
        jac_msg.layout.dim.append(
            MultiArrayDimension(label="cols", size=6, stride=6))
        jac_msg.layout.data_offset = 0
        jac_msg.data = jac.reshape((36,))
        # print(jac_msg)
        self.pub_jacobian.publish(jac_msg)
        # print(FK_T)
        # print(jac)
        return FK_T, jac


if __name__ == "__main__":
    # Create a instance of the client
    parser = ArgumentParser()
    # add an argument for initial joint positions of the robot (optional) - default is [0.0,-1.0,-1.0,0.0,-np.pi/8, np.pi/8]
    parser.add_argument('--init_jp', nargs=6, type=float, default=[
                        0.0, -1.0, 1.0, 0.0, -np.pi/8, np.pi/8])   
    #parse known and unknown arguments
    args, unknown = parser.parse_known_args()
    init_jp = args.init_jp
    print("Initial joint positions: ", init_jp)

    while (not rospy.is_shutdown()):
        _client = Client("ur5_ambf")
        _client.connect()
        ur5 = UR5_AMBF(_client, 'ur5')
        time.sleep(0.5)
        if ur5.base is not None:
            print("Found AMBF client and loaded")
            break
        else:
            print("Assuming ambf client still loading and waiting...")
            _client.clean_up()
            time.sleep(0.5)

    while (not rospy.is_shutdown()):
        while (ur5.base.get_num_joints() < 6):
            if rospy.is_shutdown():
                quit()
            print("Waiting for ur5 model to load...")
            time.sleep(0.5)
        print("UR5_AMBF loaded")
        # set init pose
        ur5.servo_jp(init_jp)
        time.sleep(2.0)  # let the move command finish
        ur5.run()
        _client.clean_up()
