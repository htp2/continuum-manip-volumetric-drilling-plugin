#!/usr/bin/env python2

import rospy

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from ambf_msgs.msg import RigidBodyState
import numpy as np
import matplotlib.pyplot as plt
# from scipy.spatial.transform import Rotation
import tf.transformations as tr
import time
import os

# import scipy.optimize
from mpl_toolkits.mplot3d import Axes3D

class free_space_calibration:
    def __init__(self, save_dir):
        self.save_dir = save_dir
        rospy.init_node('free_space_calibration', anonymous=True)
        bend_sub  = rospy.Subscriber('/ambf/volumetric_drilling/cable_pull_measured', Float32, self.bend_sub_callback)
        self.bend_pub = rospy.Publisher('m/ambf/volumetric_drilling/cable_pull_goal', Float32)
        base_pos_sub  = rospy.Subscriber('ambf/env/snake_stick/State', RigidBodyState, self.tip_pos_sub_callback)
        tip_pos_sub  = rospy.Subscriber('ambf/env/seg27/State', RigidBodyState, self.base_pos_sub_callback)
        
        self.collect = False
        self.reset_sublists()

        
        self.ms = 0.001
        self.sec = 1.0
        self.base_transform = TransformStamped()
        self.tip_transform = TransformStamped()
        self.bend_length = 0.0

        self.tip_zero_transform = np.eye(4)
        self.base_zero_transform = np.eye(4)   
        
        self.base_transforms_measured_all = []
        self.tip_transforms_measured_all = []
        self.base_transforms_measured_avg = []
        self.tip_transforms_measured_avg = []
        self.bend_motor_pos_all = []




    def run(self):
       do_run = True
       while do_run:
            user_in = input("Press Enter to collect bend datapoint, 'X' to exit, 'F' to fit: , 'Z' to set zero transform, 'A' to auto calibrate: ")
            
            if user_in == "A": 
                reps = 2
                motor_pos = np.linspace(0,0.025,100)
                motor_pos_rev = np.flipud(motor_pos)
                pos = np.append(motor_pos,motor_pos_rev)
                for r in range(reps):
                    for p in pos:
                        self.bend_pub.publish(Float32(p))
                        rospy.sleep(2000*self.ms)
                        self.collect_over_period(500*self.ms) 
                        self.save_to_output()
                continue



            if user_in == "X": break

            if user_in == "F":
                self.fit_calibration()
                # self.fit_polynomials_to_TS(self.bend_lengths_measured, self.tip_transforms_measured, self.tip_zero_transform, 3)
                continue

            self.collect_over_period(2000*self.ms) 
            # avg_base, avg_tip, avg_bend = self.collect_and_average(1000*self.ms, 20*self.ms) 

            if user_in == 'Z':
                self.base_zero_transform = self.base_transforms_measured_avg[-1]
                self.tip_zero_transform = self.tip_transforms_measured_avg[-1]

            self.save_to_output()

    def collect_over_period(self, collect_duration):
        # Collect data
        self.reset_sublists()
        self.collect = True
        rospy.sleep(collect_duration)
        self.collect = False

        # Store data internally
        self.base_transforms_measured_all.append(self.base_transforms_sublist)
        self.tip_transforms_measured_all.append(self.tip_transforms_sublist)
        self.fbg_raw_wavelengths_measured_all.append(self.fbg_raw_wavelengths_sublist)
        self.bend_motor_pos_all.append(self.bend_motor_pos_sublist)

        self.base_transforms_measured_avg.append(self.average_list_of_TransformStamped(self.base_transforms_sublist))
        self.tip_transforms_measured_avg.append(self.average_list_of_TransformStamped(self.tip_transforms_sublist))



    def collect_and_average(self, collect_time, period):
        rospy.sleep(0.5) #ensure new values
        start_time = rospy.Time.now()
        collect_duration =  rospy.Duration(collect_time)
        period_duration = rospy.Duration(period)
        base_transforms_collected = []
        tip_transforms_collected = []
        bend_lengths_collected = []       
        while (rospy.Time.now() - start_time < collect_duration):
            loop_start_time = rospy.Time.now() # TODO: implement using rosTimer
            while (rospy.Time.now() - loop_start_time < period_duration):
                base_transforms_collected.append(self.base_transform)
                tip_transforms_collected.append(self.tip_transform)
                bend_lengths_collected.append(self.bend_length)


        averaged_base = self.average_list_of_TransformStamped(base_transforms_collected)
        averaged_tip = self.average_list_of_TransformStamped(tip_transforms_collected)
        averaged_bend = sum(bend_lengths_collected)/len(bend_lengths_collected)

        return (averaged_base, averaged_tip, averaged_bend)


    def fit_polynomials_to_TS(self, lengths, TS, zero_TS, degree):

        x = np.zeros(len(lengths))
        y = np.zeros(len(lengths))
        z = np.zeros(len(lengths))
        thy = np.zeros(len(lengths))
        print("bend lengths: ", lengths)
        print("zero_TS: ", zero_TS)
        zero_T_inv = np.linalg.inv(zero_TS)


        for i in range(len(lengths)):
            # Q = np.matmul(zero_T_inv,  TS[i])
            Q =  np.linalg.inv( np.matmul(np.linalg.inv(TS[i]),zero_TS) )
            x[i]=Q[0,3]
            y[i]=Q[1,3]
            z[i]=Q[2,3]
            print(Q)
            th,dir,_ = tr.rotation_from_matrix(Q)
            thy[i] = th * np.sign(dir[1])
            print(dir)

        l = np.array(lengths)

        x_coeff = np.polyfit(l, x, degree)
        y_coeff = np.polyfit(l, y, degree)
        z_coeff = np.polyfit(l, z, degree)
        thy_coeff = np.polyfit(l, thy, degree)

        poly_x = np.poly1d(x_coeff)
        poly_y = np.poly1d(y_coeff)
        poly_z = np.poly1d(z_coeff)
        poly_thy = np.poly1d(thy_coeff)

        new_l = np.linspace(min(l), max(l))
        new_x = poly_x(new_l)
        new_y = poly_y(new_l)
        new_z = poly_z(new_l)
        new_thy = poly_thy(new_l)

        fig, (ax_x, ax_y, ax_z, ax_thy) = plt.subplots(4)
        ax_x.plot(l, x, "o", new_l, new_x)
        ax_x.set(xlabel="cable length (mm)", ylabel="Tip X value (m)")
        ax_y.plot(l, y, "o", new_l, new_y)
        ax_y.set(xlabel="cable length (mm)", ylabel="Tip Y value (m)")
        ax_z.plot(l, z, "o", new_l, new_z)
        ax_z.set(xlabel="cable length (mm)", ylabel="Tip Z value (m)")
        ax_thy.plot(l, thy, "o", new_l, new_thy)
        ax_thy.set(xlabel="cable length (mm)", ylabel="Tip Rotation Angle (th_y) (rad)")        

        timestamp = time.strftime("%Y%m%d%H%M%S_")
        save_filename = self.save_dir+timestamp+"xyzthy_snake_polyfit_coeffs.txt"
        np.savetxt(save_filename,np.array((x_coeff,y_coeff,z_coeff,thy_coeff)))
        print("Coefficients saved to " + save_filename)
        print("Close Graph to Continue:")
        plt.show()

    def average_list_of_TransformStamped(self,list_of_TS):
        N = len(list_of_TS)
        average = TransformStamped()
        average.transform.rotation.w = sum([T.transform.rotation.w for T in list_of_TS])/N
        average.transform.rotation.x = sum([T.transform.rotation.x for T in list_of_TS])/N
        average.transform.rotation.y = sum([T.transform.rotation.y for T in list_of_TS])/N
        average.transform.rotation.z = sum([T.transform.rotation.z for T in list_of_TS])/N
        average.transform.translation.x = sum([T.transform.translation.x for T in list_of_TS])/N
        average.transform.translation.y = sum([T.transform.translation.y for T in list_of_TS])/N
        average.transform.translation.z = sum([T.transform.translation.z for T in list_of_TS])/N

        R = average.transform.rotation
        p = average.transform.translation
        q = np.array([R.x,R.y,R.z,R.w])
        q = q / np.linalg.norm(q)
        g = tr.quaternion_matrix(q)
        g[0:3,-1] = np.array([p.x,p.y,p.z])

        return g

    def base_pos_sub_callback(self,ambf_rigid_body_state):
        
        self.base_transform = ambf_rigid_body_state_to_transform_stamped(ambf_rigid_body_state)

    def tip_pos_sub_callback(self,ambf_rigid_body_state):
        self.tip_transform = ambf_rigid_body_state_to_transform_stamped(ambf_rigid_body_state)

    def bend_sub_callback(self,length):
        self.bend_length = length.data

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
        
        calibration_save_directory = os.path.dirname(os.path.abspath(__file__))+"/output/"
        if not os.path.isdir(calibration_save_directory):
            os.mkdir(calibration_save_directory)
        cal_node = free_space_calibration(calibration_save_directory)
        cal_node.run()

    except rospy.ROSInterruptException:
        pass

