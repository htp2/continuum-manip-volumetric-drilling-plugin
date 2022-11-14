from PyKDL import Frame, Rotation, Vector
import numpy as np
# import pandas as pd
import json
from scipy.optimize import minimize
import os 

traj_dir_name_anat = "/home/henry/exps/20221113_ipcai_directed/trajectories/anatomy"
traj_dir_name_ambf = "/home/henry/exps/20221113_ipcai_directed/trajectories/ambf"
traj_dir_name_rob = "/home/henry/exps/20221113_ipcai_directed/trajectories/robot"
# if traj_dir_name_rob does not exist, create it
if not os.path.exists(traj_dir_name_rob):
    os.mkdir(traj_dir_name_rob)

# find all files ending in .fcsv
import glob
traj_files = glob.glob(traj_dir_name_anat + "/*.csv")
# remove files with 'params' in the name
traj_files = [traj_file for traj_file in traj_files if 'params' not in traj_file]
#find filename before .csv
for traj_file in traj_files:
    traj_id = traj_file.split('/')[-1].split('.')[0]

    units_to_m = 1.0/10.0

    # path_to_config_dir = '/home/henry/catkin_ws/src/bigss_spine/spine_robot_control/config/sim_ur5/'
    speed_mps = 0.005
    cable_max = 0.26

    traj_params = np.loadtxt(f"{traj_dir_name_anat}/{traj_id}_params.csv", delimiter=",")
    roll_ang,w, turning_point, R_init_ang, R_initx, R_inity, R_initz = traj_params
    R_init_planning = Rotation.Rot(Vector(R_initx, R_inity, R_initz), R_init_ang)
    traj_in = np.loadtxt(f"{traj_dir_name_ambf}/{traj_id}.csv", delimiter=",")

    traj = traj_in[:, 0:3]
    traj *= units_to_m

    # set world_to_anat
    world_to_anat = Frame(Rotation.Quaternion(0.0, -0.706825181105366, 0.0, 0.7073882691671998), Vector(4.5, 4.5, 3.35)*units_to_m)
    # set world_to_robot
    world_to_robot = Frame(Rotation.RotX(0.0), Vector(0.0, 0.0, 0.0)) #Identity

    robot_to_anat = world_to_robot.Inverse()*world_to_anat

    # transform each point in trajectory to robot frame
    robot_traj = np.zeros((traj.shape))
    for i, pt in enumerate(traj):
        rob_pt = robot_to_anat * Vector(pt[0], pt[1], pt[2])
        robot_traj[i, 0] = rob_pt.x()
        robot_traj[i, 1] = rob_pt.y()
        robot_traj[i, 2] = rob_pt.z()
    # robot_traj = traj
    # R_fix = Rotation(0.0, 0.0, -1.0,-1.0, 0.0, 0.0,0.0, 1.0, 0.0)  #TODO: figure our why this is correct

    # # R = Rotation.RotZ(-np.pi/2)*R_init_planning
    # R_init = R_fix *R_init_planning

    # # R_init = R_init_planning

    # des_axis = R_init.UnitY()
    # pt_on_des_axis = robot_traj[0]


    # # TODO: account for the fact that the trajectory is in the anatomical frame
    def rotation_matrix_between_two_vectors(v1, v2):
        v1 = v1/np.linalg.norm(v1)
        v2 = v2/np.linalg.norm(v2)
        axis = np.cross(v1, v2)
        axis = axis/np.linalg.norm(axis)
        angle = np.arccos(np.dot(v1, v2))
        return Rotation.Rot(Vector(axis[0],axis[1],axis[2]), angle)

    v = robot_traj[1] - robot_traj[0]
    v/=np.linalg.norm(v)

    u = robot_traj[-1] - robot_traj[0]
    u/=np.linalg.norm(u)

    n = np.cross(v, u)
    n/=np.linalg.norm(n)

    y = v
    z = n
    x = np.cross(y, z)

    #sanity check
    x /= np.linalg.norm(x)
    y /= np.linalg.norm(y)
    z /= np.linalg.norm(z)

    R_init = Rotation(x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2])
    R_init = R_init.Inverse()
    des_axis = R_init.UnitY()
    pt_on_des_axis = robot_traj[0]

    # R_init = rotation_matrix_between_two_vectors([0,1,0],v1)
    # des_axis = R_init.UnitY()



    """
    # find unit vector between first two points
    v1 = traj[1] - traj[0]
    v1 /= np.linalg.norm(v1)

    # find point 50 back from the first point along v1
    p1 = traj[0] - 50*units_to_mm * v1

    # find rotation matrix that aligns x axis with v1
    # https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
    v2 = np.array([0, 1, 0])
    v = np.cross(v2, v1)
    s = np.linalg.norm(v)
    c = np.dot(v2, v1)
    v_x = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R_des = np.eye(3) + v_x + np.dot(v_x, v_x) * (1 - c) / (s ** 2)


    R = Rotation(R_des[0, 0], R_des[0, 1], R_des[0, 2], R_des[1, 0],
                R_des[1, 1], R_des[1, 2], R_des[2, 0], R_des[2, 1], R_des[2, 2])
    """



    def print_rot(str_out, R, prefix='                '):
        str_out.append(prefix+ '[')
        str_out.append(f"{prefix}[{R.UnitX()[0]}, {R.UnitY()[0]}, {R.UnitZ()[0]}],")
        str_out.append(f"{prefix}[{R.UnitX()[1]}, {R.UnitY()[1]}, {R.UnitZ()[1]}],")
        str_out.append(f"{prefix}[{R.UnitX()[2]}, {R.UnitY()[2]}, {R.UnitZ()[2]}]")
        str_out.append(prefix+ '],')
        return str_out


    def pykdl_from_file(filename):
        T = np.loadtxt(filename)
        return Frame(Rotation(T[0, 0], T[0, 1], T[0, 2], T[1, 0], T[1, 1], T[1, 2], T[2, 0], T[2, 1], T[2, 2]), Vector(T[0, 3], T[1, 3], T[2, 3]))


    def print_frame(str_out, F, prefix='                '):
        str_out.append(prefix+'{')
        str_out.append(prefix+'    "Rotation" :')
        str_out = print_rot(str_out, F.M, prefix+'    ')
        str_out.append(f"{prefix}\"Translation\" : [{F.p.x()}, {F.p.y()}, {F.p.z()}]")
        str_out.append(prefix+'}, ')
        return str_out

    # For AMBF spine
    # # path_to_config_dir = '/home/henry/catkin_ws/src/bigss_spine/spine_robot_control/config/ambf_spine/'
    # speed_mps = 0.005
    # cable_max = 0.26


    dax = des_axis
    pt = pt_on_des_axis
    kins_name = "system"
    kins_goal_name = "systemGoal"

    offset = Frame(Rotation(), Vector(0, 0.01, 0))
    # offset = ur_eef_to_PolarisToolMarker * polarisToolMarkerToSnakeTipZeroBend * SnakeTipZeroBendToBendPlane

    # str_out.append("pose: ", pose)

    # str_out.append("desired_orientation:", R)

    str_out = []

    str_out.append("{")
    str_out.append('    "constraints": ')
    str_out.append('    [')
    str_out.append('        {')
    str_out.append('            "type": "follow_position_trajectory",            ')
    str_out.append('            "name": "follow_position_trajectory", ')
    str_out.append(f"            \"goal_linear_velocity_m/s\": {speed_mps},")
    str_out.append(f"            \"current_kinematics\": \"{kins_name}\",")
    str_out.append(f"            \"goal_kinematics\": \"{kins_goal_name}\",")
    str_out.append('            "active": true')
    str_out.append('        },')

    str_out.append('        {')
    str_out.append('            "type": "absolute_joint_limits",            ')
    str_out.append('            "name": "absolute_joint_limits", ')
    str_out.append('            "lower_limits": [-6.283, -6.283, -6.283, -6.283, -6.283, -6.283, 0.0],')
    str_out.append(f"            \"upper_limits\": [6.283, 6.283, 6.283, 6.283, 6.283, 6.283, {cable_max}],")
    str_out.append(f"            \"kinematics\": \"{kins_name}\",")
    str_out.append('            "slack_costs": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 10.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 10.0],')
    str_out.append('            "slack_limits": [0.03, 0.03, 0.03, 0.03, 0.03, 0.03,   0.01,   0.03,0.03,0.03,0.03,0.03,0.03, 0.01],')
    str_out.append('            "active": true')
    str_out.append('        },')
    str_out.append('        {')
    str_out.append('           "type": "joint_velocity_limits",            ')
    str_out.append('           "name": "joint_velocity_limits", ')
    str_out.append('           "lower_limits": [-0.05, -0.05, -0.05, -0.05, -0.05, -0.05, -0.005],')
    str_out.append('           "upper_limits": [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.005],')
    str_out.append(f"            \"kinematics\": \"{kins_name}\",")
    str_out.append('           "active": true')
    str_out.append('        },')

    str_out.append('        {')
    str_out.append('           "type": "stay_on_axis",            ')
    str_out.append('           "name": "stay_on_axis",')
    str_out.append('           "kinematics_for_last_joint_that_moves_frame_of_interest": "URInterface",')
    str_out.append('           "offset_from_kinematics_to_frame_of_interest":')

    str_out = print_frame(str_out, offset, prefix='               ')

    str_out.append(f"           \"desired_axis\":  [{dax[0]},{dax[1]}, {dax[2]}],")
    str_out.append(f"           \"point_on_desired_axis\":  [{pt[0]},{pt[1]}, {pt[2]}],")
    str_out.append('           "num_joints_system": 7,')
    str_out.append('           "importance_gain": 2.0,')
    str_out.append('           "active": true')
    str_out.append('        },')
    str_out.append('        {')
    str_out.append('           "type": "fix_orientation",            ')
    str_out.append('           "name": "fix_orientation",')
    str_out.append('           "kinematics_for_last_joint_that_moves_frame_of_interest": "URInterface",')
    str_out.append('           "offset_from_kinematics_to_frame_of_interest":')

    str_out = print_frame(str_out,offset, prefix='               ')

    str_out.append('           "desired_orientation": ')

    str_out = print_rot(str_out,R_init)

    str_out.append('           "num_joints_system": 7,')
    str_out.append('           "importance_gain": 1.0,')
    str_out.append('           "active": true')
    str_out.append('        }         ')
    str_out.append('        ')
    str_out.append('    ]')
    str_out.append('}')

    out = "\n".join(str_out)
    #save constraint config to json file
    with open(f"{traj_dir_name_rob}/{traj_id}_constraints_config.json", "w") as text_file:
        text_file.write(out)


    #append 0 pi 0 to end of each pt in traj
    out_traj = np.zeros((traj.shape[0], traj.shape[1]+3))
    for i in range(len(robot_traj)):
        out_traj[i] = np.append(robot_traj[i], [0, np.pi, 0])

    #save trajectory to csv file
    np.savetxt(f"{traj_dir_name_rob}/{traj_id}_traj.csv", out_traj, delimiter=",")
