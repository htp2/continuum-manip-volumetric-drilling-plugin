import numpy as np
import pandas as pd
import json
from scipy.optimize import minimize

# this is just to prevent pylance from complaining about np.cross not always returning
# https://github.com/microsoft/pylance-release/issues/3277
def np_cross(a:np.ndarray,b:np.ndarray)->np.ndarray:
    return np.cross(a,b)

def read_points_from_markups_json(filename):
    # read in json file
    with open(filename) as f:
        data = json.load(f)
    # get the first markup point
    markup = data["markups"][0]
    # get the first control point
    control_points = markup["controlPoints"]
    points = np.zeros((len(control_points), 3))
    # get the position
    for i, control_point in enumerate(control_points):
        points[i, :] = control_point["position"]
    return np.array(points)

#numpy rotation matrix
def rot(theta, axis):
    return np.array([[np.cos(theta) + axis[0]**2*(1-np.cos(theta)), axis[0]*axis[1]*(1-np.cos(theta)) - axis[2]*np.sin(theta), axis[0]*axis[2]*(1-np.cos(theta)) + axis[1]*np.sin(theta)],
                     [axis[1]*axis[0]*(1-np.cos(theta)) + axis[2]*np.sin(theta), np.cos(theta) + axis[1]**2*(1-np.cos(theta)), axis[1]*axis[2]*(1-np.cos(theta)) - axis[0]*np.sin(theta)],
                     [axis[2]*axis[0]*(1-np.cos(theta)) - axis[1]*np.sin(theta), axis[2]*axis[1]*(1-np.cos(theta)) + axis[0]*np.sin(theta), np.cos(theta) + axis[2]**2*(1-np.cos(theta))]])


# calculate distance of point to plane
def distance_to_plane(point, plane):
    normal, point_on_plane = plane
    return np.dot(point - point_on_plane, normal)

# # define a plane given three points
# def plane_from_points(p1, p2, p3):
#     n = np.cross(p2 - p1, p3 - p1)
#     n /= np.linalg.norm(n)
#     return n, p1

def kins(p,R, v, w_last, a):
    dt = 0.01
    pose_next = p + R@np.array((0, v*dt, 0))
    axis = np.array((0, 0, 1))
    w = w_last + a*dt
    R_next = R @ rot(w*dt, axis)
    return pose_next, R_next, w


# # trajectory cost function
def cost(traj, plane_top, plane_bottom):
    cost = 0
    for p in traj:
        cost += distance_to_plane(p, plane_top)**2
        # cost += distance_to_plane(p, plane_bottom)**2
    return cost

# penalize cloesness to points_top and points_bottom
def p_cost(traj, points_top, points_bottom):
    cost = 0
    for p in traj:
        for p_top in points_top:
            cost += np.linalg.norm(p - p_top)
        for p_bottom in points_bottom:
            cost += np.linalg.norm(p - p_bottom)
    return cost



def trajectory_from_params(Rx_comp, Ry_comp, Rz_comp, w, turning_point, entry, target, points_top, points_bottom, path_len,v, R_sl,entry_vec):
    # define initial pose
    # define initial rotation from init roll, pitch, yaw
    axis = np.array((Rx_comp, Ry_comp, Rz_comp))
    ang = np.linalg.norm(axis)
    if ang>0:
        axis /= ang
        R_init = R_sl @ rot(ang, axis)
    else:
        R_init = R_sl
    # R_init = rot(init_roll_angle, np.array((1, 0, 0))) @ rot(init_pitch_angle, np.array((0, 1, 0))) @ rot(init_yaw_angle, np.array((0, 0, 1)))
    p_init = entry
    # R_init = rot(init_roll_angle, np.array((1, 0, 0)))
    # plan trajectory
    traj = propogate_trajectory(p_init, R_init, v, w, turning_point, path_len, target)
    # calculate cost
    # pl_cost = p_cost(traj, plane_top, plane_bottom)
    # pcm = p_cost_midpoint(traj, points_top, points_bottom)
    pcm = p_cost_first_pt_break_plane(traj, points_top, points_bottom, entry_vec)
    # return trajectory and cost
    # calculate error to target
    error = np.linalg.norm(traj[-1] - target)

    total_cost = pcm + 3*error
    # total_cost = error
    return traj, total_cost

# penalize distance closest point on trajectory is to midpoint between points_top and points_bottom
def p_cost_midpoint(traj, points_top, points_bottom):
    p_cost = 0
    num = min(len(points_top), len(points_bottom))
    for i in range(num):
        midpoint = (points_top[i] + points_bottom[i])/2
        # find closest point on trajectory to midpoint
        closest_point = np.array((np.inf, np.inf, np.inf))
        for p in traj:
            if np.linalg.norm(p - midpoint) < np.linalg.norm(closest_point - midpoint):
                closest_point = p
        p_cost+= np.linalg.norm(closest_point - midpoint)
    return p_cost

# # find when a trajectory crosses a plane
# def find_crossing(traj, points_top, points_bottom, entry_vec):
#     num = min(len(points_top), len(points_bottom))
#     for i in range(num):
#         midpoint = (points_top[i] + points_bottom[i])/2
#         # define second point on plane
#         line = points_top[i] - points_bottom[i]
#         second_vec = np_cross(entry_vec,line)
#         plane_normal = np_cross(line, second_vec)
#         plane_normal /= np.linalg.norm(plane_normal)
#         # find first point on trajectory where 
#         plane_from_point_and_normal = (midpoint,plane_normal)
#         plane_side_last = np.dot(plane_from_point_and_normal, np.append(traj[0],1))
#         for p in traj[1:]:
#             plane_side = np.dot(plane_from_point_and_normal, np.append(p,1)) > 0
#             if plane_side != plane_side_last:
#                 return p

def p_cost_first_pt_break_plane(traj, points_top, points_bottom, entry_vec):
    p_cost = 0
    num = min(len(points_top), len(points_bottom))
    for i in range(num):
        midpoint = (points_top[i] + points_bottom[i])/2
        # define second point on plane
        line = points_top[i] - points_bottom[i]
        second_vec = np_cross(entry_vec,line)
        plane_normal = np_cross(line, second_vec)
        plane_normal /= np.linalg.norm(plane_normal)
        # find first point on trajectory where 
        plane_eqn = plane_from_point_and_normal(midpoint,plane_normal)
        plane_side_last = np.dot(plane_eqn, np.append(traj[0],1))
        found_point = False
        for p in traj[1:]:
            plane_side = np.dot(plane_eqn, np.append(p,1)) > 0
            if plane_side != plane_side_last:
                p_cost += np.linalg.norm(p - midpoint)
                found_point = True
                break
        if not found_point:
            p_cost += np.linalg.norm(entry_vec- midpoint) # has to be something big
    return p_cost


# get equation of plane a b c d for point and normal
def plane_from_point_and_normal(point, normal):
    normal = normal/np.linalg.norm(normal)
    return np.append(normal, -np.dot(normal, point))

def rotation_matrix_between_two_vectors(v1, v2):
    v1 = v1/np.linalg.norm(v1)
    v2 = v2/np.linalg.norm(v2)
    axis = np_cross(v1, v2)
    axis = axis/np.linalg.norm(axis)
    angle = np.arccos(np.dot(v1, v2))
    return rot(angle, axis)

# propogate trajectory
def propogate_trajectory(p_init, R_init, v, a, turning_point, max_path_len, target):
    p = p_init
    R = R_init
    traj = [p]
    total_dist = 0
    w_last = 0
    while total_dist < max_path_len:
        if total_dist < turning_point:
            a_in = 0
        else:
            a_in = a
        p,R,w_last = kins(p, R, v, w_last, a_in)
        traj.append(p)
        # find distance between p and last point in traj
        total_dist += np.linalg.norm(p - traj[-2])
        # break out of while if p is close to target
        if np.linalg.norm(p - target) < 0.1:
            break

    return np.array(traj)

# find initial roll angle w and turning point that minimizes cost
def find_optimal_params(points_top, points_bottom, entry, target, path_len, v, R_ev, entry_vec):
    # define bounds
    w_bounds = (0.0, 0.1)
    bend_len = 35
    first = path_len - bend_len
    last = path_len - bend_len/2
    turning_point_bounds = (first, last)
    cone_ang = np.deg2rad(20)
    Rx_comp_bounds = (-1.0*cone_ang,1.0*cone_ang)
    Ry_comp_bounds = (-2.0*np.pi,2.0*np.pi)
    Rz_comp_bounds = (-1.0*cone_ang,1.0*cone_ang)
    # define initial guess
    #roll pitch yaw w turning_point
    x0 = np.array((0.0,-np.deg2rad(45),0.0,0.01,(first+last)/2 ))
    # define function to minimize
    def cost_function(x):
        traj, total_cost = trajectory_from_params(x[0], x[1], x[2], x[3], x[4], entry, target, points_top, points_bottom, path_len, v, R_ev, entry_vec)
        return total_cost
    # minimize cost function
    X = minimize(cost_function, x0, method='Nelder-Mead', bounds=(Rx_comp_bounds,Ry_comp_bounds, Rz_comp_bounds, w_bounds, turning_point_bounds), options={'maxiter': 100000})
    # return optimal parameters
    # basin hopping
    # from scipy.optimize import basinhopping
    # X = basinhopping(cost_function, x0, minimizer_kwargs={'method': 'Nelder-Mead', 'bounds': (init_roll_angle_bounds,init_pitch_angle_bounds, init_yaw_angle_bounds, w_bounds, turning_point_bounds)}, niter=1000)
    return X.x


def find_opt_traj_for_entry_and_target(ie, it, entry, target, entry_vec, markup_dir, points_top, points_bottom):
    save_name = f"{markup_dir}/traj_entry{ie}_target{it}"
    print(f"finding optimal trajectory for entry {ie} and target {it}")
    # # align (target-entry) with y axis
    # sl = target - entry
    # sl = sl/np.linalg.norm(sl)
    # # find rotation matrix to align sl with y axis
    # R_sl = rotation_matrix_between_two_vectors(sl, np.array((0, 1, 0)))
    
    # align entry_vec with y axis
    entry_vec = entry_vec/np.linalg.norm(entry_vec)
    # find rotation matrix to align entry_vec with y axis
    R_entry_vec = rotation_matrix_between_two_vectors(np.array((0, 1, 0)),entry_vec)
    
    # find optimal parameters
    path_len = 80
    v = 5
    optimal_params = find_optimal_params(points_top, points_bottom, entry, target, path_len,v,R_entry_vec,entry_vec)

    # propogate trajectory with optimal parameters
    traj = trajectory_from_params(optimal_params[0], optimal_params[1], optimal_params[2], optimal_params[3], optimal_params[4], entry, target, points_top, points_bottom, path_len, v, R_entry_vec, entry_vec)[0]

    #plot trajectory in 3d
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # ax.plot(traj[:, 0], traj[:, 1], traj[:, 2])

    # 3d scatterplot of trajectory
    ax.scatter(traj[:, 0], traj[:, 1], traj[:, 2], c='b', marker='o')
    # put labels on axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # add planes
    from matplotlib.patches import Polygon
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    def plot_plane(plane, ax, color='r'):
        normal, point_on_plane = plane
        d = -point_on_plane.dot(normal)
        xx, yy = np.meshgrid(range(-150, 150), range(-150, 150))
        z = (-normal[0] * xx - normal[1] * yy - d) * 1. / normal[2]
        verts = [list(zip(xx.flatten(), yy.flatten(), z.flatten()))]
        ax.add_collection3d(Poly3DCollection(verts, facecolors=color, linewidths=1, edgecolors='k', alpha=.25))
    # add planes to plot
    # plot_plane(plane_top, ax, color='r')
    # plot_plane(plane_bottom, ax, color='b')

    #plot points_top
    ax.scatter(points_top[:, 0], points_top[:, 1], points_top[:, 2], c='r', marker='o')
    #plot points_bottom
    ax.scatter(points_bottom[:, 0], points_bottom[:, 1], points_bottom[:, 2], c='b', marker='o')

    # add entry and target
    ax.scatter(entry[0], entry[1], entry[2], color='r', s=100)
    ax.scatter(target[0], target[1], target[2], color='g', s=100)

    # 3d axis equal
    max_range = np.array([traj[:, 0].max()-traj[:, 0].min(), traj[:, 1].max()-traj[:, 1].min(), traj[:, 2].max()-traj[:, 2].min()]).max() / 2.0
    mid_x = (traj[:, 0].max()+traj[:, 0].min()) * 0.5
    mid_y = (traj[:, 1].max()+traj[:, 1].min()) * 0.5
    mid_z = (traj[:, 2].max()+traj[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    plt.savefig(f"{save_name}.png")
    plt.close(fig)

    # resample trajectory to 100 points
    from scipy.interpolate import interp1d
    t = np.linspace(0, 1, len(traj))
    t_new = np.linspace(0, 1, 100)
    traj_interp = np.zeros((100, 3))
    for i in range(3):
        f = interp1d(t, traj[:, i])
        traj_interp[:, i] = f(t_new)

    # calculate total length of trajectory
    total_length = 0
    for i in range(1, len(traj_interp)):
        total_length += np.linalg.norm(traj_interp[i] - traj_interp[i-1])

    Rx_comp = optimal_params[0]
    Ry_comp = optimal_params[1]
    Rz_comp = optimal_params[2]
    axis = np.array((Rx_comp, Ry_comp, Rz_comp))
    ang = np.linalg.norm(axis)
    axis /= ang
    R_init =  R_entry_vec  @ rot(ang, axis)
        
        #recover axis angle from rotation matrix
    from scipy.spatial.transform import Rotation as scipyRot
    R_init = scipyRot.from_matrix(R_init)
    axis = R_init.as_rotvec()
    init_ang = np.linalg.norm(axis)
    init_axis = axis / init_ang

    # traj to csv
    np.savetxt(f"{save_name}.csv", traj_interp, delimiter=",")

    np.savetxt(f"{save_name}.fcsv", traj_interp, delimiter=",")
    # prepend 3 lines to csv
    with open(f"{save_name}.fcsv", "r+") as f:
        content = f.read()
        lines = content.splitlines()
        new_lines = []
        for i,line in enumerate(lines):
            new_lines.append(f"{i+1},{line},0,0,0,1,1,1,0,F-{i+1},,vtkMRMLLabelMapVolumeNode1")
        new_content = '\n'.join(new_lines)
        f.seek(0, 0)
        f.write("# Markups fiducial file version = 4.11\n# CoordinateSystem = LPS\n# columns = id,x,y,z,ow,ox,oy,oz,vis,sel,lock,label,desc,associatedNodeID)\n" + new_content)
    with open(f"{save_name}_params.csv", "w") as f:
        f.write(f"{optimal_params[0]},{optimal_params[1]},{optimal_params[2]},{optimal_params[3]},{optimal_params[4]},{init_ang},{init_axis[0]},{init_axis[1]},{init_axis[2]}")


def main():
    #load entry points from json file
    markup_dir = "/home/henry/exps/20221113_ipcai_planned_trajs2/slicer"

    points_top = read_points_from_markups_json(f"{markup_dir}/plane_top.mrk.json")
    points_bottom = read_points_from_markups_json(f"{markup_dir}/plane_bottom.mrk.json")

    # entry_points = read_points_from_markups_json(f"{markup_dir}/Entry.mrk.json")
    target_points = read_points_from_markups_json(f"{markup_dir}/Targets.mrk.json")

    entry_vec_points_all = read_points_from_markups_json(f"{markup_dir}/EntryVecs.mrk.json")

    # temp: only use first target point and first two entry_vec_points_all
    # entry_vec_points_all = entry_vec_points_all[:4]
    # target_points = target_points[0:2]

    # subtract every second entry_vec_point from every first entry_vec_point
    entry_vec = []
    entry_points_in = []
    for i in range(0, len(entry_vec_points_all), 2):
        entry_vec.append(entry_vec_points_all[i+1] - entry_vec_points_all[i])
        entry_points_in.append(entry_vec_points_all[i])



    in_structs = []
    for ie, entry in enumerate(entry_points_in):
        for it, target in enumerate(target_points):
            in_structs.append((ie, it, entry, target, entry_vec[ie],markup_dir,points_top,points_bottom))

    num_threads = 12
    import multiprocessing
    with multiprocessing.Pool(num_threads) as pool:
        pool.starmap(find_opt_traj_for_entry_and_target, in_structs)
    # find_opt_traj_for_entry_and_target(*in_structs[0])

    print('done')

if __name__ == "__main__":
    main()

# get all 3d slicer markup nodes
# markups = slicer.util.getNodesByClass('vtkMRMLMarkupsFiducialNode')
# for m in markups:
#     # turn off all text lables
#     m.SetDisplayVisibility(False)
#     # turn off all text lables

