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

def kins(p,R, v, w_last, a, th_last):
    dt = 0.1
    pose_next = p + R@np.array((0, v*dt, 0))
    axis = np.array((0, 0, 1))
    w = w_last + a*dt
    th = th_last + w*dt
    R_next = R @ rot(w*dt, axis)
    return pose_next, R_next, w, th


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



def trajectory_from_params(roll_angle, w, turning_point, entry, target, neck_midpoint, path_len,v, R_ev, entry_vec):
    # define initial pose
    # define initial rotation from init roll, pitch, yaw
    axis = np.array((0, roll_angle, 0))
    ang = np.linalg.norm(axis)
    if ang>0:
        axis /= ang
        R_init = R_ev @ rot(ang, axis)
    else:
        R_init = R_ev
    p_init = entry

    # plan trajectory
    traj = propogate_trajectory(p_init, R_init, v, w, turning_point, path_len, target)
    
    # calculate cost
    # mid_cost = cost_neck_midpoint(traj, neck_midpoint)
    error = np.linalg.norm(traj[-1] - target)
    # total_cost = mid_cost + 100*error
    total_cost = error
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

# penalize distance closest point on trajectory is neck_midpoint
def cost_neck_midpoint(traj, neck_midpoint):
    p_cost = 0
    # find closest point on trajectory to midpoint
    closest_point = np.array((np.inf, np.inf, np.inf))
    for p in traj:
        if np.linalg.norm(p - neck_midpoint) < np.linalg.norm(closest_point - neck_midpoint):
            closest_point = p
    p_cost+= np.linalg.norm(closest_point - neck_midpoint)
    return p_cost



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

def propogate_trajectory(p_init, R_init, v, w, turning_point, max_path_len, target):
    p = p_init
    R = R_init
    traj = [p]
    total_dist = 0
    w_last = 0
    th_last = 0
    while total_dist < max_path_len:
        if total_dist < turning_point:
            w_in = 0
        else:
            w_in = w
        p,R,w_last,th_last = kins(p, R, v, w_last, w_in, th_last)
        traj.append(p)
        # find distance between p and last point in traj
        total_dist += np.linalg.norm(p - traj[-2])
        # break out of while if p is close to target
        if np.linalg.norm(p - target) < 0.1:
            break
        if np.abs(th_last) > np.pi/2: # bending too far
            break

    return np.array(traj)

def propogate_trajectory_a(p_init, R_init, v, a, turning_point, max_path_len, target):
    p = p_init
    R = R_init
    traj = [p]
    total_dist = 0
    w_last = 0
    th_last = 0
    while total_dist < max_path_len:
        if total_dist < turning_point:
            a_in = 0
        else:
            a_in = a
        p,R,w_last,th_last = kins(p, R, v, w_last, a_in, th_last)
        traj.append(p)
        # find distance between p and last point in traj
        total_dist += np.linalg.norm(p - traj[-2])
        # break out of while if p is close to target
        if np.linalg.norm(p - target) < 0.1:
            break
        if np.abs(th_last) > np.pi/2: # bending too far
            break

    return np.array(traj)

# find initial roll angle w and turning point that minimizes cost
def find_optimal_params(neck_midpoint, entry, target, path_len, v, R_ev, entry_vec):
    # define bounds
    w_bounds = (0.0, 0.05)
    bend_len = 35
    turning_point_min = path_len - bend_len
    turning_point_max = path_len - (bend_len/2)
    turning_point_bounds = (turning_point_min, turning_point_max)
    roll_bounds = (-2.0*np.pi,2.0*np.pi)
    # define initial guess
    #roll pitch yaw w turning_point
    x0 = np.array((0.0,0.01,(turning_point_min+turning_point_max)/2 ))
    # define function to minimize
    def cost_function(x):
        traj, total_cost = trajectory_from_params(x[0], x[1], x[2], entry, target, neck_midpoint, path_len, v, R_ev, entry_vec)
        return total_cost
    # minimize cost function
    # rng = np.random.default_rng()
    
    # steps = (10,10,10)
    # x0 = np.array((rng.uniform(roll_bounds[0], roll_bounds[1], steps[0]), rng.uniform(w_bounds[0], w_bounds[1], steps[1]), rng.uniform(turning_point_bounds[0], turning_point_bounds[1], steps[2])))
    
    # for i in range(steps[0]):
    #     for j in range(steps[1]):
    #         for k in range(steps[2]):
    #             x0 = np.array((x0[0][i], x0[1][j], x0[2][k]))
    #             res = minimize(cost_function, x0, method='Nelder-Mead', options={'xatol': 1e-8, 'disp': True})
    #             x0 = res.x
    # brute force method
    from scipy.optimize import brute
    X = brute(cost_function, (roll_bounds, w_bounds, turning_point_bounds), Ns=10, full_output=False)
    return X

    # X = minimize(cost_function, x0, method='Nelder-Mead', bounds=(roll_bounds, w_bounds, turning_point_bounds), callback=print_fun)
    
    # return optimal parameters
    # basin hopping
    # from scipy.optimize import basinhopping
    # X = basinhopping(cost_function, x0, minimizer_kwargs={'method':'Nelder-Mead', 'bounds':(roll_bounds, w_bounds, turning_point_bounds)}, niter=10)
    # X = basinhopping(cost_function, x0, minimizer_kwargs={'method': 'Nelder-Mead', 'bounds': (init_roll_angle_bounds,init_pitch_angle_bounds, init_yaw_angle_bounds, w_bounds, turning_point_bounds)}, niter=1000)
    # X = basinhopping(cost_function, x0, minimizer_kwargs={'method':'Nelder-Mead', 'bounds':(roll_bounds, w_bounds, turning_point_bounds)},
                # niter=10, callback=print_fun, seed=rng)

    return X.x

# def print_fun(x, f, accepted):
        # print(f"x: {x}, at minimum {f} accepted {accepted}")

def print_fun(x):
        print(f"x: {x}")

def find_opt_traj_for_entry_and_target(ie, it, entry, target, entry_vec, markup_dir, neck_midpoint):
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
    path_len = 100
    v = 5
    optimal_params = find_optimal_params(neck_midpoint, entry, target, path_len, v, R_entry_vec, entry_vec)

    # propogate trajectory with optimal parameters
    traj = trajectory_from_params(optimal_params[0], optimal_params[1], optimal_params[2], entry, target, neck_midpoint, path_len, v, R_entry_vec, entry_vec)[0]

    #plot trajectory in 3d
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # ax.plot(traj[:, 0], traj[:, 1], traj[:, 2])

    # 3d scatterplot of trajectory
    traj = traj.squeeze()
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

    ax.scatter(neck_midpoint[0], neck_midpoint[1], neck_midpoint[2], c='r', marker='o')
    #plot points_bottom
    # ax.scatter(points_bottom[:, 0], points_bottom[:, 1], points_bottom[:, 2], c='b', marker='o')

    # add entry and target
    entry = entry.squeeze()
    ax.scatter(entry[0], entry[1], entry[2], color='r', s=100)
    target = target.squeeze()
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

    Ry_comp = optimal_params[0]
    axis = np.array((0, Ry_comp, 0))
    ang = np.linalg.norm(axis)
    axis /= ang
    R_init =  R_entry_vec  @ rot(ang, axis)
        
        #recover axis angle from rotation matrix
    from scipy.spatial.transform import Rotation as scipyRot
    R_init = scipyRot.from_matrix(R_init)
    
    axis = R_init.as_rotvec()
    init_ang = np.linalg.norm(axis)
    if init_ang != 0:
        init_axis = axis / init_ang
    else:
        init_axis = np.array((0, 0, 1)) # arbitrary axis


    with open(f"{save_name}_params.csv", "w") as f:
        f.write(f"{optimal_params[0]},{optimal_params[1]},{optimal_params[2]},{init_ang},{init_axis[0]},{init_axis[1]},{init_axis[2]}")
    save_to_fcsv(traj_interp, f"{save_name}")

def save_to_fcsv(points, save_name):
    # traj to csv
    np.savetxt(f"{save_name}.csv", points, delimiter=",")

    np.savetxt(f"{save_name}.fcsv", points, delimiter=",")
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


def main():
    #load entry points from json file
    markup_dir = "/home/henry/exps/20221113_ipcai_directed/slicer"
    save_dir = "/home/henry/exps/20221113_ipcai_directed/trajectories/anatomy"
    import os
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    # os.makedirs(save_dir)
    points_top = read_points_from_markups_json(f"{markup_dir}/plane_top.mrk.json")
    points_bottom = read_points_from_markups_json(f"{markup_dir}/plane_bottom.mrk.json")

    # entry_points = read_points_from_markups_json(f"{markup_dir}/Entry.mrk.json")
    target_points = read_points_from_markups_json(f"{markup_dir}/Targets.mrk.json")

    entry_vec_points_all = read_points_from_markups_json(f"{markup_dir}/EntryVecs.mrk.json")

    # temp: only use first target point and first two entry_vec_points_all
    # entry_vec_points_all = entry_vec_points_all[:4]
    # target_points = target_points[0:2]

    # get femural neck points
    femoral_neck_points = read_points_from_markups_json(f"{markup_dir}/FemoralNeckSurface.mrk.json")
    
    # # multiply x and y components by -1 to convert from LPS to RAS
    # femoral_neck_points[:, 0] *= -1
    # femoral_neck_points[:, 1] *= -1

    # fit a cylinder to femoral neck points
    from scipy.optimize import leastsq


    """
    This is a fitting for a vertical cylinder fitting
    Reference:
    http://www.int-arch-photogramm-remote-sens-spatial-inf-sci.net/XXXIX-B5/169/2012/isprsarchives-XXXIX-B5-169-2012.pdf

    xyz is a matrix contain at least 5 rows, and each row stores x y z of a cylindrical surface
    p is initial values of the parameter;
    p[0] = Xc, x coordinate of the cylinder centre
    P[1] = Yc, y coordinate of the cylinder centre
    P[2] = alpha, rotation angle (radian) about the x-axis
    P[3] = beta, rotation angle (radian) about the y-axis
    P[4] = r, radius of the cylinder

    th, threshold for the convergence of the least squares

    """
    def cylinderFitting(xyz,p):   
        x = xyz[:,0]
        y = xyz[:,1]
        z = xyz[:,2]

        fitfunc = lambda p, x, y, z: (- np.cos(p[3])*(p[0] - x) - z*np.cos(p[2])*np.sin(p[3]) - np.sin(p[2])*np.sin(p[3])*(p[1] - y))**2 + (z*np.sin(p[2]) - np.cos(p[2])*(p[1] - y))**2 #fit function
        errfunc = lambda p, x, y, z: fitfunc(p, x, y, z) - p[4]**2 #error function 

        est_p , success = leastsq(errfunc, p, args=(x, y, z), maxfev=1000)

        return est_p

    p = np.array([-13.79,-8.45,0,0,0.3])
    print("Performing Cylinder Fitting ... ")
    est_p =  cylinderFitting(femoral_neck_points,p)
    print("Fitting Done!")
    print("Estimated Parameters: ")
    print(est_p)

    Xc,Yc,alpha,beta,r = est_p

    # rotate about Y by beta
    R_beta = rot(beta, np.array((0,1,0)))
    # rotate about X by alpha 
    R_alpha = rot(alpha, np.array((1,0,0)))
    ROT = R_beta @ R_alpha

    # find the midpoint of femoral neck
    femoral_neck_midpoint = np.mean(femoral_neck_points, axis=0)

    # z component of ROT is the normal vector of the cylinder
    normal_vec = ROT[:,2]
    # define a cost function that is the sum of all distances to points in femoral_neck_points
    def cost_func(point):
        # point is a 3D point
        # return the sum of distances from point to all points in femoral_neck_points
        return np.sum(np.linalg.norm(femoral_neck_points - point, axis=1))
    # find point that minimizes cost fuction
    from scipy.optimize import minimize
    res = minimize(cost_func, femoral_neck_midpoint, method='nelder-mead', options={'xtol': 1e-8, 'disp': True})
    opt_midpoint = res.x

    # define v2 as Xc,Yc,Zc + normal_vec


    # load femoral head surface poins
    femoral_head_points = read_points_from_markups_json(f"{markup_dir}/FemoralHeadSurface.mrk.json")
    # # multiply x and y components by -1 to convert from LPS to RAS
    # femoral_head_points[:, 0] *= -1
    # femoral_head_points[:, 1] *= -1
    # fit sphere to femoral head points
    from scipy.optimize import leastsq
    def sphereFitting(xyz,p):
        x = xyz[:,0]
        y = xyz[:,1]
        z = xyz[:,2]

        fitfunc = lambda p, x, y, z: (p[0] - x)**2 + (p[1] - y)**2 + (p[2] - z)**2
        errfunc = lambda p, x, y, z: fitfunc(p, x, y, z) - p[3]**2
        
        est_p , success = leastsq(errfunc, p, args=(x, y, z), maxfev=1000)

        return est_p
    # find the midpoint of femoral head
    femoral_head_midpoint = np.mean(femoral_head_points, axis=0)
    # find the average distance from femoral head midpoint to femoral head points
    femoral_head_radius = np.mean(np.linalg.norm(femoral_head_points - femoral_head_midpoint, axis=1))

    p = np.array([femoral_head_midpoint[0],femoral_head_midpoint[1],femoral_head_midpoint[2],femoral_head_radius])
    print("Performing Sphere Fitting ... ")
    est_p =  sphereFitting(femoral_head_points,p)
    print("Fitting Done!")
    print("Estimated Parameters: ")
    print(est_p)
    optx, opty, optz, optr = est_p
    opt_center = np.array([optx, opty, optz])
    print(opt_center)

    v = opt_center - opt_midpoint

    v1 = opt_midpoint + 10*v
    v2 = opt_midpoint - 10*v

    # define v1 as Xc,Yc,Zc - normal_vec
    print(v1)
    print(opt_center)
    print(opt_midpoint)
    print(v2)

    # load approx entry point
    approx_entry_point = read_points_from_markups_json(f"{markup_dir}/ApproxEntry.mrk.json")
    # # multiply x and y components by -1 to convert from LPS to RAS
    # approx_entry_point[:, 0] *= -1
    # approx_entry_point[:, 1] *= -1

    # # closest point on line to point
    # def closest_point_on_line_to_point(line_point1, line_point2, point):
    #     # line_point1 and line_point2 define a line
    #     # point is a point
    #     # return the closest point on the line to point
    #     line_vec = line_point2 - line_point1
    #     line_vec_norm = line_vec / np.linalg.norm(line_vec)
    #     point_vec = point - line_point1
    #     point_vec_norm = point_vec / np.linalg.norm(point_vec)
    #     dot_prod = np.dot(line_vec_norm, point_vec_norm)
    #     closest_point = line_point1 + dot_prod * line_vec
    #     return closest_point

    # find vector projection of point onto line
    # def vector_projection_of_point_onto_line(line_point1, line_point2, point):
    #     # line_point1 and line_point2 define a line
    #     # point is a point
    #     # return the vector projection of point onto line
    #     line_vec = line_point2 - line_point1
    #     line_vec_norm = line_vec / np.linalg.norm(line_vec)
    #     point_vec = point - line_point1
    #     point_vec_norm = point_vec / np.linalg.norm(point_vec)
    #     dot_prod = np.dot(line_vec_norm, point_vec_norm)
    #     vector_projection = dot_prod * line_vec
    #     return vector_projection

    # # subtract vector projection of approx entry point onto line from approx entry point
    # # to get the point on the line that is closest to the approx entry point
    # vector_projection = vector_projection_of_point_onto_line(opt_midpoint, opt_center, approx_entry_point[0])
    # closest_point = approx_entry_point[0] - vector_projection

    # # find actual entry point as the closest point to approx entry point on line between opt_midpoint and opt_center
    # actual_entry_point = closest_point_on_line_to_point(opt_midpoint, opt_center, approx_entry_point[0])
    entry_vec = v / np.linalg.norm(v)

    # # subtract every second entry_vec_point from every first entry_vec_point
    # entry_vec = []
    # entry_points_in = []
    # for i in range(0, len(entry_vec_points_all), 2):
    #     entry_vec.append(entry_vec_points_all[i+1] - entry_vec_points_all[i])
    #     entry_points_in.append(entry_vec_points_all[i])
    entry_points_in = [approx_entry_point]
    # make entry vec of shape (3,1)
    entry_vec = entry_vec.reshape(3,)
    
    entry_vec = [entry_vec]
    
    # generate random points in the sphere
    # https://stackoverflow.com/questions/5408276/sampling-uniformly-distributed-random-points-inside-a-sphere
    def random_points_in_sphere(center, radius, n_points):
        points = []
        while len(points) < n_points:
            x = np.random.uniform(-radius, radius)
            y = np.random.uniform(-radius, radius)
            z = np.random.uniform(-radius, radius)
            if x**2 + y**2 + z**2 < radius**2:
                points.append(np.array([x, y, z]) + center)
        return np.array(points)

    # random points between inner and outer radius
    # https://math.stackexchange.com/questions/87230/picking-random-points-in-the-volume-of-sphere-with-uniform-probability
    def random_points_in_sphere_between_radii(center, inner_radius, outer_radius, n_points):
        points = []
        while len(points) < n_points:
            r = np.random.uniform(inner_radius, outer_radius)
            theta = np.random.uniform(0, 2*np.pi)
            phi = np.random.uniform(0, np.pi)
            x = r * np.sin(phi) * np.cos(theta)
            y = r * np.sin(phi) * np.sin(theta)
            z = r * np.cos(phi)
            points.append(np.array([x, y, z]) + center)
        return np.array(points)

    # generate random points in the sphere
    target_points = random_points_in_sphere_between_radii(opt_center, optr/2 , optr-4, 20)
    # save target points to fcsv
    save_to_fcsv(target_points, f"{markup_dir}/target_points.fcsv")

    

    in_structs = []
    for ie, entry in enumerate(entry_points_in):
        for it, target in enumerate(target_points):
            in_structs.append((ie, it, entry, target, entry_vec[ie],save_dir,opt_midpoint))

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

