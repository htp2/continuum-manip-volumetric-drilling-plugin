#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2022

#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.

#     \author    <amunawa2@jh.edu>
#     \author    Adnan Munawar
# */
# //==============================================================================

# General Equations
# P -> Impulse (or Sequence Impulse)
# J -> Constraint Jacobian
# M -> Mass matrix (Accumulated Mass Matrix)
# v -> Constraint velocity
# dt -> Time step
# bias -> Bias factor
# K -> Constraint Mass Matrix
# s(t) -> Combined state vector of two bodies between which there is a constraint i.e. [x_1 q_1 x_2 q_t]'
# C(s) -> Constraint, which is a function of the state vector.

# For 3D. E is identity Matrix (3x3 for 3D), m1 and m2 masses, I1 and I2 Inertias
# M = diag(m1 * E, I1, m2 * E, I2)


# P = transpose(J) * lambda_prime
# lambda = -inverse(K) * ( J * v_prime + bias)
# lambda_prime = lambda / dt
# K = J * inverse(M) * transpose(J)
# v_prime = v + inverse(M) * F_external * dt (For constraint, no need to put external force F_external)
# penetration_bias = -b * C / dt
# contact_velocity_bias = a * dot(Vc,n)
# b -> [0, 1]
# a -> [0, 1]
# bias = penetration_bias + contact_velocity_bias

# For non penetrating contact
# n is the contact normal and p_1 and p_2 are contact points on body 1 and 2 in world coordinates. r_1 and r_2 are
# the vectors from the COM of body 1 and 2 to their respective contact points.
# e = p2 - p1
# C = dot(e, n) = dot((x_2 + r_2 - x_1 - r_1), n)

# dC/dt = (-n', -cross(r_1, n)', n_1', cross(r_2, n)') * (v_1, w_1, v_2, w_2)'
# Here a' indicates transpose of a vector a.

# The first part of dC/dt is the Jacobian for non penetration contact
# J = (-n', -cross(r_1, n)', n_1', cross(r_2, n)')

# From above
# K = (-n', -cross(r_1, n)', n_1', cross(r_2, n)') * inverse(M) * (-n', -cross(r_1, n)', n_1', cross(r_2, n)')'


import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt

m1 = 100.
m2 = 0.
inv_m1 = (1. / m1) if m1 > 0. else 0.
inv_m2 = (1. / m2) if m2 > 0. else 0.

# Assume nice and diagonal inertia
ix1, iy1, iz1 = 1, 1, 1
ix2, iy2, iz2 = 10, 10, 10

M_inv = np.eye(12)
M_inv[0:3, 0:3] = inv_m1 * np.eye(3)
M_inv[3:6, 3:6] = np.diag([1./ix1, 1./iy1, 1./iz1])
M_inv[6:9, 6:9] = inv_m2 * np.eye(3)
M_inv[9:12, 9:12] = np.diag([1./ix2, 1./iy2, 1./iz2])

# print(M)
# print('----')
# print(M_inv)

r1 = 1.0
r2 = 3.0
plane_normal = np.array([[0, 0, 1]]).T
plane_normal = plane_normal / la.norm(plane_normal)


def compute_contact_sphere_sphere(x1, x2, r1, r2):
    '''
    :param x1: world pos of sphere 1
    :param x2: world pos of sphere 2
    :param r1: radius of sphere 1
    :param r2: radius of sphere 2
    :return:
    '''
    e = x2 - x1
    if la.norm(e) <= (r1 + r2):
        in_contact = True
        normal = e / la.norm(e)
        rb1 = normal * r1
        rb2 = -normal * r2
    else:
        in_contact = False
        normal = np.array([[0, 0, 0]]).T
        rb1 = np.array([[0, 0, 0]]).T
        rb2 = np.array([[0, 0, 0]]).T

    return in_contact, rb1, rb2, normal


def compute_contact_sphere_plane(x1, x2, r, plane_normal):
    '''
    :param x1: world pos os sphere
    :param x2: world pos of plane
    :param r: radius of sphere 1
    :param plane_normal: world normal of plane
    :return:
    '''
    e = x2 - x1
    proj = np.dot(-e.T, plane_normal)
    if proj <= r:
        in_contact = True
        normal = -plane_normal
        rb1 = normal * r
        rb2 = x1 + normal * proj
    else:
        in_contact = False
        normal = np.array([[0, 0, 0]]).T
        rb1 = np.array([[0, 0, 0]]).T
        rb2 = np.array([[0, 0, 0]]).T

    return in_contact, rb1, rb2, normal


def compute_constraint_and_jacobian(x1, x2, rb1, rb2, n):
    '''
    :param x1: Body 1 pos 3x1
    :param x2: Body 2 pos 3x1
    :param rb1: Vector from body 1 COM to contact point 3x1
    :param rb2: Vector from body 2 COM to contact point 3x1
    :param n: Contact Normal Vector from body 1 to body 2
    :return:
    '''
    C = np.dot((x1 + rb1 - x2 - rb2).T, n)
    J = np.concatenate((-n.T, -np.cross(rb1.T, n.T), n.T, np.cross(rb2.T, n.T)))
    J = J.reshape([1, 12])
    return C, J


def compute_bias(C, V, n, b, a, dt, b_slop=0.1, a_slop=0.1):
    '''
    :param C: Constraint sol 1x1
    :param V: Velocity vector 12x1
    :param n: contact normal 3x1
    :param b: pen bias coeff 1x1
    :param a: contact vel coeff 1x1
    :param dt: time step 1x1
    :param b_slop: pen slop coeff 1x1
    :param a_slop: contact vel slop coeff 1x1
    :return:
    '''
    pen_bias = (-b / dt) * max(abs(C) - b_slop, 0.)
    # pen_bias = (-b / dt) * C
    Vc = np.dot((V[6:9] - V[0:3]).T, n)
    restitution_bias = a * Vc

    return pen_bias + restitution_bias


def compute_impulse(J, V, F_ext, bias, dt):
    '''
    :param J: Constraint Jacobian 1x12
    :param V: Velocity vector 12x1
    :param F_ext: External force 12x1
    :param bias: bias coeffs 1x1
    :param dt: time step 1x1
    :return:
    '''
    K = np.matmul(np.matmul(J, M_inv), J.T)
    # print(K)
    inv_K = 1./K

    Vi = V + np.matmul(M_inv, F_ext) * dt
    lam = -inv_K * (np.matmul(J, Vi) + bias)
    P = J.T * lam * dt
    return P


x1 = np.array([[0, 0, 10]]).T
x2 = np.array([[0, 0, 0]]).T

#bias
b = 0.4
# Restitution
a = 1.0
t_start = 0.
t_end = 50.
dt = 1./100.
t = t_start
V = np.zeros([12, 1])
F_c = np.zeros([12, 1])
F_ext = np.zeros([12, 1])
F_ext[2, 0] = -9.8 # Gravitational Force
F_ext[8, 0] = -9.8 # Gravitational Force

itr = 0

time_series = []
x1_series = []
x2_series = []
while t <= t_end:
    # V = V + np.matmul(M_inv, F_ext + F_c) * dt
    dx1 = V[0:3] * dt
    dx2 = V[6:9] * dt
    x1 = x1 + dx1
    x2 = x2 + dx2
    V = V + np.matmul(M_inv, F_ext + F_c) * dt

    # print(itr, t, x1[2], V[2], F_c[2])

    # contact, rb1, rb2, n = compute_contact_sphere_sphere(x1, x2, r1, r2)
    contact, rb1, rb2, n = compute_contact_sphere_plane(x1, x2, r1, plane_normal)
    if contact:
        C, J = compute_constraint_and_jacobian(x1, x2, rb1, rb2, n)
        bias = compute_bias(C, V, n, b, a, dt)
        P = compute_impulse(J, V, F_ext, bias, dt)
        # print('\r', P[2], n)
    else:
        P = np.zeros([12, 1])

    time_series.append(t)
    x1_series.append(x1[2])
    x2_series.append(x2[2])

    F_c = P / dt
    t = t+dt
    itr = itr + 1


plt.plot(time_series, x1_series, 'r.', time_series, x2_series, 'g.')
# plt.axis([t_start, t_end, -50, 50])
plt.grid()
# plt.show()

# file temp.txt contains the following data rows are like "x1:       0        0 0.897575	x2:0 0 0"
# read the data into a numpy array for x1 and x2 then plot as new figure
# open new figure
plt.figure(2)
x1 = np.loadtxt('/home/henry/bigss/catkin_ws/src/continuum-manip-volumetric-drilling-plugin/temp.txt', usecols=(0,1,2))
x2 = np.loadtxt('/home/henry/bigss/catkin_ws/src/continuum-manip-volumetric-drilling-plugin/temp.txt', usecols=(3,4,5))
plt.plot(x1[:,2], 'r.', x2[:,2], 'g.')
plt.show()




