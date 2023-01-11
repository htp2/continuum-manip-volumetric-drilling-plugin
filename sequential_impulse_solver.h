//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2022
    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
    \author    <henry.phalen@jhu.edu>
    \author    Henry Phalen

    \author    <amunawa2@jh.edu>
    \author    Adnan Munawar
*/
//==============================================================================

#ifndef SEQUENTIAL_IMPULSE_SOLVER_H
#define SEQUENTIAL_IMPULSE_SOLVER_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <math.h>
#include <vector>
#include <string>

bool compute_contact_sphere_sphere(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2, double r1, double r2,
                                   Eigen::Vector3d &rb1, Eigen::Vector3d &rb2, Eigen::Vector3d &normal);

bool compute_contact_sphere_plane(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2, double r, const Eigen::Vector3d &plane_normal,
                                  Eigen::Vector3d &rb1, Eigen::Vector3d &rb2, Eigen::Vector3d &normal);

void compute_constraint_and_jacobian(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2, const Eigen::Vector3d &rb1, const Eigen::Vector3d &rb2, const Eigen::Vector3d &n,
                                     double &C, Eigen::Matrix<double, 1, 12> &J);

double compute_bias(double C, const Eigen::Matrix<double, 12, 1> &V, const Eigen::Vector3d &n, double dt, double b = 0.4, double a = 1.0, double b_slop = 0.1, double a_slop = 0.1);

void compute_impulse(const Eigen::Matrix<double, 1, 12> &J, const Eigen::Matrix<double, 12, 1> &V, const Eigen::Matrix<double, 12, 1> &F_ext, double bias, double dt,
                     Eigen::Matrix<double, 12, 1> &P, Eigen::Matrix<double, 12, 12> &M_inv);

bool compute_impulse_two_sphere_collision(Eigen::Matrix<double, 12, 1> &P, const Eigen::Vector3d &x1, const Eigen::Vector3d &x2, const double r1, const double r2, const double m1, const double m2, const double dt,
                                          const Eigen::Matrix<double, 12, 1> V = Eigen::Matrix<double, 12, 1>::Zero(), const Eigen::Matrix<double, 12, 1> F_ext = Eigen::Matrix<double, 12, 1>::Zero(), const double b=0.4, const double a=1.0);

#endif // SEQUENTIAL_IMPULSE_SOLVER_H
