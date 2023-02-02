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

// General Equations
// P -> Impulse (or Sequence Impulse)
// J -> Constraint Jacobian
// M -> Mass matrix (Accumulated Mass Matrix)
// v -> Constraint velocity
// dt -> Time step
// bias -> Bias factor
// K -> Constraint Mass Matrix
// s(t) -> Combined state vector of two bodies between which there is a constraint i.e. [x_1 q_1 x_2 q_t]'
// C(s) -> Constraint, which is a function of the state vector.

// For 3D. E is identity Matrix (3x3 for 3D), m1 and m2 masses, I1 and I2 Inertias
// M = diag(m1 * E, I1, m2 * E, I2)

// P = transpose(J) * lambda_prime
// lambda = -inverse(K) * ( J * v_prime + bias)
// lambda_prime = lambda / dt
// K = J * inverse(M) * transpose(J)
// v_prime = v + inverse(M) * F_external * dt (For constraint, no need to put external force F_external)
// penetration_bias = -b * C / dt
// contact_velocity_bias = a * dot(Vc,n)
// b -> [0, 1]
// a -> [0, 1]
// bias = penetration_bias + contact_velocity_bias

// For non penetrating contact
// n is the contact normal and p_1 and p_2 are contact points on body 1 and 2 in world coordinates. r_1 and r_2 are
// the vectors from the COM of body 1 and 2 to their respective contact points.
// e = p2 - p1
// C = dot(e, n) = dot((x_2 + r_2 - x_1 - r_1), n)

// dC/dt = (-n', -cross(r_1, n)', n_1', cross(r_2, n)') * (v_1, w_1, v_2, w_2)'
// Here a' indicates transpose of a vector a.

// The first part of dC/dt is the Jacobian for non penetration contact
// J = (-n', -cross(r_1, n)', n_1', cross(r_2, n)')

// From above
// K = (-n', -cross(r_1, n)', n_1', cross(r_2, n)') * inverse(M) * (-n', -cross(r_1, n)', n_1', cross(r_2, n)')'

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include "gnuplot-iostream.h"
#include <boost/tuple/tuple.hpp>

bool compute_contact_sphere_sphere(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2, double r1, double r2,
                                   Eigen::Vector3d &rb1, Eigen::Vector3d &rb2, Eigen::Vector3d &normal)
{
    bool in_contact;
    Eigen::Vector3d e = x2 - x1;
    if (e.norm() <= (r1 + r2))
    {
        in_contact = true;
        normal = e / e.norm();
        rb1 = normal * r1;
        rb2 = -normal * r2;
    }
    else
    {
        in_contact = false;
        normal = Eigen::Vector3d::Zero();
        rb1 = Eigen::Vector3d::Zero();
        rb2 = Eigen::Vector3d::Zero();
    }
    return in_contact;
}

bool compute_contact_sphere_plane(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2, double r, const Eigen::Vector3d &plane_normal,
                                  Eigen::Vector3d &rb1, Eigen::Vector3d &rb2, Eigen::Vector3d &normal)
{
    bool in_contact;
    Eigen::Vector3d e = x2 - x1;
    double proj = -e.dot(plane_normal);
    if (proj <= r)
    {
        in_contact = true;
        normal = -plane_normal;
        rb1 = normal * r;
        rb2 = x1 + normal * proj;
    }
    else
    {
        in_contact = false;
        normal = Eigen::Vector3d::Zero();
        rb1 = Eigen::Vector3d::Zero();
        rb2 = Eigen::Vector3d::Zero();
    }
    return in_contact;
}

void compute_constraint_and_jacobian(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2, const Eigen::Vector3d &rb1, const Eigen::Vector3d &rb2, const Eigen::Vector3d &n,
                                     double &C, Eigen::Matrix<double, 1, 12> &J)
{
    Eigen::Vector3d e = x2 + rb2 - x1 - rb1;
    C = e.dot(n);
    J << -n.transpose(), -rb1.cross(n).transpose(), n.transpose(), rb2.cross(n).transpose();
}

double compute_bias(double C, const Eigen::Matrix<double, 12, 1> &V, const Eigen::Vector3d &n, double dt, double b, double a, double b_slop = 0.1, double a_slop = 0.1)
{
    double pen_bias = (-b / dt) * std::max(std::abs(C) - b_slop, 0.);
    // pen_bias = (-b / dt) * C
    double Vc = (V.segment<3>(6) - V.segment<3>(0)).dot(n);
    double restitution_bias = a * Vc;
    return pen_bias + restitution_bias;
}

void compute_impulse(const Eigen::Matrix<double, 1, 12> &J, const Eigen::Matrix<double, 12, 1> &V, const Eigen::Matrix<double, 12, 1> &F_ext, double bias, double dt,
                     Eigen::Matrix<double, 12, 1> &P, Eigen::Matrix<double, 12, 12> &M_inv)
{
    Eigen::Matrix<double, 1, 1> K = J * M_inv * J.transpose();
    double inv_K = 1. / K(0, 0);
    Eigen::Matrix<double, 12, 1> Vi = V + M_inv * F_ext * dt;
    double lam = -inv_K * (J * Vi + bias);
    P = J.transpose() * lam * dt;
}

bool compute_impulse_two_sphere_collision(Eigen::Matrix<double, 12, 1> &P, const Eigen::Vector3d &x1, const Eigen::Vector3d &x2, const double r1, const double r2, const double m1, const double m2, const double dt,
                                          const Eigen::Matrix<double, 12, 1> V, const Eigen::Matrix<double, 12, 1> F_ext, const double b, const double a)
{
    bool in_contact;
    Eigen::Vector3d rb1, rb2, normal;
    in_contact = compute_contact_sphere_sphere(x1, x2, r1, r2, rb1, rb2, normal);
    if (in_contact)
    {
        double C;
        Eigen::Matrix<double, 1, 12> J;
        compute_constraint_and_jacobian(x1, x2, rb1, rb2, normal, C, J);
        double bias = compute_bias(C, V, normal, dt, b, a);
        double inv_m1 = m1 > 0. ? 1. / m1 : 0.;
        double inv_m2 = m2 > 0. ? 1. / m2 : 0.;
        double ix1 = 1.0, iy1 = ix1, iz1 = ix1;
        double ix2 = 10.0, iy2 = ix2, iz2 = ix2;
        Eigen::Matrix<double, 12, 12> M_inv = Eigen::Matrix<double, 12, 12>::Identity();
        M_inv.block(0, 0, 3, 3) = inv_m1 * Eigen::Matrix<double, 3, 3>::Identity();
        M_inv.block(3, 3, 3, 3) = Eigen::DiagonalMatrix<double, 3>(Eigen::Vector3d(1. / ix1, 1. / iy1, 1. / iz1));
        M_inv.block(6, 6, 3, 3) = inv_m2 * Eigen::Matrix<double, 3, 3>::Identity();
        M_inv.block(9, 9, 3, 3) = Eigen::DiagonalMatrix<double, 3>(Eigen::Vector3d(1. / ix2, 1. / iy2, 1. / iz2));
        compute_impulse(J, V, F_ext, bias, dt, P, M_inv);

        // std::cout << "x1: " << x1 << std::endl;
        // std::cout << "x2: " << x2 << std::endl;
        // std::cout << "M_inv: " << M_inv << std::endl;
        // std::cout << "J: " << J << std::endl;
        // std::cout << "V: " << V << std::endl;
        // std::cout << "F_ext: " << F_ext << std::endl;
        // std::cout << "b: " << b << std::endl;
        // std::cout << "a: " << a << std::endl;
        // std::cout << "r1: " << r1 << std::endl;
        // std::cout << "r2: " << r2 << std::endl;
        // std::cout << "n: " << normal << std::endl;
        // std::cout << "rb1: " << rb1 << std::endl;
        // std::cout << "rb2: " << rb2 << std::endl;
        // std::cout << "C: " << C << std::endl;
        // std::cout << "bias: " << bias << std::endl;
        
    }
    return in_contact;
}


// main function
int main()
{
    double m1 = 100.;
    double m2 = 0.;
    double inv_m1 = m1 > 0. ? 1. / m1 : 0.;
    double inv_m2 = m2 > 0. ? 1. / m2 : 0.;
    double ix1 = 1.0, iy1 = ix1, iz1 = ix1;
    double ix2 = 10.0, iy2 = ix2, iz2 = ix2;
    Eigen::Matrix<double, 12, 12> M_inv = Eigen::Matrix<double, 12, 12>::Identity();
    M_inv.block(0, 0, 3, 3) = inv_m1 * Eigen::Matrix<double, 3, 3>::Identity();
    M_inv.block(3, 3, 3, 3) = Eigen::DiagonalMatrix<double, 3>(Eigen::Vector3d(1. / ix1, 1. / iy1, 1. / iz1));
    M_inv.block(6, 6, 3, 3) = inv_m2 * Eigen::Matrix<double, 3, 3>::Identity();
    M_inv.block(9, 9, 3, 3) = Eigen::DiagonalMatrix<double, 3>(Eigen::Vector3d(1. / ix2, 1. / iy2, 1. / iz2));

    double r1 = 1.0;
    double r2 = 3.0;
    Eigen::Vector3d plane_normal = Eigen::Vector3d(0, 0, 1);
    plane_normal = plane_normal / plane_normal.norm();

    double b = 0.4;
    double a = 1.0;
    double t_start = 0.;
    double t_end = 50.;
    double dt = 1. / 100.;
    double t = t_start;
    Eigen::Matrix<double, 12, 1> V = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, 12, 1> F_c = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, 12, 1> F_ext = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Vector3d x1 = Eigen::Vector3d(0, 0, 10);
    Eigen::Vector3d x2 = Eigen::Vector3d(0, 0, 0);
    F_ext[2] = -9.8;
    F_ext[8] = -9.8;

    int itr = 0;

    std::vector<double> time_series;
    std::vector<double> x1_series;
    std::vector<double> x2_series;

    while (t <= t_end)
    {
        Eigen::Matrix<double, 3, 1> dx1 = V.block(0, 0, 3, 1) * dt;
        Eigen::Matrix<double, 3, 1> dx2 = V.block(6, 0, 3, 1) * dt;
        x1 = x1 + dx1;
        x2 = x2 + dx2;
        V = V + M_inv * (F_ext + F_c) * dt;

        Eigen::Vector3d rb1 = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d rb2 = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d n = Eigen::Vector3d(0, 0, 0);
        // bool contact = compute_contact_sphere_plane(x1, x2, r1, plane_normal, rb1, rb2, n);
        bool contact = compute_contact_sphere_sphere(x1, x2, r1, r2, rb1, rb2, n);

        // std::cout << "contact:" << contact << std::endl;
        Eigen::Matrix<double, 12, 1> P = Eigen::Matrix<double, 12, 1>::Zero();
        if (contact)
        {
            double C;
            Eigen::Matrix<double, 1, 12> J;
            compute_constraint_and_jacobian(x1, x2, rb1, rb2, n, C, J);
            double bias = compute_bias(C, V, n, dt, b, a);
            compute_impulse(J, V, F_ext, bias, dt, P, M_inv);
        }

        time_series.push_back(t);
        x1_series.push_back(x1[2]);
        x2_series.push_back(x2[2]);

        F_c = P / dt;
        t = t + dt;
        itr = itr + 1;
        std::cout << x1.transpose() << " " << x2.transpose() << std::endl;
    }

    // plot times_series vs x1_series and time_series vs x2_series to the screen using gnuplot
    Gnuplot gp;
    gp << "set terminal qt" << std::endl;
    gp << "set title \"x1 and x2 vs time\"" << std::endl;
    gp << "plot '-' with lines title 'x1', '-' with lines title 'x2'" << std::endl;
    gp.send1d(boost::make_tuple(time_series, x1_series));
    gp.send1d(boost::make_tuple(time_series, x2_series));

    return 0;
}

// // function that takes m1, m2, r1, r2, x1, x2, and returns the contact force
// Eigen::Matrix<double, 12, 1> compute_contact_force(double m1, double m2, double r1, double r2, Eigen::Vector3d x1, Eigen::Vector3d x2, double dt)
// {
//     double inv_m1 = m1>0. ? 1. / m1 : 0.;
//     double inv_m2 = m2>0. ? 1. / m2 : 0.;
//     double ix1 = 2.0/5.0*m1*r1*r1, iy1 = ix1, iz1 = ix1; // assume sphere
//     double ix2 = 2.0/5.0*m2*r2*r2, iy2 = ix2, iz2 = ix2; // assume sphere
//     Eigen::Matrix<double, 12, 12> M_inv = Eigen::Matrix<double, 12, 12>::Identity();
//     M_inv.block(0, 0, 3, 3) = inv_m1 * Eigen::Matrix<double, 3, 3>::Identity();
//     M_inv.block(3, 3, 3, 3) = Eigen::DiagonalMatrix<double, 3>(Eigen::Vector3d(1. / ix1, 1. / iy1, 1. / iz1));
//     M_inv.block(6, 6, 3, 3) = inv_m2 * Eigen::Matrix<double, 3, 3>::Identity();
//     M_inv.block(9, 9, 3, 3) = Eigen::DiagonalMatrix<double, 3>(Eigen::Vector3d(1. / ix2, 1. / iy2, 1. / iz2));

//     double b = 0.4;
//     double a = 1.0;
//     Eigen::Matrix<double, 12, 1> V = Eigen::Matrix<double, 12, 1>::Zero();
//     Eigen::Matrix<double, 12, 1> F_c = Eigen::Matrix<double, 12, 1>::Zero();

//     Eigen::Matrix<double, 3, 1> dx1 = V.block(0, 0, 3, 1) * dt;
//     Eigen::Matrix<double, 3, 1> dx2 = V.block(6, 0, 3, 1) * dt;
//     x1 = x1 + dx1;
//     x2 = x2 + dx2;
//     V = V + M_inv * (F_ext + F_c) * dt;

//     Eigen::Vector3d rb1 = Eigen::Vector3d(0, 0, 0);
//     Eigen::Vector3d rb2 = Eigen::Vector3d(0, 0, 0);
//     Eigen::Vector3d n = Eigen::Vector3d(0, 0, 0);

//     bool contact = compute_contact_sphere_sphere(x1, x2, r1, r2, rb1, rb2, n);

//     // bool contact = compute_contact_sphere_plane(x1, x2, r1, plane_normal, rb1, rb2, n);
//     // std::cout << "contact:" << contact << std::endl;
//     Eigen::Matrix<double, 12, 1> P = Eigen::Matrix<double, 12, 1>::Zero();
//     if (contact)
//     {
//         double C;
//         Eigen::Matrix<double, 1, 12> J;
//         compute_constraint_and_jacobian(x1, x2, rb1, rb2, n, C, J);
//         double bias = compute_bias(C, V, n, b, a, dt);
//         compute_impulse(J, V, F_ext, bias, dt, P, M_inv);
//     }

//     time_series.push_back(t);
//     x1_series.push_back(x1[2]);
//     x2_series.push_back(x2[2]);

//     F_c = P / dt;
// }

// function that takes m1, m2, r1, r2, x1, x2, V and returns the contact force
Eigen::Matrix<double, 12, 1> compute_contact_force(double m1, double m2, double r1, double r2, Eigen::Vector3d x1, Eigen::Vector3d x2, Eigen::Matrix<double, 12, 1> V, double dt)
{
    double inv_m1 = m1 > 0. ? 1. / m1 : 0.;
    double inv_m2 = m2 > 0. ? 1. / m2 : 0.;
    double ix1 = 2.0 / 5.0 * m1 * r1 * r1, iy1 = ix1, iz1 = ix1; // assume sphere
    double ix2 = 2.0 / 5.0 * m2 * r2 * r2, iy2 = ix2, iz2 = ix2; // assume sphere
    Eigen::Matrix<double, 12, 12> M_inv = Eigen::Matrix<double, 12, 12>::Identity();
    M_inv.block(0, 0, 3, 3) = inv_m1 * Eigen::Matrix<double, 3, 3>::Identity();
    M_inv.block(3, 3, 3, 3) = Eigen::DiagonalMatrix<double, 3>(Eigen::Vector3d(1. / ix1, 1. / iy1, 1. / iz1));
    M_inv.block(6, 6, 3, 3) = inv_m2 * Eigen::Matrix<double, 3, 3>::Identity();
    M_inv.block(9, 9, 3, 3) = Eigen::DiagonalMatrix<double, 3>(Eigen::Vector3d(1. / ix2, 1. / iy2, 1. / iz2));

    double b = 0.4;
    double a = 1.0;
    Eigen::Matrix<double, 12, 1> F_c = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, 12, 1> F_ext = Eigen::Matrix<double, 12, 1>::Zero();
    // Eigen::Matrix<double, 3, 1> dx1 = V.block(0, 0, 3, 1) * dt;
    // Eigen::Matrix<double, 3, 1> dx2 = V.block(6, 0, 3, 1) * dt;
    // x1 = x1 + dx1;
    // x2 = x2 + dx2;
    // V = V + M_inv * (F_ext + F_c) * dt;

    Eigen::Vector3d rb1 = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d rb2 = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d n = Eigen::Vector3d(0, 0, 0);

    bool contact = compute_contact_sphere_sphere(x1, x2, r1, r2, rb1, rb2, n);
    Eigen::Matrix<double, 12, 1> P = Eigen::Matrix<double, 12, 1>::Zero();
    if (contact)
    {
        double C;
        Eigen::Matrix<double, 1, 12> J;
        compute_constraint_and_jacobian(x1, x2, rb1, rb2, n, C, J);
        double bias = compute_bias(C, V, n, dt, b, a);
        compute_impulse(J, V, F_ext, bias, dt, P, M_inv);
    }
    F_c = P / dt;
    return F_c;
}

// // function that takes in a toolcursor and returns the contact force
// void apply_forces_to_tool_cursor_sequenital_impulse(cToolCursor& tool_cursor, double dt)
// {
//     // m1 is mass of the tool cursor's object

//     // m2 is mass of the volume (rigid) therefore m2 = 0
//     double m2 = 0.0;

//     // r1 is the radius of the tool cursor's object
//     double r1 = tool_cursor.get_radius();

//     // r2 is the radius of the voxel

//     // x1 is the position of the tool cursor's

//     //m1, m2, r1, r2, x1, x2, V

// }