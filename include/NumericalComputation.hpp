/****************************************************************************
 * MIT License
 *
 * Copyright (c) 2024 İsmail Çağdaş Yılmaz
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ****************************************************************************/

#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>

/**
 * \brief  p_b/n^n of the craft relative to the NED coordinate system is calculated by numerical integration
 *         for instance by utilizing Euler integration:
 * \param sampling_time
 * \param position current position in the NED coordinate system
 * \param velocity current velocity in body fixed frame
 * \param R_b_n The rotation matrix transforming vectors from the body frame to the NED frame
 */
void position_update(double sampling_time, Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                     const Eigen::Matrix3d& R_b_n)
{
   position +=  sampling_time * R_b_n * velocity; // Check this operation has an aliasing problem or not
}

/**
 * \brief Euler angles calculated through
 * \param sampling_time
 * \param euler_angle in NED frame
 * \param angular_velocity in body fixed frame
 * \param T_nb The angular transformation matrix
 */
void euler_angles_update(double sampling_time, Eigen::Vector3d& euler_angle,
                         const Eigen::Vector3d& angular_velocity, const Eigen::Matrix3d& T_nb)
{
    auto temp = euler_angle + sampling_time * T_nb * angular_velocity;
    euler_angle = temp;
}


/**
 * \brief This is a function implementation "Unit Quaternion Normalization" which is satisfied in the presence of
 * measurement noise and numerical round-off errors. For this purpose, the following discrete-time algorithm can be applied.
 * \param sampling_time
 * \param q quaternion vector in Eigen Library form
 * \param T_q angular value transformation matrix (function of the quaternions vector)
 * \param angular_velocity current velocity in body fixed frame
 */
void unit_quaternion_update(double sampling_time, Eigen::Quaterniond& q,
                            const Eigen::Matrix<double, 4, 3>& T_q, const Eigen::Vector3d& angular_velocity)
{
    Eigen::Vector4d q_vector;

    //Extract the coefficient of the unit_quaternion_update
    q_vector.row(0) << q.coeffs()(3);
    q_vector.bottomRows<3>() = q.vec();

    // For simplicity, Euler integration implies that
    q_vector = q_vector + sampling_time * T_q * angular_velocity;
    //Normalization:
    q_vector.normalize();

    q.w() = q_vector[0];
    q.vec() << q_vector[1], q_vector[2], q_vector[3];
}


