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

#include "Angle.hpp"
#include <eigen3/Eigen/Dense>
#include <cmath>

/**
 * \brief Computes the rotation matrix from the body frame to the local navigation frame NED (North-East-Down).
 * \param euler Euler angles representing the orientation of the body frame in radians, in the order of roll, pitch, yaw.
 * \returns The rotation matrix transforming vectors from the body frame to the NED frame.
 */
Eigen::Matrix3d body_to_ned_rotation(const Angle& euler)
{
    Eigen::Matrix3d R_b_n;

    R_b_n = Eigen::AngleAxisd(euler.angles_[2], Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(euler.angles_[1], Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(euler.angles_[0], Eigen::Vector3d::UnitX());

    return R_b_n;
}

/**
 * \brief Computes the rotation matrix from the local navigation frame NED (North-East-Down) to the body frame .
 * \param euler Euler angles representing the orientation of the body frame in radians, in the order of roll, pitch, yaw.
 * \returns The rotation matrix transforming vectors from the NED frame to the body frame.
 */
Eigen::Matrix3d ned_to_body_rotation(const Angle& euler)
{
    return body_to_ned_rotation(euler).inverse();
}

/**
 * \brief Converts Euler angles to a quaternion representing the orientation.
 * \param euler Euler angles representing the orientation in radians, in the order of roll, pitch, yaw.
 * \returns Quaternion representing the orientation.
 */
Eigen::Quaterniond euler_to_quaternion(const Angle& euler)
{
    Eigen::AngleAxisd rollAngle(euler.angles_[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(euler.angles_[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(euler.angles_[2], Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    return q;
}

/**
 * \brief Converts a quaternion to a rotation matrix.
 * \note This function is added to be conform with the Transform class' naming scheme.
 * \param q Quaternion representing the orientation.
 * \returns Rotation matrix representing the orientation.
 */
Eigen::Matrix3d quaternion_to_rotation(const Eigen::Quaterniond& q)
{
    return q.toRotationMatrix();
}

/**
 * \brief Converts a quaternion to a euler angles.
 * \param q Quaternion representing the orientation.
 * \returns Euler angles in the order of roll, pitch, yaw.
 */
Eigen::Vector3d quaternion_to_euler_angles(const Eigen::Quaterniond& q)
{
    Eigen::Matrix3d R_n_b = q.toRotationMatrix();

    return Eigen::Vector3d{atan2(R_n_b(2,1), R_n_b(2, 2)),
                           -asin(R_n_b(2, 0)),
                           atan2(R_n_b(1,0), R_n_b(0, 0))};
}

/**
 * \brief Angular velocity transformation by using Euler angles
 * \param euler Euler angles representing the orientation of the body frame in radians, in the order of roll, pitch, yaw.
 * \returns T_nb The angular transformation matrix
 */
Eigen::Matrix3d body_to_ned_transformation_from_euler(const Angle& euler)
{
    assert((M_PI_2 - std::abs(euler.angles_[1])) > 0.0001f);

    // Create a 3x3 matrix initialized to identity
    Eigen::Matrix3d T_theta_nb = Eigen::Matrix3d::Identity();

    T_theta_nb(0, 1) = sin(euler.angles_[0]) * tan(euler.angles_[1]);
    T_theta_nb(0, 2) = cos(euler.angles_[0]) * tan(euler.angles_[1]);
    T_theta_nb(1, 1) = cos(euler.angles_[0]);
    T_theta_nb(1, 2) = -sin(euler.angles_[0]);
    T_theta_nb(2, 1) = sin(euler.angles_[0]) / cos(euler.angles_[1]);
    T_theta_nb(2, 2) = cos(euler.angles_[0]) / cos(euler.angles_[1]);

    return T_theta_nb;
}

/**
 * \brief Angular velocity transformation by using quaternions
 * \param q Quaternion representing the orientation.
 * \returns T_q The angular transformation matrix
 */
Eigen::Matrix<double, 4,3> body_to_ned_transformation_from_quaternion(const Eigen::Quaterniond& q)
{
    Eigen::Matrix<double, 4, 3> T_q;

    Eigen::Vector3d epsilon = q.vec();
    T_q.row(0) = -epsilon.transpose();

    Eigen::Matrix3d S_epsilon;
    S_epsilon << 0, -epsilon[2], epsilon[1],
            epsilon[2], 0, -epsilon[0],
            -epsilon[1], epsilon[0], 0;
    // q = [epsilon(0) epsilon(1) epsilon(2) eta]^T
    // eta = q.w()
    // Assign the skew-symmetric matrix to the bottom part of T_q with addition the identity matrix with eta diagonal values
    T_q.bottomRows<3>() = S_epsilon + q.w() * Eigen::Matrix3d::Identity();

    return 0.5f * T_q;
}
