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
#include <cmath>

enum class AngleType {radian, degree, gradian};

/**
 * \brief This class is a factory implementation of the Euler angles, when the angles defined in the order of
 * roll, pitch, yaw, they will be transformed to radian forms. Default constructor will not be called directly
 */
class Angle
{
public:
    Eigen::Vector3d angles_;

    /**
    * \param angle Euler angles in radian from
    * \returns Angle in radian forms
    */
    static Angle Radian(const Eigen::Vector3d& angles = Eigen::Vector3d{0, 0, 0})
    {
        return angles;
    }

    /**
    * \param angle Euler angles in degree from
    * \returns Angle in radian forms
    */
    static Angle Degree(const Eigen::Vector3d& angles = Eigen::Vector3d{0, 0, 0})
    {
        return Eigen::Vector3d {static_cast<double>(M_PI * angles[0] / 180.f),
                                static_cast<double>(M_PI * angles[1] / 180.f),
                                static_cast<double>(M_PI * angles[2] / 180.f) };
    }

    /**
    * \param angle Euler angles in gradian from
    * \returns Angle in radian forms
    */
    static Angle Gradian(Eigen::Vector3d angles = Eigen::Vector3d{0, 0, 0})
    {
        return Eigen::Vector3d {static_cast<double>(M_PI * angles[0] / 200.f),
                                static_cast<double>(M_PI * angles[1] / 200.f),
                                static_cast<double>(M_PI * angles[2] / 200.f) };
    }

    /**
    * \returns angle Euler angles in gradian form
    */
    [[nodiscard]] Eigen::Vector3d radian() const
    {
        return Eigen::Vector3d{angles_[0], angles_[1], angles_[2]};
    }

    /**
    * \returns angle Euler angles in degree form
    */
    [[nodiscard]] Eigen::Vector3d degree() const
    {
        return Eigen::Vector3d{static_cast<double>(angles_[0] * 180.f / M_PI),
                               static_cast<double>(angles_[1] * 180.f / M_PI),
                               static_cast<double>(angles_[2] * 180.f / M_PI)};
    }

    /**
    * \returns angle Euler angles in degree form
    */
    [[nodiscard]] Eigen::Vector3d gradian() const
    {
        return Eigen::Vector3d{static_cast<double>(angles_[0] * 200.f / M_PI),
                               static_cast<double>(angles_[1] * 200.f / M_PI),
                               static_cast<double>(angles_[2] * 200.f / M_PI)};
    }

private:
    [[maybe_unused]] Angle(Eigen::Vector3d angles) : angles_{std::move(angles)} {}
};

