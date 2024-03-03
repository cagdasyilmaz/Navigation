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
#include <cmath>

/**
 * \brief Structure of Radii of curvature
 * R_N: meridian radius of curvature (m)
 * R_E: transverse radius of curvature (m)
 */
struct Radii
{
    double R_N = 0.0f;
    double R_E = 0.0f;
};

/**
 * \brief Radii_of_curvature - Calculates the meridian and transverse radii of curvature
 * \param L geodetic latitude (rad)
 * \retval Radii which includes  R_N and R_E
 */
Radii Radii_of_curvature(double L)
{
    Radii radii;

    // Constants
    double R_0 = 6'378'137;      // WGS84 Equatorial radius in meters
    double e = 0.0818191908425f; // WGS84 eccentricity

    // Calculate meridian radius of curvature
    double temp =  1 - pow(e * sin(L), 2);
    radii.R_N = R_0 * (1 - pow(e, 2.0f)) / pow(temp, 1.5f);

    // Calculate transverse radius of curvature
    radii.R_E = R_0 / sqrt(temp);

    return radii;
}

/**
 * \brief NED_to_ECEF - Converts curvilinear to Cartesian position
 * resolving axes from NED to ECEF and attitude from NED- to ECEF-referenced
 * \param lat_long latitude and longitude in radian forms
 * \param h_b height (m)
 * \retval r_eb_e  = [r_eb_e, y_eb_e, z_eb_e]^T
 *                 Cartesian position of body frame w.r.t. ECEF frame, resolved along ECEF-frame axes (m)
 */
Eigen::Vector3d NED_to_ECEF_Position(const Angle_Lat_Long& lat_long, double h_b)
{
    // Calculate transverse radius of curvature
    auto radii = Radii_of_curvature(lat_long.angles_[0]);

    // Constants
    double e = 0.0818191908425f; // WGS84 eccentricity

    double cos_lat = cos(lat_long.angles_[0]);
    double sin_lat = sin(lat_long.angles_[0]);
    double cos_long = cos(lat_long.angles_[1]);
    double sin_long = sin(lat_long.angles_[1]);

    return Eigen::Vector3d{(radii.R_E + h_b) * cos_lat * cos_long,
                           (radii.R_E + h_b) * cos_lat * sin_long,
                           ((1 - pow(e, 2.0f)) * radii.R_E + h_b) * sin_lat};
}