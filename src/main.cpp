#include "Angle.hpp"
#include "Transformation.hpp"
#include "NumericalComputation.hpp"

#include <iostream>
#include <chrono>

int main()
{
    auto begin = std::chrono::steady_clock::now();

    auto euler = Angle::Degree(Eigen::Vector3d{0, 0, 5});

    Eigen::Vector3d linear_velocity{2.0f, 0.0f, 0.0f};
    Eigen::Vector3d angular_velocity{0.0f, 0.0f, -M_PI/180.0f};
    auto position = Eigen::Vector3d{0.0f, 0.0f, 0.0f};

    for(int i = 0; i < 100; i++)
    {
        auto q = euler_to_quaternion(euler);
        auto T_q= body_to_ned_transformation_from_quaternion(q);
        unit_quaternion_update(0.05f, q, T_q, angular_velocity);
        euler.angles_ = quaternion_to_euler_angles(q);

        auto R_b_n = quaternion_to_rotation(q);
        position_update(0.05f, position, linear_velocity, R_b_n);

    }

    std::cout << "position: " << position.transpose() << std::endl;
    std::cout << "angles  : " << euler.degree().transpose() << std::endl;

    std::chrono::duration<double> last =
            std::chrono::steady_clock::now() - begin;
    std::cout << "time: " << last.count() << '\n';

    return 0;
}
