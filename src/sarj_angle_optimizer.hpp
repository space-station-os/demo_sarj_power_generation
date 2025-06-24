
#ifndef __INCLUDE_SARJ_ANGLE_OPTIMIZER_HPP__
#define __INCLUDE_SARJ_ANGLE_OPTIMIZER_HPP__

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>
#include <string>

#include "rotation.hpp"
#include "space_station_design.hpp"


namespace SarjAngleOptimizer {

    constexpr double PI = 3.141592653589793;

    inline constexpr double deg2rad(double deg) { return deg / 180.0 * PI; }
    inline constexpr double rad2deg(double rad) { return rad / PI * 180.0; }


    class SarjAngleOptimizer {

    private:

        Eigen::Vector3d sarj_rotation_axis_vec;
        Eigen::Vector3d sap_base_normal_vector;

    public:

        SarjAngleOptimizer(){
            this->sarj_rotation_axis_vec = Eigen::Vector3d(1, 0, 0);
            this->sap_base_normal_vector = Eigen::Vector3d(1, 0, 0);
        }

        SarjAngleOptimizer(const space_station_design::SpaceStationDesign& ss_design) {
            this->sarj_rotation_axis_vec = ss_design.sarj_rotation_axis.normalized();
            this->sap_base_normal_vector = ss_design.sap_base_normal_vector;
        }

        double optimize_sarj_angle(const Eigen::Vector3d& sun_direction_ssbf_vec) {

            Eigen::Vector3d normal_sun_direction_ssbf_vec = sun_direction_ssbf_vec.normalized();
            double dot = this->sarj_rotation_axis_vec.dot(normal_sun_direction_ssbf_vec);
            Eigen::Vector3d projected_sun_dir_vec = normal_sun_direction_ssbf_vec - dot * this->sarj_rotation_axis_vec;
            Eigen::Vector3d projected_sun_dir_normalized_vec = projected_sun_dir_vec.normalized();
            Eigen::Vector3d cross_vec = this->sap_base_normal_vector.cross(projected_sun_dir_normalized_vec);
            double cos_theta = this->sap_base_normal_vector.dot(projected_sun_dir_normalized_vec);
            double theta = std::acos(cos_theta);

            if (cross_vec.dot(this->sarj_rotation_axis_vec) < 0) {
                theta = -theta;
            }

            // --- for debug ---
            // Eigen::Matrix3d sarj_rot_mat = Rotation::rodrigues_rotation_matrix(this->sarj_rotation_axis_vec, theta);
            // Eigen::Vector3d ss_sap_normal_vec = sarj_rot_mat * SpaceStationDesign::SAP_BASE_NORMAL_VEC;

            return theta;
        }

    };


}

#endif
