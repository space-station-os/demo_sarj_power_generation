
#include <Eigen/Dense>
#include <cmath>


namespace Rotation {

    Eigen::Matrix3d quat2dcm(const Eigen::Vector4d& quat_vec) {

        const Eigen::Vector4d& q = quat_vec;
        auto qs = quat_vec.array() * quat_vec.array();

        Eigen::Matrix3d dcm_mat;
        
        dcm_mat.coeffRef(0, 0) = qs(0) - qs(1) - qs(2) + qs(3);
        dcm_mat.coeffRef(0, 1) = 2 * (q(0) * q(1) + q(2) * q(3));
        dcm_mat.coeffRef(0, 2) = 2 * (q(0) * q(2) - q(1) * q(3));

        dcm_mat.coeffRef(1, 0) = 2 * (q(0) * q(1) - q(2) * q(3));
        dcm_mat.coeffRef(1, 1) = qs(1) - qs(0) - qs(2) + qs(3);
        dcm_mat.coeffRef(1, 2) = 2 * (q(1) * q(2) + q(0) * q(3));

        dcm_mat.coeffRef(2, 0) = 2 * (q(0) * q(2) + q(1) * q(3));
        dcm_mat.coeffRef(2, 1) = 2 * (q(1) * q(2) - q(0) * q(3));
        dcm_mat.coeffRef(2, 2) = qs(2) - qs(0) - qs(1) + qs(3);

        return dcm_mat;
    }


    Eigen::Matrix3d euler2dcm(const Eigen::Vector3d& euler_vec) {
        auto sin_euler_vec = euler_vec.array().sin();
        auto cos_euler_vec = euler_vec.array().cos();

        double s0 = sin_euler_vec[0];
        double s1 = sin_euler_vec[1];
        double s2 = sin_euler_vec[2];
        double c0 = cos_euler_vec[0];
        double c1 = cos_euler_vec[1];
        double c2 = cos_euler_vec[2];

        Eigen::Matrix3d dcm_mat;

        dcm_mat.coeffRef(0, 0) = c1 * c2;
        dcm_mat.coeffRef(0, 1) = c1 * s2;
        dcm_mat.coeffRef(0, 2) = -s1;

        dcm_mat.coeffRef(1, 0) = -c0 * s2 + s0 * s1 * c2;
        dcm_mat.coeffRef(1, 1) = c0 * c2 + s0 * s1 * s2;
        dcm_mat.coeffRef(1, 2) = s0 * c1;

        dcm_mat.coeffRef(2, 0) = s0 * s2 + c0 * s1 * c2;
        dcm_mat.coeffRef(2, 1) = -s0 * c2 + c0 * s1 * s2;
        dcm_mat.coeffRef(2, 2) = c0 * c1;

        return dcm_mat;
    }


    Eigen::Vector4d dcm2quat(const Eigen::Matrix3d& dcm) {

        std::vector<double> temp_q_vec{
            std::sqrt(1.0 + dcm.coeff(0, 0) - dcm.coeff(1, 1) - dcm.coeff(2, 2)) / 2.0,
            std::sqrt(1.0 - dcm.coeff(0, 0) + dcm.coeff(1, 1) - dcm.coeff(2, 2)) / 2.0,
            std::sqrt(1.0 - dcm.coeff(0, 0) - dcm.coeff(1, 1) + dcm.coeff(2, 2)) / 2.0,
            std::sqrt(1.0 + dcm.coeff(0, 0) + dcm.coeff(1, 1) + dcm.coeff(2, 2)) / 2.0
        };

        Eigen::Vector4d quat_vec;

        std::vector<double>::iterator max_it = std::max_element(temp_q_vec.begin(), temp_q_vec.end());
        size_t max_idx = std::distance(temp_q_vec.begin(), max_it);

        if (max_idx == 0) {
            quat_vec.coeffRef(1) = dcm.coeff(0, 1) + dcm.coeff(1, 0);
            quat_vec.coeffRef(2) = dcm.coeff(0, 2) + dcm.coeff(2, 0);
            quat_vec.coeffRef(3) = dcm.coeff(1, 2) - dcm.coeff(2, 1);
        }
        else if (max_idx == 1) {
            quat_vec.coeffRef(0) = dcm.coeff(0, 1) + dcm.coeff(1, 0);
            quat_vec.coeffRef(2) = dcm.coeff(2, 1) + dcm.coeff(1, 2);
            quat_vec.coeffRef(3) = dcm.coeff(2, 0) - dcm.coeff(0, 2);
        }
        else if (max_idx == 2) {
            quat_vec.coeffRef(0) = dcm.coeff(2, 0) + dcm.coeff(0, 2);
            quat_vec.coeffRef(1) = dcm.coeff(2, 1) + dcm.coeff(1, 2);
            quat_vec.coeffRef(3) = dcm.coeff(0, 1) - dcm.coeff(1, 0);
        }
        else {
            quat_vec.coeffRef(0) = dcm.coeff(1, 2) - dcm.coeff(2, 1);
            quat_vec.coeffRef(1) = dcm.coeff(2, 0) - dcm.coeff(0, 2);
            quat_vec.coeffRef(2) = dcm.coeff(0, 1) - dcm.coeff(1, 0);
        }

        quat_vec *= (0.25 / temp_q_vec[max_idx]);
        quat_vec[max_idx] = temp_q_vec[max_idx];

        return quat_vec;
    }


    Eigen::Matrix3d rodrigues_rotation_matrix(Eigen::Vector3d axis_vec, double angle){
        return Eigen::AngleAxis<double>(angle, axis_vec).matrix();
    }

};
