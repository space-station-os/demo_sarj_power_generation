
#include<vector>
#include<Eigen/Dense>
#include"space_station_design.hpp"


namespace SpaceStationSimulator {


    class SpaceStationDynamicsController {

    private:

        space_station_design::SpaceStationDesign ss_design;
        Eigen::Vector3d integral_error;

        // クォータニオン誤差から角速度に変換
        Eigen::Vector3d quaternion_error_to_angular_velocity(
            const Eigen::Vector4d& target_q,
            const Eigen::Vector4d& current_q,
            double gain = 2.0
        ) {
            Eigen::Quaterniond q_t(target_q);
            Eigen::Quaterniond q_c(current_q);
            Eigen::Quaterniond q_err = q_t * q_c.conjugate();
            if (q_err.w() < 0) q_err.coeffs() *= -1;

            Eigen::Vector3d axis = q_err.vec();
            return gain * axis;
        }

    public:

        SpaceStationDynamicsController() {
            this->ss_design = space_station_design::SpaceStationDesign();
            this->integral_error = Eigen::Vector3d::Zero();
        }

        // CMG制御関数（ジンバル角速度を出力）
        Eigen::VectorXd compute_cmg_gimbal_rates(
            const Eigen::Vector4d& target_q,
            const Eigen::Vector4d& current_q,
            const Eigen::Vector3d& current_w,
            double dt,
            const double kp, 
            const double kd,
            const double ki
        ) {

            const Eigen::Matrix3d& inertia_matrix = this->ss_design.inertia_matrix;
            const std::vector<Eigen::VectorXd>& cmg_h_cross_vec = this->ss_design.cmg_h_cross_vec;

            size_t n_cmg = cmg_h_cross_vec.size();
            Eigen::Vector3d w_desired = quaternion_error_to_angular_velocity(target_q, current_q);
            Eigen::Vector3d w_error = w_desired - current_w;

            // 
            this->integral_error += w_desired * dt;

            // Non-linear term (w~Iw)
            Eigen::Vector3d non_linear_term = current_w.cross(this->ss_design.inertia_matrix * current_w);

            // PD Control
            Eigen::Vector3d desired_w = kp * w_error - kd * current_w + ki * this->integral_error;
            Eigen::Vector3d desired_torque = inertia_matrix * desired_w + non_linear_term;

            // H_cross_matrix（3 x n_cmg）
            Eigen::MatrixXd h_cross(3, n_cmg);
            for (size_t i = 0; i < n_cmg; ++i) {
                h_cross.col(i) = cmg_h_cross_vec[i];
            }

            // Solve gimbal angular velocity by LS
            Eigen::VectorXd gimbal_rates = h_cross.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(desired_torque);

            return gimbal_rates;
        }
    };
}
