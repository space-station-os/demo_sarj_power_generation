
#ifndef __SPACE_STATION_DESIGN__
#define __SPACE_STATION_DESIGN__


#include <Eigen/Dense>


namespace space_station_design
{

    class SpaceStationDesign {

    public:

        // Mass properties
        double total_mass = 2000e3;
        Eigen::Vector3d cg_position;
        Eigen::Matrix3d inertia_matrix;

        size_t n_thruster = 12;
        // Thruster position @BFF
        Eigen::MatrixXd thruster_position;
        Eigen::MatrixXd thruster_orientation;
        // Rating thruster force [N]
        double rating_thruster_force = 100.0 ;
        // Thruster moment action matrix
        Eigen::MatrixXd thruster_mam;

        // ---- Control Momentum Gyroscope ----
        size_t n_cmg = 4;
        // parameters set by user
        std::vector<Eigen::Vector3d> cmg_spin_axis_vec;
        std::vector<Eigen::Vector3d> cmg_gimbal_axis_vec;
        std::vector<double> cmg_angular_momentum_vec;
        // const
        std::vector<Eigen::VectorXd> cmg_h_cross_vec;

        // Maximum rate of CMD gimbal [rad/s]
        double max_cmg_gimbal_rate = 4.0;

        // Rotation axis of SARJ at SSBF frame
        Eigen::Vector3d sarj_rotation_axis ;
        Eigen::Vector3d sap_base_normal_vector ;

        SpaceStationDesign() {

            this->cg_position = Eigen::Vector3d(0, 0, 0);
            this->inertia_matrix <<
                3000, 0, 0,
                0, 1500, 0,
                0, 0, 2000;

            this->thruster_position = Eigen::MatrixXd(3, this->n_thruster);
            this->thruster_position <<
                -8, -8, +8, +8,  0,  0,  0,  0, -4, -4, +4, +4,
                -2, +2, -2, +2, -1, -1, +1, +1,  0,  0,  0,  0,
                 0,  0,  0,  0, -1, +1, -1, +1, -2, +2, -2, +2;

            this->thruster_orientation = Eigen::MatrixXd(3, this->n_thruster);
            this->thruster_orientation <<
                -1, -1, +1, +1,  0,  0,  0,  0,  0,  0,  0,  0,
                 0,  0,  0,  0, -1, -1, +1, +1,  0,  0,  0,  0,
                 0,  0,  0,  0,  0,  0,  0,  0, -1, -1, +1, +1;

            // Normalize
            for (size_t i = 0; i < this->n_thruster; ++i) {
                auto v = this->thruster_orientation.col(i);
                auto normalized_v = v.normalized();
                this->thruster_orientation.col(i) = normalized_v;
            }

            this->thruster_mam = Eigen::MatrixXd(3, this->n_thruster);
            for (size_t i = 0; i < this->n_thruster; ++i) {
                // position vector from center of the mass
                Eigen::Vector3d r = this->thruster_position.col(i) - this->cg_position;       
                Eigen::Vector3d f = -this->thruster_orientation.col(i);
                Eigen::Vector3d torque = r.cross(f);
                this->thruster_mam.col(i) = torque;
            }

            // -------- CMG --------
            this->n_cmg = 4;
            this->cmg_spin_axis_vec.resize(this->n_cmg);
            this->cmg_gimbal_axis_vec.resize(this->n_cmg);
            this->cmg_angular_momentum_vec.resize(this->n_cmg);

            this->cmg_spin_axis_vec[0] = Eigen::Vector3d::UnitX();
            this->cmg_gimbal_axis_vec[0] = Eigen::Vector3d::UnitY();
            this->cmg_angular_momentum_vec[0] = 4.0;

            this->cmg_spin_axis_vec[1] = Eigen::Vector3d::UnitY();
            this->cmg_gimbal_axis_vec[1] = Eigen::Vector3d::UnitZ();
            this->cmg_angular_momentum_vec[1] = 4.0;

            this->cmg_spin_axis_vec[2] = Eigen::Vector3d::UnitZ();
            this->cmg_gimbal_axis_vec[2] = Eigen::Vector3d::UnitX();
            this->cmg_angular_momentum_vec[2] = 4.0;

            this->cmg_spin_axis_vec[3] = Eigen::Vector3d(1,1,1).normalized();
            this->cmg_gimbal_axis_vec[3] = Eigen::Vector3d::UnitZ();
            this->cmg_angular_momentum_vec[3] = 4.0;

            this->cmg_h_cross_vec.resize(this->n_cmg);
            for (size_t i = 0; i < this->n_cmg; ++i) {
                this->cmg_h_cross_vec[i] = this->cmg_angular_momentum_vec[i] * this->cmg_spin_axis_vec[i].cross(this->cmg_gimbal_axis_vec[i]);
            }

            // -------- SARJ --------
            // Rotation axis of SARJ at SSBF frame
            this->sarj_rotation_axis = Eigen::Vector3d(0.0, 1.0, 0.0).normalized();
            this->sap_base_normal_vector = Eigen::Vector3d(0.0, 0.0, -1.0).normalized();
        }
    };


    // Thruster orientation


} // namespace SpaceStationDesig

#endif
