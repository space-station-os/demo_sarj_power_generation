
#ifndef __SPACE_STATION_SIMULATOR__
#define __SPACE_STATION_SIMULATOR__


#include <iostream>
#include <cmath>
#include <string>
#include <iomanip>
#include <Eigen/Dense>
#include <chrono>
#include <map>
#include <vector>

#include "space_station_design.hpp"
#include "space_station_dynamics_controller.hpp"
#include "rotation.hpp"


namespace Math {
    constexpr double PI = 3.141592653589793;
    constexpr double TWO_PI = 2.0 * PI;

    inline constexpr double deg2rad(double deg) { return deg / 180.0 * Math::PI; }
    inline constexpr double rad2deg(double rad) { return rad / Math::PI * 180.0; }

}


namespace OrbitLib {

    // The Earth radius [m]
    constexpr double EARTH_RADIUS = 6378.14 * 1e3;
    // [m^3 s^-2]
    constexpr double G_ME = 3.986004418e14;

    // Ditstance from the Sun to the Earth [m]
    constexpr double SUN_EARTH_DISTANCE = 149600000 * 1e3;

    // J2
    constexpr double J2 = 1.08263e-3;

    // Earth Constant (WGS84)
    // major axis [m]
    const double WGS84_A = 6378137.0;
    // flattening
    const double WGS84_F = 1.0 / 298.257223563;
    // square of eccentricity
    const double WGS84_E2 = WGS84_F * (2 - WGS84_F);
    // Earth rotation angular velocity [rad/s]
    const double EARTH_OMEGA = 7.2921150e-5;

    // ======== Mean motion [rad/s] -> semi-major axix [m] ========
    double mean_motion_to_a(double n) {
        return std::pow(G_ME / (n * n), 1.0 / 3.0);
    }

    // ======== Calculate true anomaly by solving kepler equation ========
    double mean_to_true_anomaly(double M, double e, double tol = 1e-8) {
        M = std::fmod(M, Math::TWO_PI);
        double E = M;
        for (int i = 0; i < 100; ++i) {
            double f = E - e * sin(E) - M;
            double df = 1 - e * cos(E);
            double dE = f / df;
            E -= dE;
            if (fabs(dE) < tol) break;
        }
        double cosE = cos(E);
        double sinE = sin(E);
        double true_anom = atan2(sqrt(1 - e * e) * sinE, cosE - e);
        return true_anom;
    }

    // ======== Extract kepler elements from line 2 of TLE ========
    void extract_element_from_tle_line2(
        std::string line2,
        double& mean_motion_rev_per_day,
        double& eccentricity_no_decimal,
        double& inclination_deg,
        double& raan_deg,
        double& argp_deg,
        double& mean_anomaly_deg
    ) {
        inclination_deg = std::stod(line2.substr(8, 8));
        raan_deg = std::stod(line2.substr(17, 8));
        eccentricity_no_decimal = std::stod("0." + line2.substr(26, 7));
        argp_deg = std::stod(line2.substr(34, 8));
        mean_anomaly_deg = std::stod(line2.substr(43, 8));
        mean_motion_rev_per_day = std::stod(line2.substr(52, 11));
    }

    // ======== Calculate ECI position and velocity from TLE elements ========
    void convert_tle_element_to_eci(
        double mean_motion_rev_per_day,
        double eccentricity_no_decimal,
        double inclination_deg,
        double raan_deg,
        double argp_deg,
        double mean_anomaly_deg,
        Eigen::Vector3d& position,
        Eigen::Vector3d& velocity
    ) {
        double n = mean_motion_rev_per_day * Math::TWO_PI / 86400.0; // rad/s
        double a = mean_motion_to_a(n); // [m]
        double e = eccentricity_no_decimal;
        double i = Math::deg2rad(inclination_deg);
        double raan = Math::deg2rad(raan_deg);
        double argp = Math::deg2rad(argp_deg);
        double M = Math::deg2rad(mean_anomaly_deg);
        double nu = mean_to_true_anomaly(M, e);

        // Other related values
        double p = a * (1 - e * e);
        double r = p / (1 + e * cos(nu));
        double x_p = r * cos(nu);
        double y_p = r * sin(nu);

        double vx_p = -sqrt(OrbitLib::G_ME / p) * sin(nu);
        double vy_p = sqrt(OrbitLib::G_ME / p) * (e + cos(nu));

        // Rotation matrix (Z-X-Z rotation)
        Eigen::Matrix3d r_mat =
            Eigen::AngleAxisd(raan, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(i, Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(argp, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // Tranclate position and velocity into ECI
        Eigen::Vector3d r_p(x_p, y_p, 0.0);
        Eigen::Vector3d v_p(vx_p, vy_p, 0.0);
        position = r_mat * r_p;
        velocity = r_mat * v_p;
    }


    void convert_tle_to_eci(
        std::string tle_line2, 
        Eigen::Vector3d& ss_position_eci, Eigen::Vector3d& ss_velocity_eci
    ) {

        double mean_motion_rev_per_day;
        double eccentricity_no_decimal;
        double inclination_deg;
        double raan_deg;
        double argp_deg;
        double mean_anomaly_deg;

        // Extract TLE element
        OrbitLib::extract_element_from_tle_line2(
            tle_line2,
            mean_motion_rev_per_day,
            eccentricity_no_decimal,
            inclination_deg,
            raan_deg,
            argp_deg,
            mean_anomaly_deg
        );

        OrbitLib::convert_tle_element_to_eci(
            mean_motion_rev_per_day,
            eccentricity_no_decimal,
            inclination_deg,
            raan_deg,
            argp_deg,
            mean_anomaly_deg,
            ss_position_eci, ss_velocity_eci
        );

    }

    // ======== UTC -> GMST (simple. Error is small if a several days.) ========
    double unix_time_to_gmst(int64_t unix_seconds, int microseconds) {
        // Seconds since UNIX epoch (1970-01-01T00:00:00 UTC) to J2000 starting point
        // Julian Day of Unix Epoch
        const double JD_UNIX_EPOCH = 2440587.5;   
        // Julian Day of J2000 Epoch
        const double JD_J2000 = 2451545.0;        
        double dt_days = (double)(unix_seconds) / 86400.0 + (double)(microseconds) / (86400.0 * 1e6);
        double jd = JD_UNIX_EPOCH + dt_days;
        double d = jd - JD_J2000;

        // GMST in seconds
        double gmst_sec = 67310.54841 + (876600.0 * 3600.0 + 8640184.812866) * d / 36525.0
            + 0.093104 * (d / 36525.0) * (d / 36525.0)
            - 6.2e-6 * (d / 36525.0) * (d / 36525.0) * (d / 36525.0);

        gmst_sec = fmod(gmst_sec, 86400.0);
        if (gmst_sec < 0) gmst_sec += 86400.0;

        double gmst_rad = Math::TWO_PI * (gmst_sec / 86400.0);
        return gmst_rad; // [rad]
    }

    void eci_to_geodetic(
        const Eigen::Vector3d& position_eci_m,
        int64_t unix_seconds,
        int microseconds,
        double& latitude_deg,
        double& longitude_deg,
        double& altitude_m
    ) {
        // Step1: ECI -> ECEF rotation
        double gmst = unix_time_to_gmst(unix_seconds, microseconds);
        Eigen::Matrix3d R;
        R <<
            cos(gmst), sin(gmst), 0,
            -sin(gmst), cos(gmst), 0,
            0, 0, 1;
        Eigen::Vector3d position_ecef_m = R * position_eci_m;

        double x = position_ecef_m.x();
        double y = position_ecef_m.y();
        double z = position_ecef_m.z();

        // Step2: ECEF -> lat & lon & alt
        double r = sqrt(x * x + y * y);
        double lon = atan2(y, x);

        // Set tempolary phis as initial value
        double lat = atan2(z, r);
        double alt = 0.0;

        double N = 0.0;
        double lat_prev = 0.0;
        // Convergence condition
        const double tol = 1e-8; 

        for (int iter = 0; iter < 10; ++iter) {
            lat_prev = lat;
            N = WGS84_A / sqrt(1 - WGS84_E2 * sin(lat) * sin(lat));
            alt = r / cos(lat) - N;
            lat = atan2(z, r * (1 - WGS84_E2 * (N / (N + alt))));
            if (fabs(lat - lat_prev) < tol) {
                break;
            }
        }

        latitude_deg = Math::rad2deg(lat);
        longitude_deg = Math::rad2deg(lon);
        altitude_m = alt;
    }
}


namespace EigenUtil {

    double two_vector_angle_rad(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return std::acos(a.normalized().dot(b.normalized()));
    }

}


namespace SpaceStationSimulator {

    void keplerian_to_cartesian(
        double a, double e, double i_deg, double raan_deg, double argp_deg, double nu_deg,
        Eigen::Vector3d& position_eci, Eigen::Vector3d& velocity_eci
    ) {
        double i = Math::deg2rad(i_deg);
        double raan = Math::deg2rad(raan_deg);
        double argp = Math::deg2rad(argp_deg);
        double nu = Math::deg2rad(nu_deg);

        double p = a * (1 - e * e);
        double r = p / (1 + e * std::cos(nu));

        // Position and velocity vector at PQW frame
        Eigen::Vector3d r_pqw(
            r * std::cos(nu),
            r * std::sin(nu),
            0.0
        );

        Eigen::Vector3d v_pqw(
            -std::sqrt(OrbitLib::G_ME / p) * std::sin(nu),
            std::sqrt(OrbitLib::G_ME / p) * (e + std::cos(nu)),
            0.0
        );

        // Rotation Matrix PQW -> ECI
        Eigen::Matrix3d r_mat;
        r_mat = Eigen::AngleAxisd(-raan, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(-i, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(-argp, Eigen::Vector3d::UnitZ());

        position_eci = r_mat * r_pqw;
        velocity_eci = r_mat * v_pqw;

        return;
    }


    Eigen::Vector4d quaternion_diff_equ(const Eigen::Vector4d& q_vec, const Eigen::Vector3d& w_vec) {
        // Differential equation of quqternion
        auto r = w_vec[0];
        auto p = w_vec[1];
        auto y = w_vec[2];

        Eigen::Matrix4d sqew_mat;
        sqew_mat <<
            0, +y, -p, +r,
            -y, 0, +r, +p,
            +p, -r, 0, +y,
            -r, -p, -y, 0;

        auto dq_vec = 0.5 * sqew_mat * q_vec;
        return dq_vec;
    }


    class OrbitAccelerationModel {

    private:
        // ---- J2 ----
        bool consider_j2;
        double j2_coefficient;
        
        // ---- Air drag ----
        bool consider_air_drag;
        double cd;
        // mass [kg]
        double mass;
        // surface area [m^2]
        double a{1.0};
        // angular velocity of the Earth [rad/s]
        //double earth_omega;

        // ======== Atmosphic density model (simple) ========
        double atmospheric_density(double altitude) {
            // altitude: altitude [m]
            // return: atmosphic density [kg/m^3]
            if (altitude < 150e3) return 4e-9;
            if (altitude < 200e3) return 1e-9;
            if (altitude < 300e3) return 1e-10;
            if (altitude < 400e3) return 1e-11;
            if (altitude < 500e3) return 1e-12;
            return 1e-13;
        }

    public:
        OrbitAccelerationModel() :
            consider_j2(false), j2_coefficient(0.0),
            consider_air_drag(false), cd(0.0), mass(1.0)
        {}
        
        void activate_j2(double j2_coefficient) {
            this->consider_j2 = true;
            this->j2_coefficient = j2_coefficient;
        }

        void activate_air_drag(double cd, double mass, double radius) {
            // radius: 
            this->consider_air_drag = true;
            this->cd = cd;
            this->mass = mass;
            this->a = Math::PI * radius * radius;
        }

        Eigen::Vector3d calc_leo_acc_gravity_term(const Eigen::Vector3d& position) {
            double r_norm = position.norm();
            double r_norm3 = r_norm * r_norm * r_norm;
            Eigen::Vector3d acceleration = position * (-OrbitLib::G_ME / r_norm3);
            return acceleration;
        }

        Eigen::Vector3d calc_leo_acc_j2_term(const Eigen::Vector3d& position) {
            const double& x = position[0];
            const double& y = position[1];
            const double& z = position[2];

            double z2 = z * z;
            double r2 = x * x + y * y + z2;
            double r_norm = std::sqrt(r2);
            double r5 = r2 * r2 * r_norm;

            double factor = 1.5 * OrbitLib::J2 * OrbitLib::G_ME * OrbitLib::EARTH_RADIUS * OrbitLib::EARTH_RADIUS / r5;
            double term = 5.0 * z2 / r2;

            Eigen::Vector3d acc_J2 = {
                x * (1 - term),
                y * (1 - term),
                z * (3 - term * 5)
            };
            return acc_J2 * factor;
        }

        Eigen::Vector3d calc_leo_acc_air_drag_term(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
            double r_norm = position.norm();
            Eigen::Vector3d v_atm = {
                -OrbitLib::EARTH_OMEGA * position[1],
                OrbitLib::EARTH_OMEGA * position[0],
                0.0
            };
            Eigen::Vector3d v_rel = velocity - v_atm;
            double alt = r_norm - OrbitLib::EARTH_RADIUS;
            double rho = this->atmospheric_density(alt); // [m]
            return v_rel * (-0.5 * this->cd * this->a / this->mass * rho * v_rel.norm());
        }

        Eigen::Vector3d calc_acceleration_on_leo(const Eigen::Vector3d& position) {
            // Dynamics on LEO (low earth orbit) as differential equation
            Eigen::Vector3d acc = this->calc_leo_acc_gravity_term(position);

            if (this->consider_j2) {
                acc += this->calc_leo_acc_j2_term(position);
            }
            return acc;
        }

        Eigen::Vector3d calc_acceleration_on_leo(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
            // Dynamics on LEO (low earth orbit) as differential equation
            Eigen::Vector3d acc = this->calc_leo_acc_gravity_term(position);

            if (this->consider_j2) {
                acc += this->calc_leo_acc_j2_term(position);
            }
            if (this->consider_air_drag) {
                acc += this->calc_leo_acc_air_drag_term(position, velocity);
            }
            return acc;
        }

    };
    

    class FrameTransformer
    {
    private:
        Eigen::Matrix3d local_frame_basis_mat{ Eigen::Matrix3d::Identity() };
        Eigen::Matrix3d inv_local_frame_basis_mat{ Eigen::Matrix3d::Identity() };
        Eigen::Vector3d local_frame_origin_vec{ Eigen::Vector3d::Zero() };

    public:

        FrameTransformer() {}

        FrameTransformer(const Eigen::Matrix3d& local_frame_basis_mat, const Eigen::Vector3d& local_frame_origin_vec) {
            this->local_frame_basis_mat = local_frame_basis_mat;
            this->local_frame_origin_vec = local_frame_origin_vec;
            this->inv_local_frame_basis_mat = local_frame_basis_mat.inverse();
        }

        Eigen::Vector3d get_local_pos(const Eigen::Vector3d& global_pos_vec) const {
            return this->inv_local_frame_basis_mat * (global_pos_vec - this->local_frame_origin_vec);
        }

        Eigen::Vector3d get_global_pos(const Eigen::Vector3d& local_pos_vec) const {
            return this->local_frame_basis_mat * local_pos_vec + this->local_frame_origin_vec;
        }

        void update_basis_mat(const Eigen::Matrix3d& local_frame_basis_mat) {
            this->local_frame_basis_mat = local_frame_basis_mat;
            this->inv_local_frame_basis_mat = local_frame_basis_mat.inverse();
        }

        void update_origin_vec(const Eigen::Vector3d& local_frame_origin_vec) {
            this->local_frame_origin_vec = local_frame_origin_vec;
        }

        const Eigen::Matrix3d& get_local_frame_basis_mat() const {
            return this->local_frame_basis_mat;
        }

        const Eigen::Matrix3d& get_inv_local_frame_basis_mat() const {
            return this->inv_local_frame_basis_mat;
        }

        const Eigen::Vector3d& get_local_frame_origin_vec() const {
            return this->local_frame_origin_vec;
        }
    };



    class SpaceStationSimulator
    {

    private:

        OrbitAccelerationModel orbit_acc_model;

        // -------- variables --------

        double t{0.0};

        // ---- Parameters of the Earth ----
        // Initial phase [rad]
        double earth_init_phase{0.0};
        // Angular Velocity of revolution [rad/s]
        double earth_revolution_w{ 2.0 * Math::PI / (365.25 * 24 * 60 * 60) };

        // ---- Parameters of the Space Station ----  
        // Normal vector of SAP @BF
        Eigen::Vector3d ss_sap_normal_vec{ Eigen::Vector3d() };
        // Anlugar velocity of revolution [rad/s] 
        double ss_revolution_w{0.0};

        FrameTransformer ss_plane_inertia_ft{};

        // ---- Parameters of the Earth ----  

        Eigen::Vector3d earth_pos_vec{ Eigen::Vector3d::Zero()};
        Eigen::Vector3d ss_position_eci{ Eigen::Vector3d::Zero() };
        Eigen::Vector3d ss_velocity_eci{ Eigen::Vector3d::Zero() };
        Eigen::Vector3d ss_acceleration_eci{ Eigen::Vector3d::Zero() };

        // Global:SCI, Local:ECI
        FrameTransformer sci_eci_ft{};
        // Global:ECI, Local:SSBF
        FrameTransformer eci_ssbf_ft{};

        // Quaternion of Space Station
        Eigen::Vector4d ss_quaternion_vec{ Eigen::Vector4d() };
        // Angular velocity of SS
        Eigen::Vector3d ss_w_vec{ Eigen::Vector3d() };

        // ---- Parameters of SAP ----
        // Maximum power generation [W]
        double max_generated_power{0.0};
        // Full amount of battery [Wh]
        double full_battery_amount{0.0};
        // Maximum power generation [W]
        double cur_generated_power{0.0};
        // Current amount of battery [Wh]
        double cur_battery_amount{0.0};
        // Consumed power by other subsystem [W]
        double ss_power_consumption{0.0};

        // Space Station is in shade of the Earth or not
        bool ss_in_sunlight{false};

        space_station_design::SpaceStationDesign ssd = space_station_design::SpaceStationDesign();

        // SARJ rotation axis & SAP normal vector (normalized)
        Eigen::Vector3d sarj_rotation_axis_vec ;
        Eigen::Vector3d ss_sap_basic_normal_vec ;
        // Solar array rotary joint angle [rad]
        double sarj_angle{ 0.0 };

        // ---- Control ----
        int32_t attitude_control_plan{0};

        SpaceStationDynamicsController ss_controller;
        double control_kp;
        double control_ki;
        double control_kd;

        // thruster firng duty [0.0 ~ 1.0]
        Eigen::VectorXd thruster_firing_duty;

        // CMG
        Eigen::VectorXd cmg_gimbal_rate;

        Eigen::Vector3d calc_earth_pos_vec(double t) const {
            // -------- Calculate earth position @SCI at t --------
            double cur_phase = this->earth_init_phase + this->earth_revolution_w * t;
            Eigen::Vector3d earth_pos_vec(
                std::cos(cur_phase) * OrbitLib::SUN_EARTH_DISTANCE,
                std::sin(cur_phase) * OrbitLib::SUN_EARTH_DISTANCE,
                0.0
            );
            return earth_pos_vec;
        }

        Eigen::Vector3d calc_ss_sap_normal_vec() const {
            // -------- Calculate normal vector of SAP in SSBF --------
            Eigen::Matrix3d sarj_rot_mat = Rotation::rodrigues_rotation_matrix(this->sarj_rotation_axis_vec, this->sarj_angle);
            Eigen::Vector3d ss_sap_normal_vec = sarj_rot_mat * this->ss_sap_basic_normal_vec;
            return ss_sap_normal_vec;
        }

        Eigen::Vector3d calc_thruster_force() const {
            Eigen::Vector3d thruster_force = Eigen::Vector3d::Zero();
            for (size_t i = 0; i < this->ssd.n_thruster; ++i) {
                thruster_force += -this->ssd.thruster_orientation.col(i) * this->ssd.rating_thruster_force * this->thruster_firing_duty[i];
            }
            return thruster_force;
        }

        Eigen::Vector3d calc_cmg_torque() const {
            Eigen::Vector3d cmg_torque = Eigen::Vector3d::Zero();
            for (size_t i = 0; i < this->ssd.n_cmg; ++i) {
                cmg_torque += this->cmg_gimbal_rate[i] * this->ssd.cmg_h_cross_vec[i];
            }
            return cmg_torque;
        }


    public:

        // ========  ========
        SpaceStationSimulator()
        {
        } 

        void activate_propagation_j2(double j2_coefficient) {
            this->orbit_acc_model.activate_j2(j2_coefficient);
        }

        void activate_propagation_air_drag(double cd, double mass, double radius) {
            this->orbit_acc_model.activate_air_drag(cd, mass, radius);
        }

        void initialize(
            const Eigen::Vector3d& ss_position_eci, const Eigen::Vector3d& ss_velocity_eci,
            const Eigen::Vector3d& ss_init_euler_vec,
            const Eigen::Vector3d& ss_init_w_vec,
            int32_t attitude_control_plan
        )
        {
            // ---- Parameters of the Earth ----
            // initialized in NSDMI

            // ---- Parameters of the Space Station ----

            // -------- Control --------
            // attitude_control_plan
            // - 0: No control
            // - 1: LVLH (manual control)
            // - 2: LVLH (auto control)
            this->attitude_control_plan = attitude_control_plan;

            // -------- Initialize time-varying parameters --------
            this->t = 0.0;

            this->earth_pos_vec = this->calc_earth_pos_vec(this->t);

            this->ss_position_eci = ss_position_eci;
            this->ss_velocity_eci = ss_velocity_eci;
            this->ss_acceleration_eci = this->orbit_acc_model.calc_acceleration_on_leo(this->ss_position_eci);

            this->sci_eci_ft = FrameTransformer(
                Eigen::Matrix3d::Identity(), this->earth_pos_vec
            );
            this->eci_ssbf_ft = FrameTransformer(
                Eigen::Matrix3d::Identity(), this->ss_position_eci
            );

            Eigen::Matrix3d ss_attitude_rot_mat = Rotation::euler2dcm(ss_init_euler_vec).transpose();
            this->ss_quaternion_vec = Rotation::dcm2quat(ss_attitude_rot_mat);
            this->ss_w_vec = ss_init_w_vec;

            // ---- Power generation ----
            this->max_generated_power = 1000.0 / 6;
            this->full_battery_amount = 1000.0 * 2 * 60 * 60;
            this->cur_battery_amount = this->full_battery_amount;
            this->ss_power_consumption = 100.0;

            this->sarj_angle = 0.0;

            this->sarj_rotation_axis_vec = this->ssd.sarj_rotation_axis;
            this->ss_sap_basic_normal_vec = this->ssd.sap_base_normal_vector;

            std::cout << this->sarj_rotation_axis_vec << std::endl;
            std::cout << this->ss_sap_basic_normal_vec << std::endl;

            this->ss_sap_normal_vec = this->calc_ss_sap_normal_vec();

            this->thruster_firing_duty = Eigen::VectorXd::Zero(this->ssd.n_thruster);

            this->cmg_gimbal_rate = Eigen::VectorXd::Zero(this->ssd.n_cmg);

            this->ss_controller = SpaceStationDynamicsController();
            this->control_kp = 3.0e-3;
            this->control_ki = 1.5e-3;
            this->control_kd = 1.0e-3;
        }


        Eigen::Vector3d calc_dw(const Eigen::Vector3d& ss_w, const Eigen::Vector3d& torque_vec) const {
            Eigen::Vector3d rhs = torque_vec - ss_w.cross(this->ssd.inertia_matrix * ss_w);
            // Solve I * dw = rhs without inverting I
            Eigen::Vector3d dw = this->ssd.inertia_matrix.ldlt().solve(rhs);
            return dw;
        }

        Eigen::Vector3d calc_dw_by_actuator(const Eigen::Vector3d& ss_w) const {
            Eigen::Vector3d cmg_torque_vec = this->calc_cmg_torque();
            Eigen::Vector3d thruster_torque_vec = this->ssd.thruster_mam * this->thruster_firing_duty * this->ssd.rating_thruster_force;
            Eigen::Vector3d dw = this->calc_dw(ss_w, cmg_torque_vec + thruster_torque_vec);
            return dw;
        }

        Eigen::Vector3d calc_dw_by_leo_environment(const Eigen::Vector3d& ss_pos_eci, const Eigen::Vector4d& ss_q, const Eigen::Vector3d& ss_w) {
            // ---- Calculate gravity gradient torque ----
            auto r = ss_pos_eci.norm();
            auto r5 = r*r*r*r*r;
            // calc DCM
            auto dcm = Rotation::quat2dcm(ss_q);
            // convert position vector to BF
            auto ss_pos_bf = dcm.transpose() * ss_pos_eci;
            auto torque_vec = (3.0 * OrbitLib::G_ME / r5 * ss_pos_bf.cross(this->ssd.inertia_matrix * ss_pos_bf));
            Eigen::Vector3d dw = this->calc_dw(ss_w, torque_vec);
            
            return dw;
        }

        void update(double new_t) {

            // Simulation time
            double dt = new_t - this->t;
            if (dt < 0) {
                return;
            }

            this->t = new_t;

            // -------- Store old values --------
            Eigen::Vector3d old_earth_pos_vec = this->earth_pos_vec;
            Eigen::Vector3d old_ss_pos_vec = this->ss_position_eci;
            Eigen::Vector3d old_ss_vel_vec = this->ss_velocity_eci;

            Eigen::Vector4d old_ss_quaternion_vec = this->ss_quaternion_vec;
            Eigen::Vector3d old_ss_w_vec = this->ss_w_vec;
            
            // Eigen::Vector3d old_ss_acc_vec = this->ss_acc_vec;
            Eigen::Vector3d old_sap_normal_vec = this->ss_sap_normal_vec;
            Eigen::Vector3d old_sun_direction_vec = this->get_sun_pos_at_ss_vec();

            // -------- Dynamics --------
            
            // ---- External force and acceleration to SS ----
            Eigen::Vector3d ss_ext_force_ssbf = Eigen::Vector3d::Zero();
            // 
            ss_ext_force_ssbf += this->calc_thruster_force();
            Eigen::Vector3d ss_ext_acc_ssbf = ss_ext_force_ssbf / this->ssd.total_mass;

            // Transform
            Eigen::Vector3d ss_ext_acc_eci = this->eci_ssbf_ft.get_local_frame_basis_mat() * ss_ext_acc_ssbf;

            // ---- Runge–Kutta method ----
            double dt_h = 0.5 * dt;
            double dt_six = dt / 6.0;

            // -- k1 --
            // - Position -
            Eigen::Vector3d other_acc_k1 = ss_ext_acc_eci;
            Eigen::Vector3d k1_v = this->orbit_acc_model.calc_acceleration_on_leo(old_ss_pos_vec, old_ss_vel_vec) + other_acc_k1;
            Eigen::Vector3d k1_r = old_ss_vel_vec;
            // - Attitude -
            auto k1_w_act = this->calc_dw_by_actuator(old_ss_w_vec);
            auto k1_w_env = this->calc_dw_by_leo_environment(old_ss_pos_vec, old_ss_quaternion_vec, old_ss_w_vec);
            Eigen::Vector3d k1_w = k1_w_act + k1_w_env;
            auto k1_q = quaternion_diff_equ(old_ss_quaternion_vec, old_ss_w_vec);

            // -- k2 --
            // - Position -
            Eigen::Vector3d other_acc_k2 = ss_ext_acc_eci;
            Eigen::Vector3d pos_k2 = old_ss_pos_vec + dt_h * k1_r;
            Eigen::Vector3d vel_k2 = old_ss_vel_vec + dt_h * k1_v;
            Eigen::Vector3d k2_v = this->orbit_acc_model.calc_acceleration_on_leo(pos_k2, vel_k2) + other_acc_k2;
            Eigen::Vector3d k2_r = vel_k2;
            // - Attitude -
            auto w_for_k2 = old_ss_w_vec + dt_h * k1_w;
            auto q_for_k2 = (old_ss_quaternion_vec + dt_h * k1_q).normalized();
            auto k2_w_act = this->calc_dw_by_actuator(w_for_k2);
            auto k2_w_env = this->calc_dw_by_leo_environment(pos_k2, q_for_k2, w_for_k2);
            auto k2_w = k2_w_act + k2_w_env;
            auto k2_q = quaternion_diff_equ(q_for_k2, w_for_k2);

            // -- k3 --
            // - Position -
            Eigen::Vector3d other_acc_k3 = ss_ext_acc_eci;
            Eigen::Vector3d pos_k3 = old_ss_pos_vec + dt_h * k2_r;
            Eigen::Vector3d vel_k3 = old_ss_vel_vec + dt_h * k2_v;
            Eigen::Vector3d k3_v = this->orbit_acc_model.calc_acceleration_on_leo(pos_k3, vel_k3) + other_acc_k3;
            Eigen::Vector3d k3_r = vel_k3;
            // - Attitude -
            auto w_for_k3 = old_ss_w_vec + dt_h * k2_w;
            auto q_for_k3 = (old_ss_quaternion_vec + dt_h * k2_q).normalized();
            auto k3_w_act = this->calc_dw_by_actuator(w_for_k3);
            auto k3_w_env = this->calc_dw_by_leo_environment(pos_k3, q_for_k3, w_for_k3);
            auto k3_w = k3_w_act + k3_w_env;
            auto k3_q = quaternion_diff_equ(q_for_k3, w_for_k3);

            // -- k4 --
            // - Position -
            Eigen::Vector3d other_acc_k4 = ss_ext_acc_eci;
            Eigen::Vector3d pos_k4 = old_ss_pos_vec + dt * k3_r;
            Eigen::Vector3d vel_k4 = old_ss_vel_vec + dt * k3_v;
            Eigen::Vector3d k4_v = this->orbit_acc_model.calc_acceleration_on_leo(pos_k4, vel_k4) + other_acc_k4;
            Eigen::Vector3d k4_r = vel_k4;
            // - Attitude -
            auto w_for_k4 = old_ss_w_vec + dt * k3_w;
            auto q_for_k4 = (old_ss_quaternion_vec + dt * k3_q).normalized();
            auto k4_w_act = this->calc_dw_by_actuator(w_for_k4);
            auto k4_w_env = this->calc_dw_by_leo_environment(pos_k4, q_for_k4, w_for_k4);
            auto k4_w = k4_w_act + k4_w_env;
            auto k4_q = quaternion_diff_equ(q_for_k4, w_for_k4);

            // -- add --
            // - Position -
            this->ss_acceleration_eci = dt_six * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);
            this->ss_velocity_eci = old_ss_vel_vec + this->ss_acceleration_eci;
            this->ss_position_eci = old_ss_pos_vec + dt_six * (k1_r + 2.0 * k2_r + 2.0 * k3_r + k4_r);
            // - Attitude -
            this->ss_w_vec = old_ss_w_vec + (k1_w + 2.0 * k2_w + 2.0 * k3_w + k4_w) * dt_six;
            this->ss_quaternion_vec = old_ss_quaternion_vec + (k1_q + 2.0 * k2_q + 2.0 * k3_q + k4_q) * dt_six;
            this->ss_quaternion_vec.normalize();

            //this->ss_quaternion_vec = old_ss_quaternion_vec;

            Eigen::Matrix3d ss_rot_mat = Rotation::quat2dcm(this->ss_quaternion_vec);

            // ---- Position & Velocity ----
            this->earth_pos_vec = this->calc_earth_pos_vec(this->t);

            //// ---- Attitude ----

            if (this->attitude_control_plan == 1) {
                // --- LVLH Control () ---

                // Z-basis
                Eigen::Vector3d rot_z_vec = -old_ss_pos_vec.normalized();
                // Y-basis
                Eigen::Vector3d rot_y_vec = -old_ss_vel_vec.normalized().cross(rot_z_vec);
                // X-basis
                Eigen::Vector3d rot_x_vec = rot_y_vec.cross(rot_z_vec);

                Eigen::Matrix3d target_ss_rot_mat;
                target_ss_rot_mat <<
                    rot_x_vec[0], rot_y_vec[0], rot_z_vec[0],
                    rot_x_vec[1], rot_y_vec[1], rot_z_vec[1],
                    rot_x_vec[2], rot_y_vec[2], rot_z_vec[2];

                Eigen::Vector4d target_q = Rotation::dcm2quat(target_ss_rot_mat);

                Eigen::VectorXd optimal_cmg_gimbal_rate = this->ss_controller.compute_cmg_gimbal_rates(
                    target_q, this->ss_quaternion_vec, this->ss_w_vec, dt,
                    this->control_kp, this->control_ki, this->control_kd
                );
                double abs_max_gimbal_rate = optimal_cmg_gimbal_rate.array().abs().maxCoeff();
                //std::cout << abs_max_gimbal_rate << std::endl;
                //std::cout << optimal_cmg_gimbal_rate.transpose() << std::endl;
                if (this->ssd.max_cmg_gimbal_rate < abs_max_gimbal_rate) {
                    this->cmg_gimbal_rate = optimal_cmg_gimbal_rate / abs_max_gimbal_rate * this->ssd.max_cmg_gimbal_rate;
                }
                else {
                    this->cmg_gimbal_rate = optimal_cmg_gimbal_rate;
                }

                if (this->cmg_gimbal_rate.hasNaN()) {
                    this->cmg_gimbal_rate.setZero();
                }
            }
            else if (this->attitude_control_plan == 2) {
                // --- LVLH Control (auto) ---
                // Z-basis
                Eigen::Vector3d rot_z_vec = -old_ss_pos_vec.normalized();
                // Y-basis
                Eigen::Vector3d rot_y_vec = -old_ss_vel_vec.normalized().cross(rot_z_vec);
                // X-basis
                Eigen::Vector3d rot_x_vec = rot_y_vec.cross(rot_z_vec);

                ss_rot_mat <<
                    rot_x_vec[0], rot_y_vec[0], rot_z_vec[0],
                    rot_x_vec[1], rot_y_vec[1], rot_z_vec[1],
                    rot_x_vec[2], rot_y_vec[2], rot_z_vec[2];
                this->ss_quaternion_vec = Rotation::dcm2quat(ss_rot_mat);
            }

            // Frame transformer of SCI - ECI
            this->sci_eci_ft.update_origin_vec(this->earth_pos_vec);
            // Frame transformer of ECI - BF
            this->eci_ssbf_ft.update_basis_mat(ss_rot_mat);
            this->eci_ssbf_ft.update_origin_vec(this->ss_position_eci);

            // -------- Power --------

            // ---- Check that space Station is in shade of the Earth or not ----
            Eigen::Vector3d old_ss_pos_sci_vec = this->sci_eci_ft.get_global_pos(old_ss_pos_vec);
            Eigen::Vector3d old_normalized_ss_pos_sci_vec = old_ss_pos_sci_vec.normalized();
            double d1 = old_normalized_ss_pos_sci_vec.dot(-old_earth_pos_vec);
            double d2 = std::pow(old_earth_pos_vec.norm(), 2) - OrbitLib::EARTH_RADIUS * OrbitLib::EARTH_RADIUS;
            double discriminant = d1 * d1 - d2;
            this->ss_in_sunlight = true;
            if (0 < discriminant) {
                double t_posi = -d1 + std::sqrt(discriminant);
                if (t_posi < old_ss_pos_sci_vec.norm()) {
                    this->ss_in_sunlight = false;
                }
            }

            // ---- Update Solar array direction by SARJ angle ----
            this->ss_sap_normal_vec = this->calc_ss_sap_normal_vec();

            //std::cout << old_sun_direction_vec.transpose() << std::endl;
            double cos_theta = old_sap_normal_vec.dot(old_sun_direction_vec.normalized());

            if (this->ss_in_sunlight) {
                this->cur_generated_power = (cos_theta > 0) ? this->max_generated_power * cos_theta : 0.0;
            }
            else {
                this->cur_generated_power = 0.0;
            }

            this->cur_battery_amount -= this->ss_power_consumption * dt;
            this->cur_battery_amount += this->cur_generated_power * dt;
            if (this->cur_battery_amount < 0) {
                this->cur_battery_amount = 0.0;
            }
            else if (this->full_battery_amount < this->cur_battery_amount) {
                this->cur_battery_amount = this->full_battery_amount;
            }

            return;
        }

        // ================ Getter ================

        inline double get_time() const {
            return this->t;
        }

        inline const Eigen::Vector3d& get_ss_position_eci() const {
            return this->ss_position_eci;
        }

        inline const Eigen::Vector3d& get_ss_velocity_eci() const {
            return this->ss_velocity_eci;
        }

        inline const Eigen::Vector3d& get_ss_acceleration_eci() const {
            return this->ss_acceleration_eci;
        }

        inline const Eigen::Vector4d& get_ss_quaternion_eci() const {
            return this->ss_quaternion_vec;
        }

        inline const Eigen::Matrix3d& get_ss_rotation_matrix_eci() const {
            // Get rotation matrix that expresses attitude of the space station.
            return this->eci_ssbf_ft.get_local_frame_basis_mat();
        }

        inline const Eigen::Vector3d& get_ss_w() const {
            return this->ss_w_vec;
        }

        inline double get_sarj_angle() const {
            return this->sarj_angle;
        }
        
        inline double get_current_generated_power() const {
            return this->cur_generated_power;
        }
        
        // ==== Get battery amount [Wh] ====
        inline double get_current_battery_amount() const {
            return this->cur_battery_amount;
        }

        inline double get_battery_soc() const {
            return this->cur_battery_amount / this->full_battery_amount;
        }

        Eigen::Vector3d get_sun_pos_at_ss_vec() const {
            // -------- Calculate sun position vector at BF --------
            Eigen::Vector3d sun_pos_eci_vec = this->sci_eci_ft.get_local_pos(Eigen::Vector3d::Zero());
            // Position vector of the Sun seen from the origin of BF-frame.
            // Each axis direction is same as ECI.
            Eigen::Vector3d sun_pos_ss_center_vec = sun_pos_eci_vec - this->eci_ssbf_ft.get_local_frame_origin_vec();
            // Normalization (if don't normalize, value becomes bad when rotation matrix is multiplied.)
            Eigen::Vector3d sun_pos_ss_center_normal_vec = sun_pos_ss_center_vec / sun_pos_ss_center_vec.norm();
            Eigen::Vector3d sun_pos_ssbf_vec = this->eci_ssbf_ft.get_inv_local_frame_basis_mat() * sun_pos_ss_center_normal_vec;
            return sun_pos_ssbf_vec;
        }

        inline bool is_ss_in_sunlight() const {
            return this->ss_in_sunlight;
        }

        // ================ Setter ================

        void set_sarj_angle(double sarj_angle) {
            this->sarj_angle = sarj_angle;
        }

        void set_thruster_command(const Eigen::VectorXd& thruster_command) {
            this->thruster_firing_duty = thruster_command;
        }

    };
    
}

#endif
