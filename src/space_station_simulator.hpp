
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


namespace Math {
    constexpr double PI = 3.141592653589793;
    constexpr double TWO_PI = 2.0 * PI;

    inline constexpr double deg2rad(double deg) { return deg / 180.0 * Math::PI; }
    inline constexpr double rad2deg(double rad) { return rad / Math::PI * 180.0; }

}


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


    Eigen::Matrix3d rodrigues_rotation_matrix(Eigen::Vector3d axis_vec, double angle) {
        return Eigen::AngleAxis<double>(angle, axis_vec).matrix();
    }

};


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

        void update_position_velocity(
            const Eigen::Vector3d& old_pos_vec, const Eigen::Vector3d& old_vel_vec,
            double dt,
            Eigen::Vector3d& new_pos_vec, Eigen::Vector3d& new_vel_vec, Eigen::Vector3d& new_acc_vec
        ) {
            // ---- Runge–Kutta method ----
            double dt_h = 0.5 * dt;
            double dt_six = dt / 6.0;

            // k1
            Eigen::Vector3d k1_v = this->calc_acceleration_on_leo(old_pos_vec, old_vel_vec);
            Eigen::Vector3d k1_r = old_vel_vec;

            // k2
            Eigen::Vector3d pos_k2 = old_pos_vec + dt_h * k1_r;
            Eigen::Vector3d vel_k2 = old_vel_vec + dt_h * k1_v;
            Eigen::Vector3d k2_v = this->calc_acceleration_on_leo(pos_k2, vel_k2);
            Eigen::Vector3d k2_r = vel_k2;

            // k3
            Eigen::Vector3d pos_k3 = old_pos_vec + dt_h * k2_r;
            Eigen::Vector3d vel_k3 = old_vel_vec + dt_h * k2_v;
            Eigen::Vector3d k3_v = this->calc_acceleration_on_leo(pos_k3, vel_k3);
            Eigen::Vector3d k3_r = vel_k3;

            // k4
            Eigen::Vector3d pos_k4 = old_pos_vec + dt * k3_r;
            Eigen::Vector3d vel_k4 = old_vel_vec + dt * k3_v;
            Eigen::Vector3d k4_v = this->calc_acceleration_on_leo(pos_k4, vel_k4);
            Eigen::Vector3d k4_r = vel_k4;

            // 合成
            new_acc_vec = dt_six * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);
            new_vel_vec = old_vel_vec + new_acc_vec;
            new_pos_vec = old_pos_vec + dt_six * (k1_r + 2.0 * k2_r + 2.0 * k3_r + k4_r);

            return;
        }

    };
    

    Eigen::Vector4d update_quaternion(const Eigen::Vector4d& q_vec, const Eigen::Vector3d& w_vec, double dt) {
        // ---- Runge–Kutta method ----
        auto dt_h = dt / 2;

        auto k1_vec = quaternion_diff_equ(q_vec, w_vec);
        auto k2_vec = quaternion_diff_equ(q_vec + k1_vec * dt_h, w_vec);
        auto k3_vec = quaternion_diff_equ(q_vec + k2_vec * dt_h, w_vec);
        auto k4_vec = quaternion_diff_equ(q_vec + k3_vec * dt, w_vec);
        auto next_q_vec = q_vec + (k1_vec + k2_vec + k3_vec + k4_vec) / 6 * dt;

        return next_q_vec;
    }


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

        Eigen::Vector3d earth_pos_vec{ Eigen::Vector3d()};
        Eigen::Vector3d ss_position_eci{ Eigen::Vector3d() };
        Eigen::Vector3d ss_velocity_eci{ Eigen::Vector3d() };
        Eigen::Vector3d ss_acceleration_eci{ Eigen::Vector3d() };

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

        // SARJ rotation axis & SAP normal vector (normalized)
        Eigen::Vector3d sarj_rotation_axis_vec = SpaceStationDesign::SARJ_ROTATION_AXIS;
        Eigen::Vector3d ss_sap_basic_normal_vec = SpaceStationDesign::SAP_BASE_NORMAL_VEC;
        // Solar array rotary joint angle [rad]
        double sarj_angle{ 0.0 };

        // ---- Control ----
        int32_t attitude_control_plan{0};

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
            // - 1: LVLH
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

            Eigen::Matrix3d ss_attitude_rot_mat = Rotation::euler2dcm(ss_init_euler_vec);
            this->ss_quaternion_vec = Rotation::dcm2quat(ss_attitude_rot_mat);
            this->ss_w_vec = ss_init_w_vec;

            // ---- Power generation ----
            this->max_generated_power = 1000.0 / 6;
            this->full_battery_amount = 1000.0 * 2 * 60 * 60;
            this->cur_battery_amount = this->full_battery_amount;
            this->ss_power_consumption = 100.0;

            this->sarj_angle = 0.0;

            this->ss_sap_normal_vec = this->calc_ss_sap_normal_vec();
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
            // Eigen::Vector3d old_ss_acc_vec = this->ss_acc_vec;
            Eigen::Vector3d old_sap_normal_vec = this->ss_sap_normal_vec;
            Eigen::Vector3d old_sun_direction_vec = this->get_sun_pos_at_ss_vec();

            // -------- Dynamics --------

            // ---- Position & Velocity ----
            this->earth_pos_vec = this->calc_earth_pos_vec(this->t);

            // Space station @ECI-frame
            this->orbit_acc_model.update_position_velocity(old_ss_pos_vec, old_ss_vel_vec, dt, this->ss_position_eci, this->ss_velocity_eci, this->ss_acceleration_eci);

            // ---- Attitude ----
            Eigen::Matrix3d ss_rot_mat = Eigen::Matrix3d::Identity();

            if (this->attitude_control_plan == 0)
            {
                // --- No control ---
                // Update quaternion and normalize (if don't, norm becomes not 1)
                this->ss_quaternion_vec = update_quaternion(this->ss_quaternion_vec, this->ss_w_vec, dt).normalized();
                ss_rot_mat = Rotation::quat2dcm(this->ss_quaternion_vec);
            }
            else if (this->attitude_control_plan == 1)
            {
                // --- LVLH ---
                // X-basis is velocity vector
                Eigen::Vector3d rot_x_vec = this->ss_velocity_eci.normalized();
                // Z-basis
                Eigen::Vector3d rot_z_vec = -this->ss_position_eci.normalized();
                // Y-basis
                Eigen::Vector3d rot_y_vec = rot_x_vec.cross(rot_z_vec);

                ss_rot_mat <<
                    rot_x_vec[0], rot_y_vec[0], rot_z_vec[0],
                    rot_x_vec[1], rot_y_vec[1], rot_z_vec[1],
                    rot_x_vec[2], rot_y_vec[2], rot_z_vec[2];
            }

            // Frame transformer of SCI - ECI
            this->sci_eci_ft.update_origin_vec(this->earth_pos_vec);
            // Frame transformer of ECI - BF
            this->eci_ssbf_ft.update_basis_mat(ss_rot_mat.transpose());
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

    };
}

#endif
