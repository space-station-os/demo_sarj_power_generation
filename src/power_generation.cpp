
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <iomanip>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "geometry_msgs/msg/vector3.hpp"

#include "eigen_util.hpp"
#include "rotation.hpp"
#include "frame_transformer.hpp"
#include "topic_name.hpp"
#include "space_station_design.hpp"


constexpr double PI = 3.141592653589793;

// The Earth radius [m]
constexpr double EARTH_RADIUS = 6378.14 * 1e3;
// [m^3 s^-2]
constexpr double G_ME = 3.986004418e14;


inline constexpr double deg2rad(double deg) { return deg / 180.0 * PI; }
inline constexpr double rad2deg(double rad) { return rad / PI * 180.0; }


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


Eigen::Vector4d update_quaternion(const Eigen::Vector4d& q_vec, const Eigen::Vector3d& w_vec, double dt) {
    // Runge–Kutta method
    auto dt_h = dt / 2;

    auto k1_vec = quaternion_diff_equ(q_vec, w_vec);
    auto k2_vec = quaternion_diff_equ(q_vec + k1_vec * dt_h, w_vec);
    auto k3_vec = quaternion_diff_equ(q_vec + k2_vec * dt_h, w_vec);
    auto k4_vec = quaternion_diff_equ(q_vec + k3_vec * dt, w_vec);
    auto next_q_vec = q_vec + (k1_vec + k2_vec + k3_vec + k4_vec) / 6 * dt;

    return next_q_vec;
}


class SpaceStationPhysics : public rclcpp::Node
{

private:
    // -------- variables --------

    double t;

    // ---- Parameters of the Earth ----
    // Initial phase [rad]
    double earth_init_phase;
    // Angular Velocity of revolution [rad/s]
    double earth_revolution_w;

    double sun_earth_distance;

    // ---- Parameters of the Space Station ----  
    // initial phase [rad]
    double ss_init_phase;
    // Elevation + earth radius
    double ss_motion_radius;
    // Normal vector of SAP @BF
    Eigen::Vector3d ss_sap_normal_vec;
    // Anlugar velocity of revolution [rad/s] 
    double ss_revolution_w;

    FrameTransformer ss_plane_inertia_ft;

    // ---- Parameters of the Earth ----  

    Eigen::Vector3d earth_pos_vec;
    Eigen::Vector3d ss_pos_vec;

    // Global:SCI, Local:ECI
    FrameTransformer sci_eci_ft;
    // Global:ECI, Local:SSBF
    FrameTransformer eci_ssbf_ft;

    // Quaternion of Space Station
    Eigen::Vector4d ss_quaternion_vec;
    // Angular velocity of SS
    Eigen::Vector3d ss_w_vec;

    // ---- Parameters of SAP ----
    // Maximum power generation [W]
    double max_generated_power;
    // Full amount of battery [Wh]
    double full_battery_amount;
    // Maximum power generation [W]
    double cur_generated_power;
    // Current amount of battery [Wh]
    double cur_battery_amount;
    // Consumed power by other subsystem [W]
    double ss_power_consumption;

    // Space Station is in shade of the Earth or not
    bool ss_in_sunlight;

    // SARJ rotation axis & SAP normal vector (normalized)
    Eigen::Vector3d sarj_rotation_axis_vec = SpaceStationDesign::SARJ_ROTATION_AXIS;
    Eigen::Vector3d ss_sap_basic_normal_vec = SpaceStationDesign::SAP_BASE_NORMAL_VEC;
    // Solar array rotary joint angle [rad]
    double sarj_angle;

    // ---- Control ----
    int32_t attitude_control_plan;

    // Time of start simulation
    std::chrono::system_clock::time_point simu_start_time;

    double simu_speed_rate ;

    // ---- Publishers ----
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_t;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_generated_power;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_soc;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_sarj_angle;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_ss_in_sunlight;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr publisher_ss_attitude;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_sun_direction_ssbf;

    // ---- Subscriptions ----
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_sarj_angle;

    rclcpp::TimerBase::SharedPtr update_dynamics_timer;
    rclcpp::TimerBase::SharedPtr publish_value_timer;

    Eigen::Vector3d calc_earth_pos_vec(double t) const {
        // -------- Calculate earth position @SCI at t --------
        double cur_phase = this->earth_init_phase + this->earth_revolution_w * t;
        Eigen::Vector3d earth_pos_vec(
            std::cos(cur_phase) * this->sun_earth_distance,
            std::sin(cur_phase) * this->sun_earth_distance,
            0.0
        );
        return earth_pos_vec;
    }

    Eigen::Vector3d calc_ss_pos_vec(double t) const {
        // -------- Calculate SS position @ECI at t --------
        // Current SS phase at ECI
        double cur_phase = this->earth_init_phase + this->ss_revolution_w * t;

        // Position at orbital plane
        Eigen::Vector3d ss_pos_plane_vec(
            std::cos(cur_phase) * this->ss_motion_radius,
            std::sin(cur_phase) * this->ss_motion_radius,
            0.0
        );

        // 
        return this->ss_plane_inertia_ft.get_global_pos(ss_pos_plane_vec);
    }

    Eigen::Vector3d calc_ss_vel_vec(double t) const {
        // -------- Calculate SS velocity vector @ECI at t --------
        // Current SS phase at ECI
        double cur_phase = this->earth_init_phase + this->ss_revolution_w * t;

        Eigen::Vector3d ss_vel_plane_vec(
            -std::sin(cur_phase) * this->ss_motion_radius,
            std::cos(cur_phase) * this->ss_motion_radius,
            0.0
        );  
        return this->ss_plane_inertia_ft.get_global_pos(ss_vel_plane_vec);
    }

    void initialize(
        double ss_altitude, double ss_raan, double ss_inclination,
        int32_t attitude_control_plan,
        const Eigen::Vector3d& ss_init_euler_vec,
        const Eigen::Vector3d& ss_init_w_vec,
        double simu_timestep, double pulish_period, double speed_rate
    )
    {
        this->sun_earth_distance = 149600000 * 1e3;

        // ---- Parameters of the Earth ----
        this->earth_init_phase = 0.0;
        this->earth_revolution_w = 2.0 * PI / (365.25 * 24 * 60 * 60);

        // ---- Parameters of the Space Station ----
        this->ss_init_phase = 0.0;
        this->ss_motion_radius = EARTH_RADIUS + ss_altitude;
        // Velocity of the SS [m/s]
        double ss_v = std::sqrt(G_ME / this->ss_motion_radius);
        double ss_revolution_period = 2.0 * PI * this->ss_motion_radius / ss_v;
        this->ss_revolution_w = 2.0 * PI / ss_revolution_period;

        Eigen::Matrix3d ss_plane_inertia_rot_mat = Rotation::euler2dcm(
            Eigen::Vector3d(ss_inclination, 0.0, ss_raan)
        );
        this->ss_plane_inertia_ft = FrameTransformer(
            ss_plane_inertia_rot_mat.transpose(), Eigen::Vector3d::Zero()
        );

        // -------- Control --------
        // attitude_control_plan
        // - 0: No control
        // - 1: LVLH
        this->attitude_control_plan = attitude_control_plan;

        // -------- Initialize time-varying parameters --------
        this->t = 0.0;

        this->earth_pos_vec = this->calc_earth_pos_vec(this->t);
        this->ss_pos_vec = this->calc_ss_pos_vec(this->t);

        this->sci_eci_ft = FrameTransformer(
            Eigen::Matrix3d::Identity(), this->earth_pos_vec
        );
        this->eci_ssbf_ft = FrameTransformer(
            Eigen::Matrix3d::Identity(), this->ss_pos_vec
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

        // ---- Other -----
        this->simu_start_time = std::chrono::system_clock::now();

        this->simu_speed_rate = speed_rate;

        // ---- Callback functions ----

        // Update Dynamics
        int32_t update_dynamics_period_ms = int32_t(simu_timestep / this->simu_speed_rate * 1000);
        this->update_dynamics_timer = this->create_wall_timer(
            std::chrono::milliseconds(update_dynamics_period_ms),
            std::bind(&SpaceStationPhysics::update_dynamics_callback, this)
        );

        // Publish period [milli second]
        int32_t publish_period_ms = int32_t(pulish_period / this->simu_speed_rate * 1000);
        this->publish_value_timer = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms),
            std::bind(&SpaceStationPhysics::publish_value_callback, this)
        );

        // ---- Publishers ----
        this->publisher_t = this->create_publisher<std_msgs::msg::Float64>(TopicName::simu_time, 10);
        this->publisher_generated_power = this->create_publisher<std_msgs::msg::Float64>(TopicName::generated_power, 10);
        this->publisher_soc = this->create_publisher<std_msgs::msg::Float64>(TopicName::soc, 10);
        this->publisher_sarj_angle = this->create_publisher<std_msgs::msg::Float64>(TopicName::sarj_angle, 10);
        this->publisher_ss_in_sunlight = this->create_publisher<std_msgs::msg::Int32>(TopicName::ss_in_sunlight, 10);
        this->publisher_ss_attitude = this->create_publisher<geometry_msgs::msg::Quaternion>(TopicName::ss_attitude, 10);
        this->publisher_sun_direction_ssbf = this->create_publisher<geometry_msgs::msg::Vector3>(TopicName::sun_direction_ssbf, 10);

        // ---- Subscriptions ----
        this->subscription_sarj_angle = this->create_subscription<std_msgs::msg::Float64>(
            TopicName::target_sarj_angle_value, 10,
            std::bind(&SpaceStationPhysics::sub_sarj_angle_callback, this, std::placeholders::_1)
            );

        // -------- Output log for check --------
        std::cout << "Dynamics timestep (as simulation time): " << simu_timestep << "[s]" << std::endl;
        std::cout << "Publish period (as simulation time): " << pulish_period << "[s]" << std::endl;
        std::cout << "Simulation speed rate: " << this->simu_speed_rate << std::setprecision(6) << "x" << std::endl;
        std::cout << "Dynamics timestep (as real time): " << double(update_dynamics_period_ms)/1000 << std::setprecision(6) << "[s]" << std::endl;
        std::cout << "Publish period (as real time): " << double(publish_period_ms)/1000 << std::setprecision(6) << "[s]" << std::endl;
    }
    
    void update(double new_t) {

        // Simulation time
        double dt = new_t - this->t;
        this->t = new_t;

        // -------- Store old values --------
        auto old_earth_pos_vec = this->earth_pos_vec;
        auto old_ss_pos_vec = this->ss_pos_vec;
        Eigen::Vector3d old_sap_normal_vec = this->ss_sap_normal_vec;
        Eigen::Vector3d old_sun_direction_vec = this->get_sun_pos_at_ss_vec();

        // -------- Dynamics --------

        // ---- Position ----
        this->earth_pos_vec = this->calc_earth_pos_vec(this->t);
        this->ss_pos_vec = this->calc_ss_pos_vec(this->t);

        // ---- Attitude ----
        Eigen::Matrix3d ss_rot_mat;

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
            Eigen::Vector3d rot_x_vec = this->calc_ss_vel_vec(this->t).normalized();
            // Z-basis
            Eigen::Vector3d rot_z_vec = -old_ss_pos_vec.normalized();
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
        this->eci_ssbf_ft.update_origin_vec(this->ss_pos_vec);

        // -------- Power --------

        // ---- Check that space Station is in shade of the Earth or not ----
        Eigen::Vector3d old_ss_pos_sci_vec = this->sci_eci_ft.get_global_pos(old_ss_pos_vec);
        Eigen::Vector3d old_normalized_ss_pos_sci_vec = old_ss_pos_sci_vec.normalized();
        double d1 = old_normalized_ss_pos_sci_vec.dot(-old_earth_pos_vec);
        double d2 = std::pow(old_earth_pos_vec.norm(), 2) - EARTH_RADIUS*EARTH_RADIUS;
        double discriminant = d1*d1 - d2;
        this->ss_in_sunlight = true;
        if (0 < discriminant){
            double t_posi = -d1 + std::sqrt(discriminant);
            if (t_posi < old_ss_pos_sci_vec.norm()){
                this->ss_in_sunlight = false;
            }
        }

        // ---- Update Solar array direction by SARJ angle ----
        this->ss_sap_normal_vec = this->calc_ss_sap_normal_vec();

        double cos_theta = old_sap_normal_vec.dot(old_sun_direction_vec.normalized());

        if (this->ss_in_sunlight){
            this->cur_generated_power = (cos_theta > 0) ? this->max_generated_power * cos_theta : 0.0;
        }
        else{
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

    // 
    void update_dynamics_callback(){
        
        // ---- Get current time ----
        std::chrono::system_clock::time_point simu_cur_time = std::chrono::system_clock::now();
        double elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(simu_cur_time - this->simu_start_time).count();
        double simu_time = elapsed_time_ms / 1000 * this->simu_speed_rate;

        this->update(simu_time);
    }

    void publish_value_callback() {
        // -------- Publish --------

        // ---- Convert ---- 
        auto message_t = std_msgs::msg::Float64();
        auto message_cur_generated_power = std_msgs::msg::Float64();
        auto message_soc = std_msgs::msg::Float64();
        auto message_sarj_angle = std_msgs::msg::Float64();
        auto message_ss_in_sunlight = std_msgs::msg::Int32();
        auto message_ss_attitude = geometry_msgs::msg::Quaternion();
        auto message_sun_direction_ssbf = geometry_msgs::msg::Vector3();

        message_t.data = this->t;
        message_cur_generated_power.data = this->cur_generated_power;
        message_soc.data = this->cur_battery_amount;
        message_sarj_angle.data = this->sarj_angle;
        message_ss_in_sunlight.data = this->ss_in_sunlight;
        message_ss_attitude.x = this->ss_quaternion_vec[0];
        message_ss_attitude.y = this->ss_quaternion_vec[1];
        message_ss_attitude.z = this->ss_quaternion_vec[2];
        message_ss_attitude.w = this->ss_quaternion_vec[3];

        Eigen::Vector3d sun_direction_ssbf_vec = this->get_sun_pos_at_ss_vec();
        message_sun_direction_ssbf.x = sun_direction_ssbf_vec[0];
        message_sun_direction_ssbf.y = sun_direction_ssbf_vec[1];
        message_sun_direction_ssbf.z = sun_direction_ssbf_vec[2];

        // ---- convert time ----
        int32_t t_int32 = int32_t(this->t);
        int32_t t_millisec = int32_t(this->t - double(t_int32)) * 1000;
        int32_t t_sec = t_int32 % 60;
        int32_t t_min = (t_int32 / 60) % 60;
        int32_t t_hour = (t_int32 / 60 / 60) % 24;
        //int32_t t_day = (t_int32 / 60 / 60 / 24) % 24;

        RCLCPP_INFO(this->get_logger(), "t=%02d:%02d:%02d.%03d, generated_power=%3.2f[W], battery_amount=%3.2f[kWh]", t_hour, t_min, t_sec, t_millisec, this->cur_generated_power, this->cur_battery_amount/1000);

        // ---- Publish ----
        this->publisher_t->publish(message_t);
        this->publisher_generated_power->publish(message_cur_generated_power);
        this->publisher_soc->publish(message_soc);
        this->publisher_sarj_angle->publish(message_sarj_angle);
        this->publisher_ss_in_sunlight->publish(message_ss_in_sunlight);
        this->publisher_ss_attitude->publish(message_ss_attitude);
        this->publisher_sun_direction_ssbf->publish(message_sun_direction_ssbf);
    }

    void sub_sarj_angle_callback(const std_msgs::msg::Float64::SharedPtr msg){
        double target_sarj_agnle = msg->data;
        RCLCPP_INFO(this->get_logger(), "Subscribe: target_sarj_agnle=%4.2f[deg]", rad2deg(target_sarj_agnle));
        this->set_sarj_angle(target_sarj_agnle);
    }

    Eigen::Vector3d calc_ss_sap_normal_vec() const {
        // -------- Calculate normal vector of SAP in SSBF --------
        Eigen::Matrix3d sarj_rot_mat = Rotation::rodrigues_rotation_matrix(this->sarj_rotation_axis_vec, this->sarj_angle);
        Eigen::Vector3d ss_sap_normal_vec = sarj_rot_mat * this->ss_sap_basic_normal_vec;
        return ss_sap_normal_vec;
    }

    // -------- For Hardware Model --------

    inline double get_time() const {
        return this->t;
    }

    void set_sarj_angle(double sarj_angle) {
        this->sarj_angle = sarj_angle;
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

    double get_battery_soc() const {
        return this->cur_battery_amount / this->full_battery_amount;
    }

    // -------- Declare and get parameter for each type--------

    double declare_and_get_double_parameter(std::string param_name, double default_val){
    this->declare_parameter<double>(param_name, default_val);
    return this->get_parameter(param_name).as_double();
    }

    int32_t declare_and_get_int32_parameter(std::string param_name, int32_t default_val){
        this->declare_parameter<int32_t>(param_name, default_val);
        return static_cast<int32_t>(this->get_parameter(param_name).as_int());
    }

    Eigen::Vector3d declare_and_get_parameter(std::string param_name, std::vector<double> default_val){
        this->declare_parameter<std::vector<double>>(param_name, default_val);
        std::vector<double> temp_vec = this->get_parameter(param_name).as_double_array();
        return EigenUtil::from_std_vector(temp_vec);
    }

public:

    SpaceStationPhysics() : Node("demo_sarj_power_generation")
    {
        // -------- Declare parameters and set default value --------
        double ss_altitude = this->declare_and_get_double_parameter("ss_altitude", 400 * 1e3);
        double ss_raan = this->declare_and_get_double_parameter("ss_raan", deg2rad(10.0));
        double ss_inclination = this->declare_and_get_double_parameter("ss_inclination", deg2rad(20.0));

        int32_t attitude_control_plan = this->declare_and_get_int32_parameter("attitude_control_plan", 0);

        Eigen::Vector3d ss_init_euler_vec = this->declare_and_get_parameter("ss_init_euler_angle", {0.0, 0.0, 0.0});
        Eigen::Vector3d ss_init_w_vec = this->declare_and_get_parameter("ss_init_w_vec", {0.0, 0.02, 0.0});

        double simu_timestep = this->declare_and_get_double_parameter("simu_timestep", 20.0);
        double publish_period = this->declare_and_get_double_parameter("publish_period", 20.0);
        double speed_rate = this->declare_and_get_double_parameter("speed_rate", 200.0);

        this->initialize(
            ss_altitude, ss_raan, ss_inclination,
            attitude_control_plan,
            ss_init_euler_vec, ss_init_w_vec,
            simu_timestep, publish_period, speed_rate
        );
    }
};


int main(int argc, char* argv[]){
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpaceStationPhysics>());
    rclcpp::shutdown();
    
    return 1;
}

