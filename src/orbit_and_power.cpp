
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
#include "std_msgs/msg/float64_multi_array.hpp"

#include "eigen_util.hpp"
#include "topic_name.hpp"
#include "space_station_design.hpp"
#include "space_station_simulator.hpp"


class SpaceStationSimulationNode : public rclcpp::Node
{

private:
    // -------- variables --------

    SpaceStationSimulator::SpaceStationSimulator sss;

    // Time of start simulation
    std::chrono::system_clock::time_point simu_start_time;

    double simu_speed_rate ;

    // ---- Publishers ----
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> int32_publisher_map;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> float64_publisher_map;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr> vector3_publisher_map;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr> quaternion_publisher_map;

    // ---- Subscriptions ----
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_sarj_angle;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_dury_subscription;

    rclcpp::TimerBase::SharedPtr update_dynamics_timer;
    rclcpp::TimerBase::SharedPtr publish_value_timer;

    // ================ Function about publish ================

    void add_int32_publisher(std::string topic_name){
        this->int32_publisher_map[topic_name] = this->create_publisher<std_msgs::msg::Int32>(topic_name, 10);
    }

    void add_float64_publisher(std::string topic_name){
        this->float64_publisher_map[topic_name] = this->create_publisher<std_msgs::msg::Float64>(topic_name, 10);
    }
    
    void add_vector3_publisher(std::string topic_name){
        this->vector3_publisher_map[topic_name] = this->create_publisher<geometry_msgs::msg::Vector3>(topic_name, 10);
    }
    
    void add_quaternion_publisher(std::string topic_name){
        this->quaternion_publisher_map[topic_name] = this->create_publisher<geometry_msgs::msg::Quaternion>(topic_name, 10);
    }
    
    void publish_float64(std::string topic_name, double v){
        auto message_instance = std_msgs::msg::Float64();
        message_instance.data = v;
        this->float64_publisher_map[topic_name]->publish(message_instance);
    }

    void publish_int32(std::string topic_name, int32_t v){
        auto message_instance = std_msgs::msg::Int32();
        message_instance.data = v;
        this->int32_publisher_map[topic_name]->publish(message_instance);
    }

    void publish_vector3(std::string topic_name, const Eigen::Vector3d& v){
        auto message_instance = geometry_msgs::msg::Vector3();
        message_instance.x = v[0];
        message_instance.y = v[1];
        message_instance.z = v[2];
        this->vector3_publisher_map[topic_name]->publish(message_instance);
    }

    void publish_quaternion(std::string topic_name, const Eigen::Vector4d& v){
        auto message_instance = geometry_msgs::msg::Quaternion();
        message_instance.x = v[0];
        message_instance.y = v[1];
        message_instance.z = v[2];
        message_instance.w = v[3];
        this->quaternion_publisher_map[topic_name]->publish(message_instance);
    }

    // ================ Function about subscription ================
    

    // 
    void update_dynamics_callback(){
        
        // ---- Get current time ----
        std::chrono::system_clock::time_point simu_cur_time = std::chrono::system_clock::now();
        double elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(simu_cur_time - this->simu_start_time).count();
        double simu_time = elapsed_time_ms / 1000 * this->simu_speed_rate;

        this->sss.update(simu_time);
    }

    void publish_value_callback() {
        // -------- Publish --------

        // ---- convert time ----
        double simu_time = this->sss.get_time();
        int32_t t_int32 = int32_t(simu_time);
        int32_t t_millisec = int32_t(simu_time - double(t_int32)) * 1000;
        int32_t t_sec = t_int32 % 60;
        int32_t t_min = (t_int32 / 60) % 60;
        int32_t t_hour = (t_int32 / 60 / 60) % 24;
        // int32_t t_day = (t_int32 / 60 / 60 / 24) % 24;

        auto current_generated_power = this->sss.get_current_generated_power();
        auto current_battery_amount = this->sss.get_current_battery_amount();
        RCLCPP_INFO(this->get_logger(), "t=%02d:%02d:%02d.%03d, generated_power=%3.2f[W], battery_amount=%3.2f[kWh]", t_hour, t_min, t_sec, t_millisec, current_generated_power, current_battery_amount/1000);

        // ---- Publish ----
        this->publish_float64(TopicName::simu_time, simu_time);
        this->publish_float64(TopicName::generated_power, this->sss.get_current_generated_power());
        this->publish_float64(TopicName::battery_level, this->sss.get_current_battery_amount());
        this->publish_float64(TopicName::sarj_angle, this->sss.get_sarj_angle());

        this->publish_int32(TopicName::ss_in_sunlight, this->sss.is_ss_in_sunlight());

        this->publish_quaternion(TopicName::ss_attitude, this->sss.get_ss_quaternion_eci());

        this->publish_vector3(TopicName::ss_position_eci, this->sss.get_ss_position_eci());
        this->publish_vector3(TopicName::ss_velocity_eci, this->sss.get_ss_velocity_eci());
        this->publish_vector3(TopicName::ss_acceleration_eci, this->sss.get_ss_acceleration_eci());
        this->publish_vector3(TopicName::sun_direction_ssbf, this->sss.get_sun_pos_at_ss_vec());

    }

    void sub_sarj_angle_callback(const std_msgs::msg::Float64::SharedPtr msg){
        double target_sarj_agnle = msg->data;
        RCLCPP_INFO(this->get_logger(), "Subscribe: target_sarj_agnle=%4.2f[deg]", Math::rad2deg(target_sarj_agnle));
        this->sss.set_sarj_angle(target_sarj_agnle);
    }
    
    void sub_thruster_duty_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        size_t vector_size = msg->data.size();
        // RCLCPP_INFO(this->get_logger(), "Received vector of size: %zu", vector_size);
        
        Eigen::VectorXd thruster_duty(vector_size);
        for (size_t i = 0; i < vector_size; ++i) {
            thruster_duty[i] = msg->data[i];
        }
        this->sss.set_thruster_command(thruster_duty);
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

    SpaceStationSimulationNode() : Node("orbit_and_power")
    {
        // -------- Declare parameters and set default value --------
        int32_t attitude_control_plan = this->declare_and_get_int32_parameter("attitude_control_plan", 2);

        Eigen::Vector3d ss_init_euler_vec = this->declare_and_get_parameter("ss_init_euler_angle", {0.0, 0.0, 0.0});
        Eigen::Vector3d ss_init_w_vec = this->declare_and_get_parameter("ss_init_w_vec", {0.0, 0.0, 0.0});

        double simu_timestep = this->declare_and_get_double_parameter("simu_timestep", 20.0);
        double publish_period = this->declare_and_get_double_parameter("publish_period", 60.0);
        double simu_speed_rate = this->declare_and_get_double_parameter("speed_rate", 400.0);

        std::string line2 = "2 60182  97.9211 215.3545 0001598  99.1275 261.0117 14.79484184 44375";

        // -------- Control --------
        // attitude_control_plan
        // - 0: No control
        // - 1: LVLH
    
        //string line1 = "1 25544U 98067A   22095.91869325  .00012930  00000 - 0  23502 - 3 0  9991";
        //string line2 = "2 25544  51.6452 334.5328 0004408 351.0413  99.6998 15.49890618333972";
    
        Eigen::Vector3d ss_position_eci;
        Eigen::Vector3d ss_velocity_eci;
    
        OrbitLib::convert_tle_to_eci(line2, ss_position_eci, ss_velocity_eci);
    
        this->sss = SpaceStationSimulator::SpaceStationSimulator();
        // sss.activate_propagation_j2(OrbitLib::J2);
        // this->sss.activate_propagation_air_drag(2.2, 1000, 10);
        this->sss.initialize(ss_position_eci, ss_velocity_eci, ss_init_euler_vec, ss_init_w_vec, attitude_control_plan);

        // ---- Other -----
        this->simu_start_time = std::chrono::system_clock::now();

        this->simu_speed_rate = simu_speed_rate;

        // ---- Callback functions ----

        // Update Dynamics
        int32_t update_dynamics_period_ms = int32_t(simu_timestep / this->simu_speed_rate * 1000);
        this->update_dynamics_timer = this->create_wall_timer(
            std::chrono::milliseconds(update_dynamics_period_ms),
            std::bind(&SpaceStationSimulationNode::update_dynamics_callback, this)
        );

        // Publish period [milli second]
        int32_t publish_period_ms = int32_t(publish_period / this->simu_speed_rate * 1000);
        this->publish_value_timer = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms),
            std::bind(&SpaceStationSimulationNode::publish_value_callback, this)
        );

        // ---- Publishers ----
        this->add_float64_publisher(TopicName::simu_time);
        this->add_float64_publisher(TopicName::generated_power);
        this->add_float64_publisher(TopicName::battery_level);
        this->add_float64_publisher(TopicName::sarj_angle);

        this->add_int32_publisher(TopicName::ss_in_sunlight);

        this->add_quaternion_publisher(TopicName::ss_attitude);

        this->add_vector3_publisher(TopicName::ss_position_eci);
        this->add_vector3_publisher(TopicName::ss_velocity_eci);
        this->add_vector3_publisher(TopicName::ss_acceleration_eci);
        this->add_vector3_publisher(TopicName::sun_direction_ssbf);

        // ---- Subscriptions ----
        this->subscription_sarj_angle = this->create_subscription<std_msgs::msg::Float64>(
            TopicName::target_sarj_angle_value, 10,
            std::bind(&SpaceStationSimulationNode::sub_sarj_angle_callback, this, std::placeholders::_1)
        );

        this->thruster_dury_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            TopicName::thruster_duty, 10,
            std::bind(&SpaceStationSimulationNode::sub_thruster_duty_callback, this, std::placeholders::_1)
        );

        // -------- Output log for check --------
        std::cout << "Dynamics timestep (as simulation time): " << simu_timestep << "[s]" << std::endl;
        std::cout << "Publish period (as simulation time): " << publish_period << "[s]" << std::endl;
        std::cout << "Simulation speed rate: " << this->simu_speed_rate << std::setprecision(6) << "x" << std::endl;
        std::cout << "Dynamics timestep (as real time): " << double(update_dynamics_period_ms)/1000 << std::setprecision(6) << "[s]" << std::endl;
        std::cout << "Publish period (as real time): " << double(publish_period_ms)/1000 << std::setprecision(6) << "[s]" << std::endl;

    }
};


int main(int argc, char* argv[]){
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpaceStationSimulationNode>());
    rclcpp::shutdown();
    
    return 1;
}

