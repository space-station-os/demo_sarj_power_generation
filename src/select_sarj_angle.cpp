
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "geometry_msgs/msg/vector3.hpp"

#include "eigen_util.hpp"
#include "rotation.hpp"
#include "frame_transformer.hpp"
#include "topic_name.hpp"
#include "space_station_design.hpp"


constexpr double PI = 3.141592653589793;

inline constexpr double deg2rad(double deg) { return deg / 180.0 * PI; }
inline constexpr double rad2deg(double rad) { return rad / PI * 180.0; }


class SelectSarjAngle : public rclcpp::Node
{
private:
    Eigen::Vector3d sarj_rotation_axis_vec;

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;


public:
    SelectSarjAngle() : rclcpp::Node("select_sarj_angle"){
        this->subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            TopicName::sun_direction_ssbf, 10,
            std::bind(&SelectSarjAngle::topic_callback, this, std::placeholders::_1)
            );

        this->publisher_ = this->create_publisher<std_msgs::msg::Float64>(TopicName::target_sarj_angle_value, 10);

        // Normalize
        this->sarj_rotation_axis_vec = SpaceStationDesign::SARJ_ROTATION_AXIS.normalized();
    }


private:
    void topic_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        // ---- Calculate optimal SARJ angle ----
        Eigen::Vector3d sun_direction_ssbf_vec(msg->x, msg->y, msg->z);
        sun_direction_ssbf_vec = sun_direction_ssbf_vec.normalized();
        double dot = this->sarj_rotation_axis_vec.dot(sun_direction_ssbf_vec);
        Eigen::Vector3d projected_sun_dir_vec = sun_direction_ssbf_vec - dot * this->sarj_rotation_axis_vec;
        Eigen::Vector3d projected_sun_dir_normalized_vec = projected_sun_dir_vec.normalized();
        Eigen::Vector3d cross_vec = SpaceStationDesign::SAP_BASE_NORMAL_VEC.cross(projected_sun_dir_normalized_vec);
        double cos_theta = SpaceStationDesign::SAP_BASE_NORMAL_VEC.dot(projected_sun_dir_normalized_vec);
        double theta = std::acos(cos_theta);

        if (cross_vec.dot(this->sarj_rotation_axis_vec) < 0){
            theta = -theta;
        }

        // --- for debug ---
        // Eigen::Matrix3d sarj_rot_mat = Rotation::rodrigues_rotation_matrix(this->sarj_rotation_axis_vec, theta);
        // Eigen::Vector3d ss_sap_normal_vec = sarj_rot_mat * SpaceStationDesign::SAP_BASE_NORMAL_VEC;

        RCLCPP_INFO(this->get_logger(), "Optimal SARJ angle=%4.2f[deg]", rad2deg(theta));

        // --- Publish ---
        auto output_msg = std_msgs::msg::Float64();
        output_msg.data = theta;
        this->publisher_->publish(output_msg);
    }
};


int main(int argc, char* argv[]){
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SelectSarjAngle>());
    rclcpp::shutdown();
    
    return 1;
}
