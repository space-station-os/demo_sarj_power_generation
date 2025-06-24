
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/vector3.hpp"
#include <std_msgs/msg/float64.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

#include "eigen_util.hpp"
#include "rotation.hpp"
#include "frame_transformer.hpp"
#include "topic_name.hpp"
#include "space_station_design.hpp"


constexpr double PI = 3.141592653589793;

inline constexpr double deg2rad(double deg) { return deg / 180.0 * PI; }
inline constexpr double rad2deg(double rad) { return rad / PI * 180.0; }


class SelectThrusterCommand : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ss_pos_pub;

    space_station_design::SpaceStationDesign ss_design;


public:
    SelectThrusterCommand() : rclcpp::Node("select_thruster_command"){
        this->sub = this->create_subscription<geometry_msgs::msg::Vector3>(
            TopicName::ss_position_eci, 10,
            std::bind(&SelectThrusterCommand::topic_callback, this, std::placeholders::_1)
            );

        this->ss_pos_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(TopicName::thruster_duty, 10);

        this->ss_design = space_station_design::SpaceStationDesign();
    }


private:
    void topic_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        Eigen::Vector3d ss_pos(msg->x, msg->y, msg->z);
        RCLCPP_INFO(this->get_logger(), "Space Station Position @ECI=[%f, %f, %f]", ss_pos.x(), ss_pos.y(), ss_pos.z());

        // --- Publish ---
        std_msgs::msg::Float64MultiArray output_msg;
        output_msg.data = {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        this->ss_pos_pub->publish(output_msg);
    }
};


int main(int argc, char* argv[]){
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SelectThrusterCommand>());
    rclcpp::shutdown();
    
    return 1;
}
