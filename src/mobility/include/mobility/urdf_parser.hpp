#ifndef URDF_PARSER_HPP
#define URDF_PARSER_HPP

#include <string>
#include "tinyxml2.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

class UrdfParser : public rclcpp::Node {
public:
    UrdfParser() : Node("urdf_parser") {
        // Subscribe to the robot_description topic
        urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", 10,
            std::bind(&UrdfParser::urdfCallback, this, std::placeholders::_1));
    }

    bool isUrdfLoaded() const {
        return urdf_loaded_;
    }

    void getUrdfParams(double &mass, double &Iyy, double &length, double &total_mass, double &wheel_radius) {
        mass = pendulum_mass_;
        Iyy = pendulum_Iyy_;
        length = pendulum_length_;
        total_mass = pendulum_mass_;
        wheel_radius = 0.0;
    }

private:
    void urdfCallback(const std_msgs::msg::String::SharedPtr msg) {
        tinyxml2::XMLDocument urdf;
        urdf.Parse(msg->data.c_str());

        auto link = urdf.FirstChildElement("robot")->FirstChildElement("link");
        while (link) {
            const char* link_name = link->Attribute("name");
            if (link_name && std::string(link_name) == "pendulum") {
                auto inertial = link->FirstChildElement("inertial");
                if (inertial) {
                    auto mass_elem = inertial->FirstChildElement("mass");
                    auto inertia_elem = inertial->FirstChildElement("inertia");
                    auto origin_elem = inertial->FirstChildElement("origin");
                    auto wheel_elem = link->FirstChildElement("left_wheel");

                    pendulum_mass_ = mass_elem ? mass_elem->DoubleAttribute("value") : 0.0;
                    pendulum_Iyy_ = inertia_elem ? inertia_elem->DoubleAttribute("iyy") : 0.0;
                    pendulum_length_ = origin_elem ? origin_elem->DoubleAttribute("z") : 0.0;
                    total_mass_ = pendulum_mass_;
                    wheel_radius_ = wheel_elem ? wheel_elem->DoubleAttribute("radius") : 0.0;

                    urdf_loaded_ = true;

                    RCLCPP_INFO(this->get_logger(), "Loaded URDF - Mass: %f kg, Iyy: %f kg·m², Length: %f m",
                                pendulum_mass_, pendulum_Iyy_, pendulum_length_);
                    return;
                }
            }
            link = link->NextSiblingElement("link");
        }

        RCLCPP_WARN(this->get_logger(), "Pendulum link not found in URDF!");
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
    double pendulum_mass_ = 0.0;
    double pendulum_Iyy_ = 0.0;
    double pendulum_length_ = 0.0;
    double total_mass_ = 0.0;
    double wheel_radius_ = 0.0;
    bool urdf_loaded_ = false;
};

#endif // URDF_PARSER_HPP
