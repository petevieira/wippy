#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <chrono>
#include <pthread.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "mobility/urdf_parser.hpp"
#include "mobility/mobility_node.hpp"

MobilityNode::MobilityNode() : Node("mobility_node") {
    // Set thread to high priority for real-time control
    setThreadPriority(98); // 1-99, 99 is highest priority

    // Create message subscriptions
    imu_sub_ = this->createImuSubscriber();

    // Create message publishers
    torque_cmd_pub_ = this->createTorqueCmdPublisher();
    state_pub_ = this->createStatePublisher();

    // Read robot parameters from URDF
    readRobotParamsFromUrdf();

    // Start real-time control loop using controlLoop as callback function
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&MobilityNode::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Mobility node started");
}

void MobilityNode::setThreadPriority(int priority) {
    struct sched_param sch_params;
    sch_params.sched_priority = priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch_params) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set thread priority");
    } else {
        RCLCPP_INFO(this->get_logger(), "Thread priority set to %d", priority);
    }
}

void MobilityNode::readRobotParamsFromUrdf() {
    urdf_parser_ = std::make_shared<UrdfParser>();

    // Wait for URDF to be loaded
    while (!urdf_parser_->isUrdfLoaded() && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for URDF to be loaded...");
        auto temp_node = std::make_shared<rclcpp::Node>("temp_node");
        rclcpp::spin_some(temp_node);
    }

    // URDF loaded, get parameters
    urdf_parser_->getUrdfParams(
        pendulum_mass_, pendulum_Iyy_, pendulum_length_, total_mass_, wheel_radius_
    );

    RCLCPP_INFO(
        this->get_logger(),
        "Loaded URDF - Pendulum Mass: %f kg, Iyy: %f kg·m², Length: %f m, Total Mass: %f kg, Wheel Radius: %f m",
        pendulum_mass_, pendulum_Iyy_, pendulum_length_, total_mass_, wheel_radius_
    );
}

rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
MobilityNode::createImuSubscriber() {
    return this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imuCallback(msg); });
}

rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
MobilityNode::createTorqueCmdPublisher() {
    return this->create_publisher<std_msgs::msg::Float64MultiArray>("torque_cmds", 10);
}

rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr
MobilityNode::createStatePublisher() {
    return this->create_publisher<geometry_msgs::msg::Vector3>("simulated_state", 10);
}

void MobilityNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double pitch = msg->orientation.y;
    double pitch_vel = msg->angular_velocity.y;

    // Process IMU data
    std_msgs::msg::Float64MultiArray torque_cmd;
    torque_cmd.data.push_back(0.0);
    torque_cmd_pub_->publish(torque_cmd);

    geometry_msgs::msg::Vector3 state;
    state.x = 0.0;
    state.y = 0.0;
    state.z = 0.0;
    state_pub_->publish(state);
}

void MobilityNode::sendTorqueCmd(double left_wheel, double right_wheel) {
    auto torque_cmd = std_msgs::msg::Float64MultiArray();
    torque_cmd.data = {left_wheel, right_wheel};
    torque_cmd_pub_->publish(torque_cmd);
}

void MobilityNode::simulateState(double x, double y, double theta) {
    auto state_msg = geometry_msgs::msg::Vector3();
    state_msg.x = x;
    state_msg.y = y;
    state_msg.z = theta;
    state_pub_->publish(state_msg);
}

void MobilityNode::computeControl(double pitch, double pitch_vel) {
    // Create 4x4 A matrix (system dynamics)
    double r_dot_term = -pendulum_mass_ * gravity_ * pow(pendulum_length_, 2) / pendulum_Iyy_;
    double theta_dot_term = pendulum_mass_ * pendulum_length_ * gravity_ / pendulum_Iyy_;
    Eigen::Matrix4d A;
    A << 0, 1,       0,        0,
            0, 0,  r_dot_term,    0,
            0, 0,       0,        1,
            0, 0, theta_dot_term, 0;

    // Create 4x1 B matrix (how system is influenced by control input)
    Eigen::Vector4d B;
    B << 0,
            (pendulum_mass_ * pow(pendulum_length_, 2) + pendulum_Iyy_) / (pendulum_mass_ * pendulum_Iyy_),
            0,
            -pendulum_length_ / pendulum_Iyy_;

    // Create gain matrix K
    Eigen::Matrix<double, 1, 4> K;
    K << this->k_gain[0], this->k_gain[1], this->k_gain[2], this->k_gain[3];

    // Control logic
    sendTorqueCmd(0.0, 0.0);
    simulateState(0.0, 0.0, 0.0);
}

void MobilityNode::logTime() {
    auto now = this->now();
    RCLCPP_INFO(
        this->get_logger(), "Current time: %f.%09ld sec", now.seconds(), now.nanoseconds()
    );
}

void MobilityNode::controlLoop() {
    // Control logic
    sendTorqueCmd(0.0, 0.0);
    simulateState(0.0, 0.0, 0.0);
}
