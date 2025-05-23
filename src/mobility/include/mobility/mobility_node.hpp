#ifndef MOBILITY_NODE_HPP
#define MOBILITY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "urdf_parser.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

/**
 * MobilityNode class
 * This class handles the mobility of the robot by subscribing to IMU data,
 * publishing torque commands, and simulating the state of the system.
 * It uses a real-time control loop to process the data and send commands.
 * The class is designed to be used with ROS 2 and requires the URDF of the robot.
 */
/**
 * Constructor
 * Initializes the node, sets thread priority, creates subscribers and publishers,
 * and reads robot parameters from URDF.
 */
class MobilityNode : public rclcpp::Node {
public:

    explicit MobilityNode();

private:
    /**
     * Set thread priority for real-time control
     * @param priority thread priority (1-99, 99 is highest)
     */
    void setThreadPriority(int priority);

    /**
     * Read robot parameters from URDF
     */
    void readRobotParamsFromUrdf();

    /**
     * IMU callback function
     * This function is called when new IMU data is received.
     * @param msg IMU message
     */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * Send torque command to motors
     * @param left_wheel Torque command for left wheel
     * @param right_wheel Torque command for right wheel
     */
    void sendTorqueCmd(double left_wheel, double right_wheel);

    /**
     * Simulate state of the system
     * @param x x position
     * @param y y position
     * @param theta orientation
     */
    void simulateState(double x, double y, double theta);

    /**
     * Compute control command
     * This function computes the control command based on the pitch and pitch velocity.
     * @param pitch Pitch angle
     * @param pitch_vel Pitch velocity
     */
    void computeControl(double pitch, double pitch_vel);

    /**
     * Control loop
     * This function runs in a separate thread and handles the control loop.
     */
    void controlLoop();

    /**
     * Log time
     * This function logs the time taken for each control loop iteration.
     */
    void logTime();

    /**
     * Create IMU subscriber.
     * There should be an IMU in the URDF robot description, which should publish IMU data.
     * @return Shared pointer to the IMU subscriber
     */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr createImuSubscriber();

    /**
     * Create torque command publisher.
     * This will publish torque commands to the real or simulated motors.
     * @return Shared pointer to the torque command publisher
     */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr createTorqueCmdPublisher();

    /**
     * Create state publisher.
     * This will publish the state of the system (x, y, theta).
     * @return Shared pointer to the state publisher
     */
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr createStatePublisher();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr state_pub_;
    std::shared_ptr<UrdfParser> urdf_parser_;

    double pendulum_mass_;
    double pendulum_Iyy_;
    double pendulum_length_;
    double total_mass_;
    double wheel_radius_;
    double gravity_ = 9.81;
    double k_gain[4] = {1.0, 1.70785474, -26.26305572, -2.78540675};
};

#endif // MOBILITY_NODE_HPP
