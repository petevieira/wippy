#include "mobility/mobility_node.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * Adds the node to the executor and starts the spin loop.
 * This triggers the Mobility Node's constructor.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Use StaticSingleThreadExecutor for deterministic execution
    auto node = std::make_shared<MobilityNode>();
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}