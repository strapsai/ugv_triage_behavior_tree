
#include <rclcpp/rclcpp.hpp>

#include "ugv_top_behavior_tree/ugv_top_behavior_tree.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto ugv_top_behavior_tree= std::make_shared<Behavior_Executive>();
    ugv_top_behavior_tree->initialize();

    rclcpp::spin(ugv_top_behavior_tree);
    rclcpp::shutdown();
    return 0;
}
