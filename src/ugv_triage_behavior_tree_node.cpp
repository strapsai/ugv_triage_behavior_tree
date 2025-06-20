
#include <rclcpp/rclcpp.hpp>

#include "ugv_triage_behavior_tree/ugv_triage_behavior_tree.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto ugv_triage_behavior_tree= std::make_shared<Behavior_Executive>();
    ugv_triage_behavior_tree->initialize();

    rclcpp::spin(ugv_triage_behavior_tree);
    rclcpp::shutdown();
    return 0;
}
