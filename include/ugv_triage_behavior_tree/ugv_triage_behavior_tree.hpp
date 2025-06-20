
#ifndef __HIGHLEVEL_BEHAVIOR_TREE_HPP__
#define __HIGHLEVEL_BEHAVIOR_TREE_HPP__


// C++ standard headers.
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// ROS headers.
#include <rclcpp/rclcpp.hpp>


// Behavior Tree headers.
#include <behavior_tree/behavior_tree.h>

// msgs
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>


enum class BehaviorState_t : uint8_t {
    IDLE    = 0,
    EXPLORE = 1,
    INSPECT = 2,
    MANUAL = 4
};

inline BehaviorState_t integer_to_behavior_state(uint8_t state) {
    switch (state) {
        case 0:
            return BehaviorState_t::IDLE;
        case 1:
            return BehaviorState_t::EXPLORE;
        case 2:
            return BehaviorState_t::INSPECT;
        case 4:
            return BehaviorState_t::MANUAL;
        default:
            std::stringstream ss;
            ss << "Invalid behavior state: " << state;  
            throw std::invalid_argument(ss.str());
    }
}

class Behavior_Executive : public rclcpp::Node {

public:
    Behavior_Executive();
    virtual ~Behavior_Executive();
    void initialize();

private:
    void pub_updates();
    void prepare_execute_units();
    void execute();

    void callback_state(std_msgs::msg::UInt8 msg);
    void callback_init(std_msgs::msg::Bool msg);
    void callback_estop(std_msgs::msg::Bool msg);
    void timer_callback_execution();
    void timer_callback_init();
    

private:

    std::vector<std::shared_ptr<bt::Condition>> conditions_;
    std::vector<std::shared_ptr<bt::Action>> actions_;
    std::vector<std::function<void()>> execution_units_;

    // Conditions
    std::shared_ptr<bt::Condition> condition_system_init;
    std::shared_ptr<bt::Condition> condition_estop;
    std::shared_ptr<bt::Condition> condition_init_timeout;
    std::shared_ptr<bt::Condition> condition_idle_mode_req;   // Idle Mode Requested.
    std::shared_ptr<bt::Condition> condition_manual_mode_req;   // Manual Mode Requested.
    std::shared_ptr<bt::Condition> condition_inspect_mode_req; // Inspect Mode Requested.
    std::shared_ptr<bt::Condition> condition_explore_mode_req;   // Explore Mode Requested.

    // Actions
    std::shared_ptr<bt::Action> action_idle_mode;
    std::shared_ptr<bt::Action> action_manual_mode;
    std::shared_ptr<bt::Action> action_inspect_mode;
    std::shared_ptr<bt::Action> action_explore_mode;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_current_state_int;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_in_explore_mode;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_in_inspect_mode;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_in_manual_mode;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_state;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_init;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_execution;
    rclcpp::TimerBase::SharedPtr timer_init_action;
    
    // Topic names
    std::string init_topic;
    std::string estop_topic;
    std::string current_state_topic;
    std::string explore_mode_topic;
    std::string inspect_mode_topic;
    std::string manual_mode_topic;
    std::string state_selection_topic;

    BehaviorState_t curr_mode;

    int init_timeout_ms;
    bool system_init_timer_running;
    
    bool initialized_;
};

#endif // __ROBOT_BEHAVIOR_EXECUTIVE_ROBOT_BEHAVIOR_EXECUTIVE_HPP__
