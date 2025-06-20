
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
#include <base_node_msgs/msg/working_request.hpp>
#include <humanflow_msgs/msg/re_id_person_array.hpp>
#include <geographic_utils/geographic_utils.hpp>

#include <plan_executor_msgs/msg/milestone.hpp>
#include <base_node_msgs/msg/node_state.hpp>

#include <triage_database_interface/msg/database_array.hpp>

#define INSPECTABLE_DISTANCE 3.0

enum class BehaviorState_t : uint8_t {
    IDLE    = 0,
    EXPLORE = 1,
    INSPECT = 2,
    APPROACH = 3,
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
        case 3:
            return BehaviorState_t::APPROACH;
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
    
    
    void callback_casualties(humanflow_msgs::msg::ReIDPersonArray);
    void callback_robot_gps(sensor_msgs::msg::NavSatFix);

    void callback_milestone(plan_executor_msgs::msg::Milestone msg);
    void callback_rd(base_node_msgs::msg::NodeState msg);
    void callback_hemo(base_node_msgs::msg::NodeState msg);

    void callback_database(triage_database_interface::msg::DatabaseArray msg);

    void clear_milstone(int id);
    

    double get_distance(sensor_msgs::msg::NavSatFix a, sensor_msgs::msg::NavSatFix b);
    void switch_mode(BehaviorState_t state);
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
    std::shared_ptr<bt::Condition> condition_approach_mode_req;   // Approach Mode Requested.
    
    std::shared_ptr<bt::Condition> condition_m_multiview_received;
    std::shared_ptr<bt::Condition> condition_m_stop_received;

    // Actions
    std::shared_ptr<bt::Action> action_idle_mode;
    std::shared_ptr<bt::Action> action_manual_mode;
    std::shared_ptr<bt::Action> action_inspect_mode;
    std::shared_ptr<bt::Action> action_explore_mode;

    std::shared_ptr<bt::Action> action_find_casualty;
    std::shared_ptr<bt::Action> action_go_to_inspection;

    std::shared_ptr<bt::Action> action_request_plan;
    std::shared_ptr<bt::Action> action_wait_milestone;
    std::shared_ptr<bt::Action> action_RD;
    std::shared_ptr<bt::Action> action_HEMO;
    std::shared_ptr<bt::Action> action_multiview_milestone_clear;
    std::shared_ptr<bt::Action> action_reset;
    // Publishers
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_current_state_int;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_in_explore_mode;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_in_inspect_mode;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_in_manual_mode;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_exploration_request;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_inspecting_target;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_plan_request;
    rclcpp::Publisher<base_node_msgs::msg::WorkingRequest>::SharedPtr pub_RD_request;
    rclcpp::Publisher<base_node_msgs::msg::WorkingRequest>::SharedPtr pub_HEMO_request;
    rclcpp::Publisher<plan_executor_msgs::msg::Milestone>::SharedPtr pub_milestone;
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_state;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_init;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop;

    rclcpp::Subscription<humanflow_msgs::msg::ReIDPersonArray>::SharedPtr sub_observed_casualties;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_robot_gps;

    rclcpp::Subscription<plan_executor_msgs::msg::Milestone>::SharedPtr sub_milestone;
    rclcpp::Subscription<base_node_msgs::msg::NodeState>::SharedPtr sub_rd_nodestate;
    rclcpp::Subscription<base_node_msgs::msg::NodeState>::SharedPtr sub_hemo_nodestate;

    rclcpp::Subscription<triage_database_interface::msg::DatabaseArray>::SharedPtr sub_database;
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

    std::string exploration_mode_activate_topic;
    std::string exploration_request_topic;
    std::string observed_casualty_topic;
    std::string robot_gps_topic;

    std::string milestone_in_topic;
    std::string milestone_out_topic;
    std::string inspecting_target_topic;
    std::string plan_request_topic;
    std::string rd_request_topic;
    std::string hemo_request_topic;
    std::string rd_nodestate_topic;
    std::string hemo_nodestate_topic;

    std::string database_topic;

    BehaviorState_t curr_mode;

    int init_timeout_ms;
    bool system_init_timer_running;


    bool robot_coordinates_obtained = false;
    sensor_msgs::msg::NavSatFix latest_robot_gps;

    bool reachable_casualty_found=false;

    std::list<std::pair<int, int>> milestones; //ID, index

    bool rd_inspection_state_prev = -1;
    bool rd_obtained = false;

    int hemo_inspection_state_prev = -1;
    int hemo_obtained = false;
    
    bool initialized_;
};

#endif // __ROBOT_BEHAVIOR_EXECUTIVE_ROBOT_BEHAVIOR_EXECUTIVE_HPP__
