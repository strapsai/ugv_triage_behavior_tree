
#include "ugv_triage_behavior_tree/ugv_triage_behavior_tree.hpp"

Behavior_Executive::Behavior_Executive()
: Node("ugv_triage_behavior_executive"),
  init_topic{"sub_init"},
  estop_topic{"sub_estop"},
  current_state_topic{"current_state_int"},
  explore_mode_topic{"in_explore_mode"},
  inspect_mode_topic{"in_inspect_mode"},
  manual_mode_topic{"in_manual_mode"},
  state_selection_topic{"state_selection_topic"},

  exploration_request_topic{"exploration_request_topic"},
  observed_casualty_topic{"observed_casualty_topic"},
  robot_gps_topic{"robot_gps_topic"},

  curr_mode(BehaviorState_t::IDLE),
  init_timeout_ms(5000),
  system_init_timer_running(false),
  initialized_(false)
{
    this->declare_parameter("init_timeout_ms", 5000);
    this->declare_parameter("init_topic", "sub_init");
    this->declare_parameter("estop_topic", "sub_estop");
    this->declare_parameter("current_state_topic", "current_state_int");
    this->declare_parameter("explore_mode_topic", "in_explore_mode");
    this->declare_parameter("inspect_mode_topic", "in_inspect_mode");
    this->declare_parameter("manual_mode_topic", "in_manual_mode");
    this->declare_parameter("state_selection_topic", "state_selection_topic");

    this->declare_parameter("exploration_request_topic", "exploration_request_topic");
    this->declare_parameter("observed_casualty_topic", "observed_casualty_topic");
    this->declare_parameter("robot_gps_topic", "robot_gps_topic");
}

Behavior_Executive::~Behavior_Executive() {
    RCLCPP_INFO(this->get_logger(), "Behavior Executive Node Destroyed");
}

void Behavior_Executive::initialize() {

    if (initialized_) {
        RCLCPP_WARN(this->get_logger(), "Already initialized");
    }

    // ==================================================
    // Get parameters.
    // ==================================================
    this->get_parameter("init_timeout_ms", init_timeout_ms);
    this->get_parameter("init_topic", init_topic);
    this->get_parameter("estop_topic", estop_topic);
    this->get_parameter("current_state_topic", current_state_topic);
    this->get_parameter("explore_mode_topic", explore_mode_topic);
    this->get_parameter("inspect_mode_topic", inspect_mode_topic);
    this->get_parameter("manual_mode_topic", manual_mode_topic);
    this->get_parameter("state_selection_topic", state_selection_topic);
    RCLCPP_INFO(this->get_logger(), "Init Timeout: %d", init_timeout_ms);
    RCLCPP_INFO(this->get_logger(), "Init Topic: %s", init_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Estop Topic: %s", estop_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Current State Topic: %s", current_state_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "In Explore Mode Topic: %s", explore_mode_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "In Inspect Mode Topic: %s", inspect_mode_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "In Manual Mode Topic: %s", manual_mode_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "State Selection Topic: %s", state_selection_topic.c_str());

    this->get_parameter("exploration_request_topic", exploration_request_topic);
    this->get_parameter("observed_casualty_topic", observed_casualty_topic);
    this->get_parameter("robot_gps_topic", robot_gps_topic);
    RCLCPP_INFO(this->get_logger(), "Exploration Request Topic: %s", exploration_request_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Observed Casualty Topic: %s", observed_casualty_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Robot GPS Position Topic: %s", robot_gps_topic.c_str());

    RCLCPP_INFO(this->get_logger(), "Behavior Executive Node Initialized");

    // ==================================================
    // Initialize the conditions.
    // ==================================================
    condition_system_init = std::make_shared<bt::Condition>("System Init", this);
    conditions_.push_back(condition_system_init);

    condition_init_timeout = std::make_shared<bt::Condition>("Init Timeout", this);
    conditions_.push_back(condition_init_timeout);

    condition_estop = std::make_shared<bt::Condition>("Estop", this);
    conditions_.push_back(condition_estop);
    condition_estop->set(false);

    condition_idle_mode_req = std::make_shared<bt::Condition>("Idle Mode Req", this);  
    conditions_.push_back(condition_idle_mode_req);

    condition_manual_mode_req = std::make_shared<bt::Condition>("Manual Mode Req", this);  
    conditions_.push_back(condition_manual_mode_req);

    condition_inspect_mode_req = std::make_shared<bt::Condition>("Inspect Mode Req", this);  
    conditions_.push_back(condition_inspect_mode_req);

    condition_explore_mode_req = std::make_shared<bt::Condition>("Explore Mode Req", this);
    conditions_.push_back(condition_explore_mode_req);

    condition_approach_mode_req = std::make_shared<bt::Condition>("Approach Mode Req", this);
    conditions_.push_back(condition_approach_mode_req);

    condition_mode_selected = std::make_shared<bt::Condition>("Exploration Mode Selected", this);
    conditions_.push_back(condition_mode_selected);

    // ==================================================
    // Initialize the actions.
    // ==================================================
    action_idle_mode = std::make_shared<bt::Action>("Idle Mode", this);
    actions_.push_back(action_idle_mode);

    action_manual_mode = std::make_shared<bt::Action>("Manual Mode", this);
    actions_.push_back(action_manual_mode);

    action_inspect_mode = std::make_shared<bt::Action>("Inspect Mode", this);
    actions_.push_back(action_inspect_mode);

    action_explore_mode = std::make_shared<bt::Action>("Explore Mode", this);
    actions_.push_back(action_explore_mode);

    action_find_casualty = std::make_shared<bt::Action>("Find Casualty", this);
    actions_.push_back(action_find_casualty);
    
    action_go_to_inspection = std::make_shared<bt::Action>("Go to Inspection", this);
    actions_.push_back(action_go_to_inspection);

    // ==================================================
    // Initialize the publishers.
    // ==================================================
    pub_current_state_int = create_publisher<std_msgs::msg::UInt8>(current_state_topic, 10);
    pub_in_explore_mode = create_publisher<std_msgs::msg::Bool>(explore_mode_topic, 10);
    pub_in_inspect_mode = create_publisher<std_msgs::msg::Bool>(inspect_mode_topic, 10);
    pub_in_manual_mode = create_publisher<std_msgs::msg::Bool>(manual_mode_topic, 10);

    pub_exploration_request = create_publisher<std_msgs::msg::Bool>(exploration_request_topic, 10);
    // ==================================================
    // Initialize the subscribers.
    // ==================================================
    sub_state = create_subscription<std_msgs::msg::UInt8>(
        state_selection_topic, 10,
        std::bind(&Behavior_Executive::callback_state, this, std::placeholders::_1));
    
    sub_init = create_subscription<std_msgs::msg::Bool>(
        init_topic, 10,
        std::bind(&Behavior_Executive::callback_init, this, std::placeholders::_1));

    sub_estop = create_subscription<std_msgs::msg::Bool>(
        estop_topic, 10,
        std::bind(&Behavior_Executive::callback_estop, this, std::placeholders::_1));

    sub_observed_casualties = create_subscription<humanflow_msgs::msg::ReIDPersonArray>(
        observed_casualty_topic, 10,
        std::bind(&Behavior_Executive::callback_casualties, this, std::placeholders::_1));

    sub_robot_gps = create_subscription<sensor_msgs::msg::NavSatFix>(
        robot_gps_topic, 10,
        std::bind(&Behavior_Executive::callback_robot_gps, this, std::placeholders::_1));


    // ==================================================
    // Initialize the timers.
    // ==================================================
    timer_init_action = create_wall_timer(
        std::chrono::milliseconds(init_timeout_ms),
        std::bind(&Behavior_Executive::timer_callback_init, this));
    timer_init_action->cancel();

    timer_execution = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        std::bind(&Behavior_Executive::timer_callback_execution, this));

    prepare_execute_units();

    initialized_ = true;
}

void Behavior_Executive::prepare_execute_units() {
    // Init timer.
    execution_units_.push_back([this]() {
        if(condition_init_timeout->get()) {
            RCLCPP_INFO_STREAM_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000, "System Init Complete");
        } else {
            RCLCPP_INFO_STREAM_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000, "System Init...");
            
            if ( !system_init_timer_running ) {
                timer_init_action->reset();
                system_init_timer_running = true;
            }
        }
    });

    // Idle Mode.
    execution_units_.push_back([this]() {
        if ( action_idle_mode->is_active() ) {
            if ( action_idle_mode->active_has_changed() ) {
                action_idle_mode->set_running();
            }

            if ( action_idle_mode->is_running() ) {
                RCLCPP_INFO_STREAM_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000, "Current Mode: Idle");
            }
        } else {
            if ( action_idle_mode->active_has_changed() ) {
                action_idle_mode->set_failure();
            }
        }
    });

    // Manual Mode.
    execution_units_.push_back([this]() {
        std_msgs::msg::Bool msg;
        msg.data = false;
        if ( action_manual_mode->is_active() ) {
            if ( action_manual_mode->active_has_changed() ) {
                action_manual_mode->set_running();
            }

            if ( action_manual_mode->is_running() ) {
                RCLCPP_INFO_STREAM_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000, "Current Mode: Manual");

                msg.data = true;
            }
        } else {
            if ( action_manual_mode->active_has_changed() ) {
                action_manual_mode->set_failure();
            }
        }

        pub_in_manual_mode->publish(msg);
    });
    
    // Inspect Mode.
    execution_units_.push_back([this]() {
        if ( action_inspect_mode->is_active() ) {
            if ( action_inspect_mode->active_has_changed() ) {
                action_inspect_mode->set_running();
            }
            
            if ( action_inspect_mode->is_running() ) {
                RCLCPP_INFO_STREAM_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000, "Current Mode: Inspect");

                std_msgs::msg::Bool msg;
                msg.data = true;
                pub_in_inspect_mode->publish(msg);
            }
        } else {
            if ( action_inspect_mode->active_has_changed() ) {
                action_inspect_mode->set_failure();
            }

            std_msgs::msg::Bool msg;
            msg.data = false;
            pub_in_inspect_mode->publish(msg);
        }
    });

    // Explore Mode.
    execution_units_.push_back([this]() {
        if ( action_explore_mode->is_active() ) {
            if ( action_explore_mode->active_has_changed() ) {
                action_explore_mode->set_running();
            }
            
            if ( action_explore_mode->is_running() ) {
                RCLCPP_INFO_STREAM_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000, "Current Mode: Explore");

                std_msgs::msg::Bool msg;
                msg.data = true;
                pub_in_explore_mode->publish(msg);
            }
        } else {
            if ( action_explore_mode->active_has_changed() ) {
                action_explore_mode->set_failure();
            }

            std_msgs::msg::Bool msg;
            msg.data = false;
            pub_in_explore_mode->publish(msg);
        }
    });

    execution_units_.push_back([this]() {
        if(condition_mode_selected->get()) {
            RCLCPP_INFO_STREAM_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000, "Exploration Mode Activated...");
        } else {
            RCLCPP_INFO_STREAM_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000, "Exploration Mode Not Active...");
        }
    });

    // Explore Area.
    execution_units_.push_back([this]() {
        if ( action_find_casualty->is_active() ) {
            if ( action_find_casualty->active_has_changed() ) {
                action_find_casualty->set_running();
            }
            
            if ( action_find_casualty->is_running() ) {
                RCLCPP_INFO_STREAM_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000, "Looking for Casualty...");

                //base_node_msgs::msg::WorkingRequest msg;
                std_msgs::msg::Bool msg;
                msg.data = true;
                pub_exploration_request->publish(msg);

                if (reachable_casualty_found){
                  RCLCPP_INFO_STREAM_THROTTLE(
                     this->get_logger(), *this->get_clock(), 1000, "Found a casualty in reach! Will transition to inspection mode.");
                  action_find_casualty->set_success();
                  reachable_casualty_found = false;
                }

            }
        } else {
            if ( action_find_casualty->active_has_changed() ) {
                action_find_casualty->set_failure();
            }
        }
    });

    execution_units_.push_back([this]() {
        if ( action_go_to_inspection->is_active() ) {
            if ( action_go_to_inspection->active_has_changed() ) {
                action_go_to_inspection->set_running();
            }
            
            if ( action_go_to_inspection->is_running() ) {
                RCLCPP_INFO_STREAM_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000, "Looking for Casualty...");

                switch_mode(BehaviorState_t::INSPECT);
            }
        } else {
            if ( action_go_to_inspection->active_has_changed() ) {
                action_go_to_inspection->set_failure();
            }
        }
    });
}

void Behavior_Executive::execute() {
    for (const auto &unit : execution_units_) {
        unit();
    }

    for (const auto &ptr : conditions_) {
        ptr->publish();
    }

    for (const auto &ptr : actions_) {
        ptr->publish();
    }

    pub_updates();
}

void Behavior_Executive::pub_updates(){
    if ( !condition_system_init->get() || !condition_init_timeout->get() ) {
        return;
    }

    std_msgs::msg::UInt8 msg;
    msg.data = static_cast<uint8_t>(curr_mode);
    pub_current_state_int->publish(msg);
}

void Behavior_Executive::callback_state(std_msgs::msg::UInt8 msg) {
    curr_mode = integer_to_behavior_state(msg.data);
    switch_mode(curr_mode);
}

void Behavior_Executive::callback_init(std_msgs::msg::Bool msg){
    if(msg.data == true) {
        condition_system_init->set(true); 
    } else {
        condition_system_init->set(false);
    }
}

void Behavior_Executive::callback_estop(std_msgs::msg::Bool msg){
  condition_estop->set(msg.data); 
}

void Behavior_Executive::callback_mode_select(std_msgs::msg::Bool msg){
    if(msg.data == true) {
      condition_mode_selected->set(true); 
    } else {
      condition_mode_selected->set(false);
    }
}

void Behavior_Executive::callback_casualties(humanflow_msgs::msg::ReIDPersonArray msg){
  if (action_find_casualty->is_active() && action_find_casualty->is_running()){
  if (msg.peoplemap.size() == 0){
    reachable_casualty_found = false;
    return;
  }
  else if (!robot_coordinates_obtained){
    return;
  }
  else {
    for (auto &person : msg.peoplemap){
      auto person_position = person.gps;
      auto robot_position = latest_robot_gps;

      double distance = get_distance(robot_position, person_position);
      RCLCPP_INFO(this->get_logger(), "Casualty ID: %d is %.2fm away.", person.global_id ,distance);
      
      if (distance < INSPECTABLE_DISTANCE){
        reachable_casualty_found = true;
        break;
      }
    }
    
  }

  }
}

void Behavior_Executive::callback_robot_gps(sensor_msgs::msg::NavSatFix msg){
  latest_robot_gps = msg;
  robot_coordinates_obtained = true;
  return;
}

double Behavior_Executive::get_distance(sensor_msgs::msg::NavSatFix a, sensor_msgs::msg::NavSatFix b){ 
  auto a_utm = geographic_utils::latlon_to_utm(a.latitude, a.longitude);
  auto b_utm = geographic_utils::latlon_to_utm(b.latitude, b.longitude);
  if (a_utm.zone != b_utm.zone){
    RCLCPP_ERROR(this->get_logger(), "THE ROBOT AND CASUALTY COORDINATES ARE IN DIFFERENT UTM ZONES!");
    return -1;
  }
  double dif_x = a_utm.x-b_utm.x;
  double dif_y = a_utm.y-b_utm.y;
  return sqrt(dif_x*dif_x+dif_y*dif_y);
}

void Behavior_Executive::switch_mode(BehaviorState_t state){

    switch (state) {
        case BehaviorState_t::IDLE:
            condition_idle_mode_req->set(true);
            condition_inspect_mode_req->set(false);
            condition_explore_mode_req->set(false);
            condition_manual_mode_req->set(false);
            condition_approach_mode_req->set(false);
            break;
        case BehaviorState_t::MANUAL:
            condition_manual_mode_req->set(true);
            condition_inspect_mode_req->set(false);
            condition_explore_mode_req->set(false);
            condition_idle_mode_req->set(false);
            condition_approach_mode_req->set(false);
            break;
        case BehaviorState_t::INSPECT:
            condition_inspect_mode_req->set(true);
            condition_idle_mode_req->set(false);
            condition_explore_mode_req->set(false);
            condition_manual_mode_req->set(false);
            condition_approach_mode_req->set(false);
            break;
        case BehaviorState_t::EXPLORE:
            condition_explore_mode_req->set(true);
            condition_idle_mode_req->set(false);
            condition_inspect_mode_req->set(false);
            condition_manual_mode_req->set(false);
            condition_approach_mode_req->set(false);
            break;
        case BehaviorState_t::APPROACH:
            condition_approach_mode_req->set(true);
            condition_idle_mode_req->set(false);
            condition_inspect_mode_req->set(false);
            condition_explore_mode_req->set(false);
            condition_manual_mode_req->set(false);
    }
}


//Action timer defintions
void Behavior_Executive::timer_callback_init() {
    condition_init_timeout->set(true);
    timer_init_action->cancel();
    system_init_timer_running = false;

    condition_idle_mode_req->set(true);

    RCLCPP_INFO(get_logger(), "Init Timer Timeout");
}

void Behavior_Executive::timer_callback_execution(){
    execute();
}
