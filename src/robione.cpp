#include "robione_ros2_driver/robione.h"
#include "robione_ros2_driver/serial_converter.h"
namespace robione{

    RobioneInterface::RobioneInterface() : Node("robione_ros2_driver")
    {
        load_params();
        create_subs();
        create_pubs();
        try_serial_connection();


        comm_pro_.set_device_id(params_.commpro.device_id);
        comm_pro_.set_callback(std::bind(&RobioneInterface::commpro_callback, this, std::placeholders::_1));


        RCLCPP_INFO(this->get_logger(), "Robione interface is initialized successfully.");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&RobioneInterface::timer_callback, this));
    }

    void RobioneInterface::timer_callback() {
        if (serial_.isOpen()) {
            comp_to_llc.msg_counter = SerialConverter::getMsgCounter();
            if(check_timeout()){
                if(params_.robione.use_joystick){
                    SerialConverter::convert_joy_to_llc(comp_to_llc, joy_ptr_);
                }
                else{
                    if(check_control_cmd(control_cmd_ptr_)){
                        comp_to_llc = SerialConverter::convert_control_cmd_to_llc(control_cmd_ptr_, params_.robione.set_steer_rate);
                    }
                    else{
                        comp_to_llc.fill_zero();
                        comp_to_llc.emergency_cmd = SerialConverter::EmergencyStatus::EMERGENCY;
                    }
                }
            }
            else{
                comp_to_llc.fill_zero();
                comp_to_llc.emergency_cmd = SerialConverter::EmergencyStatus::EMERGENCY;
            }
            comm_pro_.fill_comm_pro(serial_interface::MessageID::COMP_TO_LLC_MSG_ID, params_.commpro.vcu_id, reinterpret_cast<const uint8_t*>(&comp_to_llc), sizeof(comp_to_llc));
            serial_.write(comm_pro_.get_raw_comm_pro());
        }
    }


    void RobioneInterface::load_params() {
        params_.serial.port_name = this->declare_parameter<std::string>("serial_config.port", "/dev/ttyUSB0");
        params_.serial.baud_rate = this->declare_parameter<int>("serial_config.baudrate", 115200);
        params_.topic.control_cmd_topic = this->declare_parameter<std::string>("topic_config.control_cmd_topic", "/interface/control_cmd");
        params_.topic.joy_topic = this->declare_parameter<std::string>("topic_config.joystick_topic", "/joy");
        params_.commpro.device_id = this->declare_parameter<uint8_t>("commpro_config.interface_id", 0x01);
        params_.commpro.vcu_id = this->declare_parameter<uint8_t>("commpro_config.vcu_id", 0x0);
        params_.timeout.control_cmd_timeout = this->declare_parameter<float>("timeout_config.control_cmd_timeout", 0.5);
        params_.timeout.joystick_timeout = this->declare_parameter<float>("timeout_config.joystick_timeout", 0.5);
        params_.robione.set_steer_rate = this->declare_parameter<bool>("robione_config.set_steer_rate", false);
        params_.robione.use_joystick = this->declare_parameter<bool>("robione_config.use_joystick", false);

        RCLCPP_INFO(this->get_logger(), "\033[1;34mConfigurations are loaded.\033[0m");
        RCLCPP_INFO(this->get_logger(), "\033[1;34m--------------------------------------------------------\033[0m");
        RCLCPP_INFO(this->get_logger(), "\033[1;34mSerial config:\033[0m");
        RCLCPP_INFO(this->get_logger(), "port_name: %s", params_.serial.port_name.c_str());
        RCLCPP_INFO(this->get_logger(), "baud_rate: %d", params_.serial.baud_rate);
        RCLCPP_INFO(this->get_logger(), "\033[1;34mTopic config:\033[0m");
        RCLCPP_INFO(this->get_logger(), "control_cmd_topic: %s", params_.topic.control_cmd_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "joy_topic: %s", params_.topic.joy_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "\033[1;34mCommPro config:\033[0m");
        RCLCPP_INFO(this->get_logger(), "interface_id: %d", params_.commpro.device_id);
        RCLCPP_INFO(this->get_logger(), "vcu_id: %d", params_.commpro.vcu_id);
        RCLCPP_INFO(this->get_logger(), "\033[1;34mTimeout config:\033[0m");
        RCLCPP_INFO(this->get_logger(), "control_cmd_timeout: %f", params_.timeout.control_cmd_timeout);
        RCLCPP_INFO(this->get_logger(), "joystick_timeout: %f", params_.timeout.joystick_timeout);
        RCLCPP_INFO(this->get_logger(), "\033[1;34mRobione config:\033[0m");
        RCLCPP_INFO(this->get_logger(), "set_steer_rate: %d", params_.robione.set_steer_rate);
        RCLCPP_INFO(this->get_logger(), "use_joystick: %d", params_.robione.use_joystick);
        RCLCPP_INFO(this->get_logger(), "\033[1;34m--------------------------------------------------------\033[0m");
    }

    void RobioneInterface::try_serial_connection(){
        do{
            try {
                serial_.open(params_.serial.port_name, params_.serial.baud_rate);
            }
            catch (std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open the serial port!!!");
                RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
                rclcpp::sleep_for(std::chrono::seconds(1));
            }
        }while(!serial_.isOpen() && rclcpp::ok());
        if(serial_.isOpen()){
            RCLCPP_INFO(this->get_logger(), "Serial port is opened successfully.");
            serial_.setCallback(std::bind(&RobioneInterface::serial_callback, this, std::placeholders::_1, std::placeholders::_2));
        }
    }


    void RobioneInterface::serial_callback(const char* buffer, size_t len) {
        comm_pro_.add_data(reinterpret_cast<const uint8_t*>(buffer), len);
    }

    void RobioneInterface::commpro_callback(const comm_pro *comm_pro) {
        auto solved_comm = comm_pro->get_solved_comm_pro();
        switch(solved_comm.msgId){
            case serial_interface::MessageID::LLC_MSG_ID:{
                serial_interface::LlcToComp llc_to_comp{};
                std::memcpy(reinterpret_cast<void*>(&llc_to_comp), solved_comm.msg, sizeof(llc_to_comp));
                pub_vehicle_status_->publish(SerialConverter::convert_llc_to_vehicle_status(llc_to_comp));
                break;
            }
            case serial_interface::MessageID::DEBUG_MSG_FAST_ID:{
                serial_interface::DebugMsgFast_t debug_fast{};
                std::memcpy(reinterpret_cast<void*>(&debug_fast), solved_comm.msg, sizeof(debug_fast));
                pub_system_status_->publish(SerialConverter::convert_debug_to_system_status(debug_fast));
                break;
            }
            case serial_interface::MessageID::DEBUG_MSG_SLOW_ID:{
                serial_interface::DebugMsgSlow_t debug_slow{};
                std::memcpy(reinterpret_cast<void*>(&debug_slow), solved_comm.msg, sizeof(debug_slow));
                pub_vehicle_info_->publish(SerialConverter::convert_debug_to_vehicle_info(debug_slow));
                break;
            }
        }
    }

    void RobioneInterface::create_subs(){
        if(params_.robione.use_joystick){
            sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
                    params_.topic.joy_topic, 10, std::bind(&RobioneInterface::joy_callback, this, std::placeholders::_1));
        }
        else{
            sub_control_cmd_ = this->create_subscription<robione_ros2_driver::msg::ControlCmd>(
                    params_.topic.control_cmd_topic, 10, std::bind(&RobioneInterface::control_cmd_callback, this, std::placeholders::_1));
        }
    }

    void RobioneInterface::create_pubs(){
        pub_vehicle_status_ = this->create_publisher<robione_ros2_driver::msg::VehicleStatus>("/robione/vehicle_status", 10);
        pub_vehicle_info_ = this->create_publisher<robione_ros2_driver::msg::VehicleInfo>("/robione/vehicle_info", 10);
        pub_system_status_ = this->create_publisher<robione_ros2_driver::msg::SystemStatus>("/robione/system_status", 10);
    }

    void RobioneInterface::control_cmd_callback(const robione_ros2_driver::msg::ControlCmd::SharedPtr msg) {
        control_cmd_ptr_ = msg;
        control_cmd_time_ = rclcpp::Clock().now();
    }

    void RobioneInterface::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        joy_ptr_ = msg;
        joy_time_ = rclcpp::Clock().now();
    }

    bool RobioneInterface::check_timeout(){
        rclcpp::Clock clock{ RCL_ROS_TIME };
        bool state = true;
        if(!params_.robione.use_joystick){
            if(control_cmd_ptr_ == nullptr){
                RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "\033[1;33mControl cmd is not received yet.\033[0m");
                state = false;
            }
            else if((rclcpp::Clock().now() - control_cmd_time_).seconds() > params_.timeout.control_cmd_timeout){
                RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "\033[1;33mControl cmd is not received for %f seconds.\033[0m", params_.timeout.control_cmd_timeout);
                state = false;
            }
        }
        else{
            if(joy_ptr_ == nullptr){
                RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "\033[1;33mJoystick cmd is not received yet.\033[0m");
                state = false;
            }
            else if((rclcpp::Clock().now() - joy_time_).seconds() > params_.timeout.joystick_timeout){
                RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "\033[1;33mJoystick cmd is not received for %f seconds.\033[0m", params_.timeout.joystick_timeout);
                state = false;
            }
        }

        return state;
    }

    bool RobioneInterface::check_control_cmd(const robione_ros2_driver::msg::ControlCmd::ConstSharedPtr control_cmd_ptr){
        rclcpp::Clock clock{ RCL_ROS_TIME };
        bool state = true;
        if(control_cmd_ptr->throttle > 1.0 || control_cmd_ptr->throttle < 0.0){
            state = false;
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "\033[1;33mThrottle value is out of range.\033[0m");
        }

        if(control_cmd_ptr->brake > 1.0 || control_cmd_ptr->brake < 0.0){
            state = false;
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "\033[1;33mBrake value is out of range.\033[0m");
        }

        if(control_cmd_ptr->steer > 1.0 || control_cmd_ptr->steer < -1.0){
            state = false;
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "\033[1;33mSteer value is out of range.\033[0m");
        }

        if(control_cmd_ptr->steer_rate > 545.0 || control_cmd_ptr->steer_rate < 43.0){
            state = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "\033[1;33mSteer rate value is out of range.\033[0m");
        }

        if(control_cmd_ptr->handbrake > robione_ros2_driver::msg::ControlCmd::HANDBRAKE_OFF){
            state = false;
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "\033[1;33mHandbrake value is out of range.\033[0m");
        }

        if(control_cmd_ptr->gear > robione_ros2_driver::msg::ControlCmd::GEAR_REVERSE){
            state = false;
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "\033[1;33mGear value is out of range.\033[0m");
        }

        if(control_cmd_ptr->emergency > robione_ros2_driver::msg::ControlCmd::EMERGENCY_ON){
            state = false;
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "\033[1;33mEmergency value is out of range.\033[0m");
        }
        return state;
    }


}
