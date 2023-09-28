#include <rclcpp/rclcpp.hpp>
#include "../AsyncSerial.h"
#include "../comm_pro.hpp"

#include "robione_ros2_driver/msg/control_cmd.hpp"
#include "robione_ros2_driver/msg/vehicle_status.hpp"
#include "robione_ros2_driver/msg/vehicle_info.hpp"
#include "robione_ros2_driver/msg/system_status.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "robione_ros2_driver/serial_interface.h"

namespace robione{
    class RobioneInterface : public rclcpp::Node{
    private:
        CallbackAsyncSerial serial_;
        comm_pro comm_pro_;
        serial_interface::CompToLlc comp_to_llc{};
        /*
         * params
        */
        struct params{
            struct serial_config{
                std::string port_name;
                int baud_rate;
            };
            struct commpro_config{
                uint8_t device_id;
                uint8_t vcu_id;
            };
            struct timeout_config{
                float control_cmd_timeout;
                float joystick_timeout;
            };
            struct robione_config{
                bool set_steer_rate;
                bool use_joystick;
            };
            struct topic_config{
                std::string control_cmd_topic;
                std::string joy_topic;
            };

            serial_config serial;
            commpro_config commpro;
            timeout_config timeout;
            robione_config robione;
            topic_config topic;
        };
        params params_;

        /*
        * Vcu communication timer
        */
        rclcpp::TimerBase::SharedPtr timer_;
        void timer_callback();
        /*
         *Subscribers
         */
        robione_ros2_driver::msg::ControlCmd::ConstSharedPtr control_cmd_ptr_;
        rclcpp::Subscription<robione_ros2_driver::msg::ControlCmd>::SharedPtr sub_control_cmd_;
        void control_cmd_callback(const robione_ros2_driver::msg::ControlCmd::SharedPtr msg);
        rclcpp::Time control_cmd_time_;

        sensor_msgs::msg::Joy::ConstSharedPtr joy_ptr_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        rclcpp::Time joy_time_;


        /*
         * PUBLISHERS
         */
        rclcpp::Publisher<robione_ros2_driver::msg::VehicleStatus>::SharedPtr pub_vehicle_status_;
        rclcpp::Publisher<robione_ros2_driver::msg::VehicleInfo>::SharedPtr pub_vehicle_info_;
        rclcpp::Publisher<robione_ros2_driver::msg::SystemStatus>::SharedPtr pub_system_status_;

    public:
        RobioneInterface();
        ~RobioneInterface() override{
            serial_.close();
        }

        void load_params();
        void try_serial_connection();

        void serial_callback(const char *buffer, size_t len);

        void commpro_callback(const comm_pro *comm_pro);

        void create_subs();


        bool check_timeout();

        bool check_control_cmd(const robione_ros2_driver::msg::ControlCmd::ConstSharedPtr control_cmd_ptr);

        void create_pubs();
    };
}