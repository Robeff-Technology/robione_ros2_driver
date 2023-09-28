//
// Created by elaydin on 9/26/23.
//

#ifndef SERIAL_CONVERTER_H
#define SERIAL_CONVERTER_H
#include "serial_interface.h"
#include "robione_ros2_driver/msg/control_cmd.hpp"
#include "robione_ros2_driver/msg/vehicle_status.hpp"
#include "robione_ros2_driver/msg/vehicle_info.hpp"
#include "robione_ros2_driver/msg/system_status.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <cstdint>
namespace robione{
    class SerialConverter {
    private:
        static uint32_t msg_counter_;
        SerialConverter() = default;
        ~SerialConverter() = default;
    public:

        enum JoyAxes: uint8_t{
            STEER_AXIS = 0x00,
            THROTTLE_AXIS = 0x04,
            DIRECTION_AXIS = 0x07,
        };

        enum JoyButtons:uint8_t{
            HANDBRAKE_BUTTON = 0x05,
            RELEASE_BUTTON = 0x04,
            EMERGENCY_BUTTON = 0x03,
        };

        enum EmergencyStatus: uint8_t{
            NO_EMERGENCY = 0x00,
            EMERGENCY = 0x01,
        };

        enum HandBrakeStatus: uint8_t{
            PRESS = 0x00,
            RELEASE = 0x01,
        };

        enum GearStatus: uint8_t{
            DRIVE = 0x01,
            REVERSE = 0x02,
            PARK = 0x03
        };

        static serial_interface::CompToLlc convert_control_cmd_to_llc(const robione_ros2_driver::msg::ControlCmd::ConstSharedPtr msg, bool set_steer_rate);
        static void convert_joy_to_llc(serial_interface::CompToLlc &comp_to_llc, const sensor_msgs::msg::Joy::ConstSharedPtr msg);

        static robione_ros2_driver::msg::VehicleStatus convert_llc_to_vehicle_status(const serial_interface::LlcToComp &llc_to_comp);
        static robione_ros2_driver::msg::VehicleInfo convert_debug_to_vehicle_info(const serial_interface::DebugMsgSlow_t &debug_slow);
        static robione_ros2_driver::msg::SystemStatus convert_debug_to_system_status(const serial_interface::DebugMsgFast_t &debug_fast);
        static uint32_t getMsgCounter(){return msg_counter_++;}
    };
}

#endif //SERIAL_CONVERTER_H
