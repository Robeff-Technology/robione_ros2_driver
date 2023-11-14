#include "robione_ros2_driver/serial_converter.h"
#include <rclcpp/rclcpp.hpp>
//
// Created by elaydin on 9/26/23.
//

namespace robione{
    uint32_t SerialConverter::msg_counter_ = 0;

     serial_interface::CompToLlc SerialConverter::convert_control_cmd_to_llc(robione_ros2_driver::msg::ControlCmd::ConstSharedPtr msg, bool set_steer_rate){
         serial_interface::CompToLlc comp_to_llc{};
         comp_to_llc.msg_counter = getMsgCounter();
         comp_to_llc.long_cmd.set_throttle = msg->throttle;
         comp_to_llc.long_cmd.set_brake = msg->brake;
         comp_to_llc.lat_cmd.set_steering_angle = msg->steer;
         if(set_steer_rate){
            comp_to_llc.lat_cmd.set_steering_angle_rate = msg->steer_rate;
            comp_to_llc.vehicle_cmd.mode = 0x01;
         }
         else{
            comp_to_llc.lat_cmd.set_steering_angle_rate = 0.0;
            comp_to_llc.vehicle_cmd.mode = 0x00;
         }
         comp_to_llc.vehicle_cmd.gear = msg->gear;
         comp_to_llc.vehicle_cmd.handbrake = msg->handbrake;
         comp_to_llc.vehicle_cmd.blinkers = msg->blinkers;
         comp_to_llc.vehicle_cmd.head_light = msg->head_light;
         comp_to_llc.emergency_cmd = msg->emergency;
         return comp_to_llc;
     }

     void SerialConverter::convert_joy_to_llc(serial_interface::CompToLlc &comp_to_llc, const sensor_msgs::msg::Joy::ConstSharedPtr msg){
         /*
          * axes[0]->steer 1->left -1->right
          * axes[4]->throttle 0-1->throttle 0-(-1)->brake
          * axes[6]-> park-> -1
          * axes[7]->direction (1.0->forward -1.0->reverse)
          * button[5]->handbrake
          * button[4]->release
          * button[3]->emergency
          */
         static uint8_t last_emergency_press = 0;
         comp_to_llc.msg_counter = getMsgCounter();
         comp_to_llc.long_cmd.set_throttle = msg->axes[JoyAxes::THROTTLE_AXIS] > 0.0 ? msg->axes[JoyAxes::THROTTLE_AXIS] : 0.0;
         comp_to_llc.long_cmd.set_brake = msg->axes[JoyAxes::THROTTLE_AXIS] < 0.0 ? -msg->axes[JoyAxes::THROTTLE_AXIS] : 0.0;
         comp_to_llc.lat_cmd.set_steering_angle = msg->axes[JoyAxes::STEER_AXIS];
         comp_to_llc.lat_cmd.set_steering_angle_rate = 0.0;
         comp_to_llc.vehicle_cmd.mode = 0x00;
         if(msg->buttons[JoyButtons::HANDBRAKE_BUTTON] > 0){
            comp_to_llc.vehicle_cmd.handbrake = HandBrakeStatus::PRESS;
         }
         if(msg->buttons[JoyButtons::RELEASE_BUTTON] > 0){
            comp_to_llc.vehicle_cmd.handbrake = HandBrakeStatus::RELEASE;
         }

         if(msg->axes[JoyAxes::DIRECTION_AXIS] > 0.0){
            comp_to_llc.vehicle_cmd.gear = GearStatus::DRIVE;
         }
         if(msg->axes[JoyAxes::DIRECTION_AXIS] < 0.0){
            comp_to_llc.vehicle_cmd.gear = GearStatus::REVERSE;
         }

         if(msg->buttons[JoyButtons::EMERGENCY_BUTTON] > last_emergency_press){
            comp_to_llc.emergency_cmd = !comp_to_llc.emergency_cmd;
         }

         last_emergency_press = msg->buttons[JoyButtons::EMERGENCY_BUTTON];
     }

     robione_ros2_driver::msg::VehicleStatus SerialConverter::convert_llc_to_vehicle_status(const serial_interface::LlcToComp &llc_to_comp) {
        robione_ros2_driver::msg::VehicleStatus vehicle_status{};
        vehicle_status.stamp = rclcpp::Clock().now();
        vehicle_status.message_count = llc_to_comp.data_counter;

        vehicle_status.throttle = llc_to_comp.control_info.throttle;
        vehicle_status.velocity = llc_to_comp.control_info.velocity;
        vehicle_status.steer_angle = llc_to_comp.control_info.steering_angle;

        vehicle_status.handbrake = llc_to_comp.vehicle_info.handbrake;
        vehicle_status.gear = llc_to_comp.vehicle_info.gear;
        vehicle_status.mode = llc_to_comp.vehicle_info.mode;
        vehicle_status.head_light = llc_to_comp.vehicle_info.head_light;
        vehicle_status.blinkers = llc_to_comp.vehicle_info.blinkers;

        vehicle_status.intervention = llc_to_comp.state_info.intervention;
        vehicle_status.ready = llc_to_comp.state_info.ready;
        vehicle_status.emergency = llc_to_comp.emergency;

        return vehicle_status;
     }

     robione_ros2_driver::msg::VehicleInfo SerialConverter::convert_debug_to_vehicle_info(const serial_interface::DebugMsgSlow_t &debug_slow) {
        robione_ros2_driver::msg::VehicleInfo vehicle_info{};
        vehicle_info.stamp = rclcpp::Clock().now();
        vehicle_info.battery_voltage = debug_slow.voltages.batvoltage;
        vehicle_info.supply_voltage = debug_slow.voltages.supplyVoltage;
        vehicle_info.usb_comm_period = debug_slow.periods.comp_period;
        vehicle_info.brake_status_period = debug_slow.periods.brakeStatusPeriod;
        vehicle_info.brake_encoder_period = debug_slow.periods.brakeEncoderPeriod;
        vehicle_info.rf_controller_period = debug_slow.periods.rfControllerPeriod;
        vehicle_info.steer_controller_period = debug_slow.periods.steerControllerPeriod;
        vehicle_info.bat_controller_period = debug_slow.periods.batControllerPeriod;
        vehicle_info.bms_period = debug_slow.periods.bmsPeriod;
        vehicle_info.charge_status = debug_slow.charge_status;
        vehicle_info.soc = debug_slow.SoC;
        vehicle_info.battery_temperature1 = debug_slow.bat_temp1;
        vehicle_info.battery_temperature2 = debug_slow.bat_temp2;
        return vehicle_info;
     }

     robione_ros2_driver::msg::SystemStatus SerialConverter::convert_debug_to_system_status(const serial_interface::DebugMsgFast_t &debug_fast){
         robione_ros2_driver::msg::SystemStatus system_status{};

         system_status.stamp = rclcpp::Clock().now();
         system_status.vehicle_current = debug_fast.currents.sysCurrent;
         system_status.brake_left_current = debug_fast.currents.brakeLeftCurrent;
         system_status.brake_right_current = debug_fast.currents.brakeRightCurrent;
         system_status.sys_error_code = debug_fast.sysErrors;
         system_status.communicaton_error_code = debug_fast.sysCommErrors;

         return system_status;
     }
}


