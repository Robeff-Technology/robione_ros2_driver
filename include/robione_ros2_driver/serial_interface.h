//
// Created by elaydin on 9/13/23.
//

#ifndef STRUCTS_H
#define STRUCTS_H
#include <cstdint>
namespace serial_interface {

    enum MessageID:uint8_t {
        LLC_MSG_ID = 0x36U,
        COMP_TO_LLC_MSG_ID = 0x31U,
        COMP_TO_SRV_MSG_ID = 0x52U,
        LLC_TO_SRV_MSG_ID  = 0x53U,
        SRV_TO_LLC_MSG_ID  = 0x77U,
        DEBUG_MSG_FAST_ID  = 0x88U,
        DEBUG_MSG_SLOW_ID  =  0x89U,
    };

    struct __attribute__((packed)) LongitudinalCmd {
        float set_throttle;
        float set_brake;
    };

    struct __attribute__((packed))  LateralCmd {
        float set_steering_angle;
        float set_steering_angle_rate;
    };

    struct __attribute__((packed)) VehicleCmd {
        uint8_t gear;
        uint8_t handbrake;
        uint8_t blinkers;
        uint8_t mode;
    };

    struct __attribute__((packed)) ControlInfo {
        float velocity;
        float steering_angle;
        float throttle;
    };

    struct __attribute__((packed)) VehicleInfo {
        uint8_t gear;
        uint8_t mode;
        uint8_t handbrake;
        uint8_t head_light;
        uint8_t turn_indicators;
    };

    struct __attribute__((packed)) StateInfo {
        uint8_t intervention;
        uint8_t ready;
    };

    struct __attribute__((packed)) CompToLlc {
        uint32_t msg_counter;
        LongitudinalCmd long_cmd;
        LateralCmd lat_cmd;
        VehicleCmd vehicle_cmd;
        uint8_t emergency_cmd;

        void fill_zero(){
            long_cmd.set_throttle = 0;
            long_cmd.set_brake = 0;
            lat_cmd.set_steering_angle = 0;
            lat_cmd.set_steering_angle_rate = 0;
            vehicle_cmd.gear = 0;
            vehicle_cmd.handbrake = 0;
            vehicle_cmd.blinkers = 0;
            vehicle_cmd.mode = 0;
            emergency_cmd = 0;
        }
    };

    struct __attribute__((packed)) LlcToComp {
        uint32_t data_counter;
        ControlInfo control_info;
        VehicleInfo vehicle_info;
        StateInfo state_info;
        uint8_t emergency;
    };

    struct __attribute__((packed)) LogData {
        uint32_t system_time;
        uint16_t bat_volt;
        uint16_t applied_dac;
        uint8_t joystick_ref_angle;
        int16_t vcu_steer_ref_angle;
        int16_t current;
        uint8_t soc;
        uint8_t set_throttle;
        uint16_t act_rpm;
        int16_t steering_act_angle;
        int16_t steering_ref_angle;
        uint16_t steering_current;
        uint8_t mode;
        uint16_t min_cell_voltage;
        int16_t acceleration;
    };

    struct __attribute__((packed)) SrvToComp {
        float steering; // -1 - 1 right-axis
        float throttle; // -1 - 1 left-axis
        uint8_t handbrake; //0:Applied 1:Released
        uint8_t direction; //0:Forward 1:Backward
        uint8_t emergency; //0 :Normal, 1:emergency
        uint8_t autoware_control_mode; // 1:Enable 4:Disable
        uint8_t operation_mode; // 1:Stop 2:Autonomous 3:Local 4:Remote
    };

    struct __attribute__((packed)) Currents_t {
        float sysCurrent;
        float brakeLeftCurrent;
        float brakeRightCurrent;
    };
    struct __attribute__((packed)) Voltages_t {
        float batvoltage;
        float supplyVoltage;
    };

    union __attribute__((packed)) BrakeParams{
        uint8_t raw[8];
        struct __attribute__((packed)) {
            uint16_t state_b1                              : 3;
            uint16_t state_b2                              : 3;
            uint16_t mode_b1                               : 2;
            uint16_t mode_b2                               : 2;
            uint16_t calib_status_b1                       : 1;
            uint16_t calib_status_b2                       : 1;
            uint16_t reserved                              : 4;
        }bits;
        uint16_t all;
    };

    struct __attribute__((packed)) BrakeMsg_t {
        int16_t angle1;
        int16_t angle2;
        int16_t averageAngle;
        uint16_t params;
        float current1;
        float current2;
        int32_t encoderVal1;
        int32_t encoderVal2;
    };


    struct __attribute__((packed)) SysCommPeriods_t {
        float comp_period;
        float brakeStatusPeriod;
        float brakeEncoderPeriod;
        float rfControllerPeriod;
        float steerControllerPeriod;
        float bmsPeriod;
    };

    struct __attribute__((packed)) DebugMsgFast_t {
        Currents_t currents;
        uint16_t sysErrors;
        uint16_t sysCommErrors;
    };

    struct __attribute__((packed)) DebugMsgSlow_t {
        uint8_t charge_status;
        uint8_t SoC;
        Voltages_t voltages;
        SysCommPeriods_t periods;
        float bat_temp1;
        float bat_temp2;
    };
};


#endif //STRUCTS_H
