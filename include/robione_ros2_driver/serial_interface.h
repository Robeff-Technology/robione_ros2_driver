//
// Created by elaydin on 9/13/23.
//

#ifndef STRUCTS_H
#define STRUCTS_H
#include <cstdint>
namespace serial_interface {

    enum MessageID:uint8_t {
        LLC_MSG_ID = 0x36U,
        COMP_TO_LLC_MSG_ID = 0x87U,
        DEBUG_MSG_FAST_ID  = 0x88U,
        DEBUG_MSG_SLOW_ID  =  0x89U,
        SET_STEERING_ALIGN_MSG_ID = 0x90U,
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
        uint8_t head_light;
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
        uint8_t blinkers;
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
    struct __attribute__((packed)) Currents_t {
        float sysCurrent;
        float brakeLeftCurrent;
        float brakeRightCurrent;
    };
    struct __attribute__((packed)) Voltages_t {
        float batvoltage;
        float supplyVoltage;
    };

    struct __attribute__((packed)) SysCommPeriods_t {
        float comp_period;
        float brakeStatusPeriod;
        float brakeEncoderPeriod;
        float rfControllerPeriod;
        float steerControllerPeriod;
        float bmsPeriod;
        float batControllerPeriod;
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
