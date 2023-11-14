# robione_ros2_driver
## Table of Contents
1. [Description](#description)
2. [Key Features](#key-features)
3. [Installation](#installation)
    - [Prerequisites](#prerequisites)
    - [Installation Steps](#installation-steps)
    - [Configuration](#configuration)
4. [Launching the Driver](#launching-the-driver)
5. [Configuration Parameters](#configuration-parameters)
    - [`topic_config`](#topic_config)
    - [`serial_config`](#serial_config)
    - [`commpro_config`](#commpro_config)
    - [`timeout_config`](#timeout_config)
    - [`robione_config`](#robione_config)
6. [Receive Messages](#receive-messages)
    - [`ControlCmd`](#controlcmd)
7. [Transmit Messages](#transmit-messages)
    - [`SystemStatus`](#systemstatus)
    - [`VehicleInfo`](#vehicleinfo)
    - [`VehicleStatus`](#vehiclestatus)
8. [Using Joy Controller with ROS 2(Optional)](#using-joy-controller-with-ros-2(Optional))
    - [Prerequisites](#joy-node-prerequisites)
    - [Installation of the `joy` package](#installation-of-the-joy-package)
    - [Button and Axes Configuration](#button-and-axes-configuration)
    - [Example Using with DualShock 4](#example-using-with-dualshock-4)
9. [Using Remote Controller with Radiolink T8FB Controller](#using-remote-controller-with-radiolink-t8fb-controller)
    - [T8FB Features](#t8fb-features)
    - [Buttons Functions](#buttons-functions)


## Description

The `robione_ros2_driver` is a powerful and flexible ROS2 driver designed for seamless integration with Robione mobile robots. This driver provides a comprehensive interface for controlling and communicating with Robione robots, enabling you to unlock the full potential of your robotic platform in ROS2-based applications.

#### Author: [Robeff Technology](https://www.robeff.com)
#### Maintainer : [Robeff Technology](mailto:support@robeff.com)
## Key Features

- **Plug-and-Play Compatibility:** Easily connect your Robione mobile robot to ROS2 using this driver. It supports usb communication, making it a versatile solution for different robot models.

- **Real-time Feedback:** Receive real-time feedback from the robot's sensors and actuators, enabling you to monitor and control the robot's state, including position, velocity, and sensor data.

- **ROS2 Ecosystem Integration:** Seamlessly integrate the driver into the ROS2 ecosystem, allowing you to leverage existing ROS2 packages, tools, and libraries for developing your robot applications.

- **Customization:** Tailor the driver to your specific robot configuration and requirements. Customize control interfaces, sensor configurations, and behavior parameters to suit your application needs.

- **Robione API Support:** Interact with the Robione robot using the provided ROS2 API, allowing you to send commands, receive status updates, and access robot-specific functionality.

- **Open Source:** This driver is open-source, encouraging collaboration and community contributions to improve its functionality and compatibility with various Robione robot models.

Whether you're developing autonomous navigation algorithms, building robotic applications for research, or simply exploring the capabilities of your Robione mobile robot, the `robione_ros2_driver` is your gateway to a seamless and efficient ROS2 integration experience.

# Installation

This guide will walk you through the installation process for the `robione_ros2_driver`. Before you begin, ensure you have ROS2 Humble installed and the `sensor_msgs` package available. Additionally, make sure you have the necessary permissions to install packages and access the required dependencies.

## Prerequisites

- `ROS2 Humble`: Make sure you have a working ROS2 Humble installation. If you don't have ROS2 Humble installed, you can follow the official installation instructions: [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html).
- `sensor_msgs` package installed. You can install it using the following command:

    ```bash
    sudo apt install ros-<ros2-distro>-sensor-msgs
    ```

## Installation Steps

1. Clone the `robione_ros2_driver` repository to your ROS2 workspace:

    ```bash
    cd /path/to/your/ros2/workspace/src
    git clone https://github.com/Robeff-Technology/robione_ros2_driver.git
    ```

2. Build the `robione_ros2_driver` package:

    ```bash
    cd /path/to/your/ros2/workspace
    colcon build
    ```

3. Source your ROS2 workspace to make the `robione_ros2_driver` package available:

    ```bash
    source /path/to/your/ros2/workspace/install/setup.bash
    ```

## Configuration

The `robione_ros2_driver` package may require configuration to match your specific robot setup. You can modify the configuration parameters by editing the provided configuration file.

1. Locate the configuration file. It is typically located at:

    ```
    /path/to/your/ros2/workspace/src/robione_ros2_driver/config/robione_ros2_driver.param.yaml
    ```

2. Edit the configuration file according to your robot's specifications and requirements.

## Launching the Driver

You can launch the `robione_ros2_driver` using the `ros2 launch` command. The driver launch file is provided with the package:

```bash
ros2 launch robione_ros2_driver robione_ros2_driver.launch.py
```

## Configuration Parameters

This document provides an explanation of the configuration parameters used in the `robione_ros2_driver` for controlling a Robione mobile robot. You can customize these parameters in the YAML configuration file to tailor the driver's behavior to your specific requirements.

## `topic_config`

- `control_cmd_topic`: This parameter specifies the ROS topic where control commands are received. By default, it is set to `/robione/control_cmd`. You can modify this topic to match your desired control command input.

- `joystick_topic`: This parameter specifies the ROS topic for receiving joystick input, if applicable. The default value is `/joy`. Adjust this topic if you have a different joystick input topic.

## `serial_config`

- `port`: The `port` parameter determines the serial port used for communication with the Robione robot. The default value is `/dev/ttyUSB0`. Ensure that it matches the correct serial port for your hardware.

- `baudrate`: This parameter sets the baud rate for the serial communication. The default value is `115200`. Make sure it matches the baud rate used by your robot's hardware interface.

## `commpro_config`

- `interface_id`: This parameter specifies the interface ID for communication with the robot. The default value is `1`. It may be specific to your robot's communication protocol.

- `vcu_id`: The `vcu_id` parameter defines the VCU (Vehicle Control Unit) ID for communication. The default value is `0`. Ensure it aligns with your robot's setup.

## `timeout_config`

- `control_cmd_timeout`: This parameter sets the timeout duration for waiting for control command messages. The default value is `0.5` seconds. Adjust this timeout to match your desired control responsiveness.

- `joystick_timeout`: The `joystick_timeout` parameter determines the timeout for joystick input. It is set to `5.0` seconds by default. Modify this timeout to suit your joystick input requirements.

## `robione_config`

- `set_steer_rate`: This parameter is a boolean flag (`true` or `false`) that indicates whether the driver should set the steering rate. It is set to `false` by default.

- `use_joystick`: The `use_joystick` parameter is also a boolean flag (`true` or `false`) that specifies whether the joystick should be used for control. The default value is `false`, meaning the joystick is not used for control.

Feel free to adjust these parameters in your configuration file to match the specifics of your Robione mobile robot and your control preferences.


# Receive Messages
## `ControlCmd`

| Field        | Type       | Description                        | Range/Constants                            |
|--------------|------------|------------------------------------|--------------------------------------------|
| `throttle`   | `float32`  | Throttle command                   | `0.0` to `1.0`                            |
| `brake`      | `float32`  | Brake command                      | `0.0` to `1.0`                            |
| `steer`      | `float32`  | Steering command                   | `-1.0` (full right) to `1.0` (full left)  |
| `steer_rate` | `float32`  | Steering rate (degrees per second) | `43` deg/s to `545` deg/s                 |
| `handbrake`  | `uint8`    | Handbrake state                    | `HANDBRAKE_ON` or `HANDBRAKE_OFF`         |
| `gear`       | `uint8`    | Gear selection                     | `GEAR_NEUTRAL`, `GEAR_DRIVE`, `GEAR_REVERSE` |
| `emergency`  | `uint8`    | Emergency state                    | `EMERGENCY_OFF` or `EMERGENCY_ON`         |
| `blinkers`   | `uint8`    | Blinker state                      | `NO_COMMAND`, `LEFT_TURN_SIGNAL`, `RIGHT_TURN_SIGNAL` |
| `head_light` | `uint8`    | Headlight state                    | `HEAD_LIGHT_OFF` or `HEAD_LIGHT_ON`       |

:warning:**WARNING**: If driver sends brake command to robot, it will be applied immediately. So, you should send throttle command to robot when brake command equals 0.

:collision:**ERROR**: If values outside of this range cause emergency to robione.
## Field Descriptions

- `throttle`: Represents the throttle command, ranging from `0.0` (no throttle) to `1.0` (full throttle).

- `brake`: Represents the brake command, ranging from `0.0` (no brake) to `1.0` (full brake).

- `steer`: Represents the steering command, ranging from `-1.0` (full right) to `1.0` (full left).

- `steer_rate`: Represents the rate of change of the steering angle in degrees per second, ranging from `43` deg/s to `545` deg/s.
## Constants

- `handbrake`: A constant with values `HANDBRAKE_ON` (0) or `HANDBRAKE_OFF` (1) indicating the state of the handbrake.

- `gear`: A constant with values `GEAR_NEUTRAL` (0), `GEAR_DRIVE` (1), or `GEAR_REVERSE` (2) representing the selected gear.

- `emergency`: A constant with values `EMERGENCY_OFF` (0) or `EMERGENCY_ON` (1) indicating the emergency state.

- `blinkers`  : Represents the blinker command, ranging from `NO_COMMAND` (no blinker) to `LEFT_TURN_SIGNAL` (left blinker) and `RIGHT_TURN_SIGNAL` (right blinker).

- `head_light`: Represents the headlight command, ranging from `HEAD_LIGHT_OFF` (headlight off) to `HEAD_LIGHT_ON` (headlight on).


# Transmit Messages
## `SystemStatus`
The `SystemStatus` message provides information about the system's status and errors.

| Field                   | Type        | Description                                      | Bit Definitions                                 |
|-------------------------|-------------|--------------------------------------------------|-------------------------------------------------|
| `timestamp`             | `Time`      | Timestamp                                        | N/A                                             |
| `vehicle_current`       | `float32`   | Vehicle current in Amperes (A)                  | N/A                                             |
| `brake_left_current`    | `float32`   | Left brake current in Amperes (A)               | N/A                                             |
| `brake_right_current`   | `float32`   | Right brake current in Amperes (A)              | N/A                                             |
| `sys_error_code`        | `uint16`    | System error code                                | See bit definitions below                       |
| `communication_error_code`| `uint16`   | Communication error code                         | See bit definitions below                       |

### Bit Definitions
- `sys_error_code`: System error code, where individual bits have specific meanings:
    - Bit 0: Battery voltage error
    - Bit 1: Supply voltage error
    - Bit 2: Controller none

- `communication_error_code`: Communication error code, where individual bits have specific meanings:
    - Bit 0: USB communication error
    - Bit 1: RF controller communication error
    - Bit 2: Brake controller communication error
    - Bit 3: Steering controller communication error
    - Bit 4: BMS communication error
## `VehicleInfo`

The `VehicleInfo` message provides information about the system's status and errors.

| Field                   | Type        | Description                                                 |
|-------------------------|-------------|-------------------------------------------------------------|
| `timestamp`             | `Time`      | Timestamp                                                   |
| `battery_voltage`       | `float32`   | Battery voltage in Volts (V)                                | 
| `supply_voltage`        | `float32`   | Supply voltage in Volts (V)                                 | 
| `usb_comm_period`       | `float32`   | USB communication period in Hertz (Hz)                      | 
| `brake_status_period`   | `float32`   | Brake status update period in Hertz (Hz)                    |
| `brake_encoder_period`  | `float32`   | Brake encoder update period in Hertz (Hz)                   | 
| `rf_controller_period`  | `float32`   | RF controller update period in Hertz (Hz)                   | 
| `steer_controller_period`| `float32`  | Steer controller update period in Hertz (Hz)                | 
| `bms_period`            | `float32`   | Battery Management System (BMS) update period in Hertz (Hz) |
| `bat_controller_period` | `float32`   | Battery controller update period in Hertz (Hz)              |
| `battery_temperature1`  | `float32`   | Battery temperature sensor 1 in degrees Celsius (°C)        | 
| `battery_temperature2`  | `float32`   | Battery temperature sensor 2 in degrees Celsius (°C)        | 
| `charge_status`         | `uint8`     | Robot's charge status   (0->not charged 1->charging)        |
| `soc`         | `uint8`     | state of charge  (%)        | 

## `VehicleStatus`

| Field             | Type       | Description                           | Range/Constants                             |
|-------------------|------------|---------------------------------------|---------------------------------------------|
| `builtin_interfaces/Time` | `Time` | Timestamp                             | N/A                                         |
| `message_count`   | `uint32`   | Message count                         | N/A                                         |
| `velocity`        | `float32`  | Vehicle velocity in meters per second (m/s) | N/A                                          |
| `steer_angle`     | `float32`  | Steering angle in degrees              | -220 to 220                                 |
| `throttle`        | `float32`  | Throttle command                      | `0.0` to `1.0`                              |
| `handbrake`       | `uint8`    | Handbrake state                        | `HANDBRAKE_ON` or `HANDBRAKE_OFF`           | 
| `gear`            | `uint8`    | Gear selection                         | `GEAR_NEUTRAL`, `GEAR_DRIVE`, `GEAR_REVERSE` |
|`blinkers`         | `uint8`    | Blinker state                          | `NO_COMMAND`, `LEFT_TURN_SIGNAL`, `RIGHT_TURN_SIGNAL` |
| `head_light`      | `uint8`    | Headlight state                        | `HEAD_LIGHT_OFF` or `HEAD_LIGHT_ON`         |
| `intervention`    | `uint8`    | Intervention state                    | `NO_INTERVENTION` or `INTERVENTION`         | 
| `ready`           | `uint8`    | Readiness state                       | `NOT_READY` or `READY`                      | 
| `mode`            | `uint8`    | Steering mode                         | `SET_STEER_RATE` or `NO_STEER_RATE`         |
| `emergency`       | `uint8`    | Emergency state                       | `EMERGENCY_OFF` or `EMERGENCY_ON`           |

# Using Joy Controller with ROS 2(Optional)
In this guide, we'll walk you through the steps to set up and use a DualShock controller with ROS 2 to control your Robione.>
## Prerequisites 

Before you begin, make sure you have the following:
- A controller compatible with ros2/joy (e.g., DualShock 3 or DualShock 4).
- A computer running ROS 2 (e.g., Ubuntu with ROS 2 installed).

## Steps

### 1. Install `joy` Package

The first step is to install the `joy` package for ROS 2, which provides drivers and utilities for joystick devices.
```bash
sudo apt-get install ros-<ros2-distro>-joy
```
### 2. Usage
Before running the joy node, you have to source your ros2 workspace.
```bash
source /path/to/your/ros2/workspace/install/setup.bash
```
Run the joy node to start reading joystick data. Make sure your joystick or game controller is connected to your computer.
```bash
ros2 run joy joy_node
```
After running joy_node you can see the output of the joystick data by running the following command:
```bash
ros2 topic echo /joy
```
After this step, you can use the joystick data to control your robot.Please don't forget to enter /joy topic name in the configuration file's `joystick_topic` section.

Before launching the robione_ros2_driver, you have to change the `use_joystick` parameter to `true` in the configuration file. After this configuration, you can launch the robione_ros2_driver.
```bash
ros2 launch robione_ros2_driver robione_ros2_driver.launch.py
```


## Button and Axes Configuration

In this section, we'll explain the button and axes configurations for the ROS 2 joy package.

### Axes Configuration

| Axis Number | Function        | Values          |
| ----------- | --------------- | --------------- |
| Axis 0      | Steering        | 1 (left) to -1 (right) |
| Axis 4      | Throttle/Brake  | 0 to 1 (throttle), 0 to -1 (brake) |
| Axis 7      | Direction       | 1.0 (forward), -1.0 (reverse) |

#### Axis Functions

- **Steering**: Controls the steering of the vehicle, with values ranging from 1 (left) to -1 (right).
- **Throttle/Brake**: Controls acceleration and braking, with values ranging from 0 (brake) to 1 (throttle) and 0 (brake) to -1 (reverse).
- **Direction**: Sets the direction of movement, with 1.0 for forward and -1.0 for reverse.

### Buttons Configuration

| Button Number | Function          |
| ------------- | ----------------- |
| Button 3      | Emergency Stop    |
| Button 4      | Release           |
| Button 5      | Handbrake         |

#### Button Functions

- **Emergency Stop**: Immediately halts all vehicle operations.
- **Release**: Releases the brakes.
- **Handbrake**: Engages the handbrake to prevent vehicle movement.

## Example Using with DualShock 4
![image](https://github.com/Robeff-Technology/robione_ros2_driver/assets/63052608/36d05af3-3e3a-46c0-9379-7a9a3b3d57db)


# Using Remote Controller with Radiolink T8FB Controller
## T8FB Features

## **Remote**

-One two-way switch, one three-way switch, two VR switches, four trimmers, two sticks.

-Universal JST battery connector supports multiple batteries, include 4pcs AA batteries or 2S to 4S LiPo battery.

-Default low voltage alarm to 5.0V automatically after detection.

### Buttons Functions

#### SwA

It is used for emergencies. Emergency is activated when the button is in the forward direction and when it is in reverse, it is in drive mode. When the emergency is activated, the vehicle brakes.

#### SwB

It is used to determine the direction of the vehicle and to apply the handbrake. In the up position, the vehicle direction is forward, in the middle position, the vehicle is in reverse mode. At the bottom, the handbrake of the vehicle is applied.

#### VrB

It is used to change the controller. When pushed forward, the vehicle goes into autonomous mode and when pulled back, the control passes to the remote.

#### Rudder(Left Stick)

It is used to adjust the steering angle.

#### Aileron(Right Stick)

It is used to drive the vehicle and to brake gradually. When pushed forward, the vehicle accelerates. When retracted, the brake is applied gradually.

![image](https://github.com/Robeff-Technology/robione_ros2_driver/assets/122979266/db1c5d75-9ed6-4bf2-8003-9b89e4090bb7)


### Modes

#### Autonomous Mode

When the vehicle is put into this mode, the vehicle is driven autonomously with the commands coming from the computer.

#### Remote Mode

When this mode is switched on, the computer is disabled and control passes to the remote control, and the vehicle is controlled manually.

### Warnings

:warning: If the battery of the remote control runs out, the remote makes a beep sound. In this case, the remote control battery needs to be replaced or charged for 10 minutes.

:warning: It is not recommended to use the remote control at a distance of more than 15 meters.
