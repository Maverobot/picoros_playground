# PicoROS Playground

This repository is a small playground for experimenting with the [PicoROS](https://github.com/Pico-ROS/Pico-ROS-software) framework on an ESP32-S3 microcontroller.

The project structure was initially extracted from the ROSCon 2025 Workshop
[embedded-mobile-base](https://github.com/ros-controls/roscon2025_control_workshop/tree/master/embedded-mobile-base)
repository, and then adapted to host multiple small examples (starting with a joystick teleop node).

## What this project provides

- A PlatformIO-based ESP32-S3 project using:
  - [PicoROS](https://github.com/ros-controls/picoros) as the ROS 2 client library.
  - [Zenoh](https://zenoh.io/) via `rmw_zenoh_cpp` as the ROS 2 middleware on the host.
- An example `joystick_teleop` node that:
  - Reads a joystick via ADC and a button via GPIO.
  - Publishes `geometry_msgs/msg/TwistStamped` messages on the `/cmd_vel` topic.
  - Uses WiFi + SNTP to obtain a proper wall-clock time on the ESP32.
- Reusable patterns taken from the `embedded_mobile_base` example:
  - LED initialization and status indication.
  - PicoROS node and publisher setup.
  - Serial link configuration between ESP32 and Zenoh router.

This is intended as a sandbox to try out, hack on, and extend PicoROS-based firmware, not a polished product.

## Repository layout (relevant parts)

- `src/main.cpp`
  Entry point that selects which example to run (currently `joystick_teleop`).

- `src/joystick_teleop.h`
  Joystick teleoperation example:
  - Initializes LEDs, NVS, WiFi, and SNTP.
  - Connects to a Zenoh router over serial using PicoROS.
  - Publishes `/cmd_vel` commands based on joystick input.

- `src/wifi_time.h`
  Helper utilities for:
  - Connecting to a WiFi network as a station.
  - Initializing SNTP and waiting until the system time is set.

Other source and configuration files largely follow the structure of the original `embedded-mobile-base` workshop project.
