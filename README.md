# ouster_http_node

A ROS2 package for integrating timestamp information from a PPS (Pulse Per Second) signal into ROS2 messages, enabling precise time synchronization.

## Overview

This package connects to a device exposing timestamp and PPS signal information via HTTP, parses the data, and publishes it as a custom ROS2 message (`TimeStatus`).  
It is designed for applications where high-precision time synchronization is required, such as robotics, autonomous systems, or distributed sensor fusion.

The node extracts key timing parameters including:
- PPS lock status
- Timestamp from the sensor
- Diagnostics counters
- Synchronization options

This information can then be used by downstream nodes for time alignment, logging, or synchronization with other systems.

## Features

- HTTP polling of timestamp and PPS information.
- Safe JSON parsing with robust error handling.
- Publishes a custom ROS2 message with detailed timing data.
- Designed for easy integration with ROS2-based systems.

## Installation

Clone this repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/anthonytec2/ouster_http_node.git
cd ~/ros2_ws
colcon build --packages-select ouster_http_node
source install/setup.bash
