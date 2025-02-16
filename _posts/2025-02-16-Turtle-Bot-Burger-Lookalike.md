---
theme: minima
title: Building a ROS2 Humble Mobile Robot with Docker
date: 2025-02-16
categories: [ROS2, Robotics, Docker, Embedded Systems]
tags: [ROS2 Humble, Raspberry Pi, ESP32, RPLidar, SLAM, Navigation, Docker]
---

## Introduction

This blog post details the process of building a ROS2 Humble-based mobile robot, similar in design to the TurtleBot Burger, and deploying it using Docker for improved portability and reproducibility. The robot utilizes a Raspberry Pi 4 for high-level functions, an ESP32 for low-level motor control, an RPLidar A1 for 2D mapping and localization, and the ROS2 Navigation2 stack for autonomous navigation.

## Hardware Components

*   **Raspberry Pi 4:** The central processing unit, running the ROS2 nodes for navigation, SLAM, and high-level control.
*   **ESP32:** A microcontroller used for low-level motor control and sensor data acquisition.
*   **L298N Motor Driver:**  An H-bridge motor driver used to control the DC motors.
*   **DC Motors (x2):**  Provide the robot's movement capabilities.
*   **RPLidar A1:** A 2D LiDAR sensor for mapping and localization.
*   **Robot Chassis/Platform:** A platform to house the electronics and motors.
*   **Power Supplies:** Separate power supplies for the Raspberry Pi and the motors.

## Software Stack

*   **ROS2 Humble:** The Robot Operating System 2, providing a framework for robot software development.
*   **SLAM Toolbox:** Used for simultaneous localization and mapping (SLAM) to create a map of the environment.
*   **Navigation2 (Nav2):** The ROS2 navigation stack, enabling autonomous path planning and obstacle avoidance.
*   **Docker:**  Containerization technology used to package the ROS2 environment and dependencies into a single, portable unit.
*   **ESP32 Firmware (Arduino IDE):** Custom firmware for the ESP32 to control the motors based on commands from the Raspberry Pi.

## System Architecture

The robot's software architecture consists of the following ROS2 nodes:

1.  **RPI-ESP Interface:** Handles communication between the Raspberry Pi and the ESP32, sending motor commands and receiving sensor data.
2.  **Lidar Sensor:** Processes data from the RPLidar A1, publishing LaserScan messages.
3.  **Localization and Mapping:**  Utilizes SLAM Toolbox to create a map of the environment and localize the robot within the map.
4.  **System Monitor:** Monitors the robot's health, sensor status, and system performance.
5.  **Motor Control:** Provides a web interface for manual control of the robot's movement.

## Hardware Setup

[wiring diagram here]

The hardware setup involves connecting the Raspberry Pi, ESP32, motor driver, motors, and RPLidar according to the following:

*   **Raspberry Pi:** Powered with a dedicated 5V power supply.  The RPLidar is connected via USB.
*   **ESP32:** Connected to the Raspberry Pi via UART serial communication.
*   **L298N Motor Driver:** Connected to the ESP32 GPIO pins for control signals and to the DC motors for power.
*   **RPLidar:** Connected to the Raspberry Pi via USB.

## Software Setup

### ROS2 Package Creation

The ROS2 package `my_robot` was created with the following structure:  
my_robot/  
├── CMakeLists.txt  
├── package.xml  
└── my_robot/  
&ensp;&ensp;&ensp;&ensp;&ensp;├── rpi_esp_interface.py  
&ensp;&ensp;&ensp;&ensp;&ensp;├── lidar_sensor.py  
&ensp;&ensp;&ensp;&ensp;&ensp;├── localization_mapping.py  
&ensp;&ensp;&ensp;&ensp;&ensp;├── system_monitor.py  
&ensp;&ensp;&ensp;&ensp;&ensp;└── motor_control.py  
├── setup.config  
└── setup.py

The `CMakeLists.txt` and `package.xml` files define the package's build configuration and dependencies, respectively. The `src/` directory contains the Python scripts for each ROS2 node.

### Node Descriptions

*   **`rpi_esp_interface.py`:**  Communicates with the ESP32 via serial, sending motor commands and receiving sensor data.
*   **`lidar_sensor.py`:** Reads data from the RPLidar A1 and publishes `LaserScan` messages on the `/scan` topic.
*   **`localization_mapping.py`:**  Uses SLAM Toolbox to perform SLAM, creating a map and localizing the robot.
*   **`system_monitor.py`:**  Monitors the robot's system status and publishes status messages.
*   **`motor_control.py`:** Provides a simple web interface for controlling the robot's movement (forward, backward, left, right, stop).

[Include code snippets of the key ROS2 nodes here.]

### Docker Configuration

A `Dockerfile` was created to package the ROS2 environment and dependencies into a Docker image.

[Include the contents of your Dockerfile here.]

The Docker image is built using the command:

```bash
docker build -t my_robot_image .
```

The Docker container is run with the command:

```bash
docker run -it --net=host --privileged -v /dev:/dev my_robot_image
```
*  `-it`: Runs the container in interactive mode (allows you to access the shell).

*  `--net=host`: Uses the host network. This is important for ROS2 to communicate properly, especially when using DDS for communication.

*  `--privileged`: Gives the container elevated privileges. This is often needed to access hardware devices like the RPLidar. Use with caution in production environments. Consider using more fine-grained capabilities if possible.

*  `-v /dev:/dev`: Mounts the host's /dev directory into the container. This is essential for the container to access serial ports (for the ESP32 and RPLidar).

*  `my_robot_image`: The name of the Docker image you built.

### Launch File

A launch.py file is used to launch all the ROS2 nodes simultaneously.

[Include the content of your launch.py file here.]

### Challenges and Solutions

**Serial Communication**: Getting the serial communication between the Raspberry Pi and ESP32 working reliably required careful configuration of the serial port settings and error handling.

**SLAM Toolbox Configuration**: Tuning the SLAM Toolbox parameters to achieve accurate mapping and localization was a challenging but essential task.

**Navigation2 Setup**: Configuring the Navigation2 stack to perform autonomous navigation required a deep understanding of the costmaps, planners, and controllers.

**Docker Permissions**: Granting the Docker container access to the serial port required the use of the --privileged flag and mounting the /dev directory.

### Future Work

Implement wheel odometry using wheel encoders.

Improve the accuracy of the SLAM map and Nav2 navigation.

Add more sensors (e.g., an IMU) to improve localization.

Develop a more sophisticated web interface for motor control and robot status monitoring.

Implement a more robust system monitoring and error reporting system.

### Conclusion

This project demonstrates the process of building a ROS2 Humble mobile robot and deploying it using Docker. The robot is capable of autonomous navigation, SLAM, and manual control. The use of Docker simplifies the deployment and management of the robot's software.

### Resources

ROS2 Humble Documentation

SLAM Toolbox Documentation

Navigation2 Documentation

Docker Documentation

