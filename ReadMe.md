**Understanding the Goal:**

To create a ROS2 Humble-based mobile robot resembling the TurtleBot Burger, with the following key features:

*   **Hardware:** Raspberry Pi 4, ESP32, L298N motor driver, DC motors, RPLidar A1.
*   **Functionality:** Autonomous navigation (Nav2), SLAM (SLAM Toolbox), differential drive, manual control via web interface, system monitoring.
*   **ROS2 Nodes:** RPI-ESP interface, Lidar processing, Localization/Mapping, System Monitoring, Motor Control.
*   **Dockerization:**  To simplify deployment, management, and reproducibility, you want to use Docker.

**Here's a detailed breakdown and step-by-step guide:**

**I. Hardware Setup and Connections:**

1.  **Components List:**

    *   Raspberry Pi 4 (with power supply, microSD card, and ethernet/WiFi)
    *   ESP32 development board (e.g., ESP32-DevKitC)
    *   L298N Motor Driver module
    *   2 x DC Motors (choose motors appropriate for your robot size and desired speed/torque)
    *   RPLidar A1 (and its USB cable)
    *   Power supply for motors (separate from the Pi's power supply) – Voltage depends on your motors.
    *   Breadboard and Jumper Wires
    *   Robot Chassis/Platform (similar to TurtleBot Burger or custom-built)
    *   Wheels and Encoders(Optional)
    *   USB-to-Serial adapter (for debugging ESP32)
    *   **Safety:** Fuse(s) for motor power circuit.
    *   Voltage Regulator (if needed to step down voltage for the ESP32 or other components)

2.  **Wiring Diagram (Important):**  This is a *high-level* outline.  Double-check the pinouts of your specific components.

    *   **Raspberry Pi 4:**
        *   Power: 5V power supply
        *   Ethernet/WiFi:  For network connectivity.
        *   USB:
            *   RPLidar A1 USB connection
            *   USB-to-Serial adapter (if needed for ESP32 programming)
        *   GPIO:
            *   UART (Serial) or I2C connection to ESP32 (for communication). Choose one based on your preference and speed requirements.
            *   GPIOs for controlling the L298N Motor Driver (if not using the ESP32 for direct motor control - *not recommended*)
    *   **ESP32:**
        *   Power: 3.3V (regulated from either the Pi or a separate regulator)
        *   UART/I2C: Connected to Raspberry Pi's corresponding pins.
        *   GPIO:
            *   Motor control signals to L298N
            *   (Optional) Encoder inputs (if you add wheel encoders)
    *   **L298N Motor Driver:**
        *   Power:  Connect to your motor power supply.
        *   Motor Outputs: Connect to your DC motors.
        *   Control Inputs: Connect to ESP32 GPIOs.
    *   **DC Motors:**
        *   Connected to L298N outputs.
        *   Mechanically connected to wheels.
    *   **RPLidar A1:**
        *   USB: Connect to Raspberry Pi USB port
        *   Power: Usually powered through USB, but check its specifications

3.  **Hardware Connection Steps (Detailed):**

    *   **Motor Driver to Motors:**  Connect the L298N motor driver outputs to the terminals of your DC motors. Pay attention to polarity to control the direction of rotation.
    *   **Motor Driver Control Signals:** Connect the control pins of the L298N (IN1, IN2, IN3, IN4, Enable pins) to the GPIO pins of the ESP32.  The specific pins you choose depend on your ESP32 code.
    *   **ESP32 to Raspberry Pi:** Connect the UART (TX, RX) or I2C (SDA, SCL) pins of the ESP32 to the corresponding pins on the Raspberry Pi.  You'll also need a common ground connection.
    *   **RPLidar to Raspberry Pi:** Connect the RPLidar A1 to a USB port on the Raspberry Pi.
    *   **Power Connections:**  Double-check all your power connections and ensure that you are using the correct voltage levels for each component. **Important: Use a separate power supply for the motors and the Raspberry Pi.**

**II. Software Setup:**

1.  **Raspberry Pi Setup:**

    *   Install the latest version of Raspberry Pi OS (64-bit Bookworm is recommended, but make sure it supports ROS2 Humble well).
    *   Enable SSH so you can access the Pi remotely.
    *   Update the system: `sudo apt update && sudo apt upgrade`
    *   **Install ROS2 Humble:**  Follow the official ROS2 Humble installation instructions for Debian packages:  [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)  *Important: Complete the "Setting up your environment" section!*

2.  **ESP32 Firmware (Arduino IDE):**

    *   Install the Arduino IDE: [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)
    *   Install the ESP32 board support in the Arduino IDE:  Follow these instructions: [https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide/](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide/)
    *   Write the ESP32 firmware to:
        *   Receive commands from the Raspberry Pi (via Serial/UART or I2C).
        *   Control the L298N motor driver based on the received commands.
        *   (Optional) Read encoder values (if you add encoders).
        *   Send motor status/encoder data back to the Raspberry Pi.

    *   **Example ESP32 Firmware Snippet (Conceptual - using Serial):**

    ```arduino
    // ESP32 Motor Control Firmware (Conceptual)
    #define IN1 2  // Example pin
    #define IN2 4
    #define IN3 5
    #define IN4 18

    void setup() {
      Serial.begin(115200);
      pinMode(IN1, OUTPUT);
      pinMode(IN2, OUTPUT);
      pinMode(IN3, OUTPUT);
      pinMode(IN4, OUTPUT);
    }

    void loop() {
      if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim(); // Remove whitespace

        if (command == "forward") {
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          digitalWrite(IN3, HIGH);
          digitalWrite(IN4, LOW);
        } else if (command == "backward") {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, HIGH);
          digitalWrite(IN3, LOW);
          digitalWrite(IN4, HIGH);
        } else if (command == "stop") {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, LOW);
          digitalWrite(IN3, LOW);
          digitalWrite(IN4, LOW);
        }

        // ... other commands
      }
    }
    ```

    *   **Upload the firmware to the ESP32.**

**III. ROS2 Package and Node Creation:**

1.  **Create a ROS2 Workspace:**

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

2.  **Create the ROS2 Package:**

    ```bash
    ros2 pkg create --build-type ament_cmake my_robot
    ```

    *   `my_robot` is the name of your package. Change it if you like.

3.  **Create the Nodes (Python Examples):**

    *   **`rpi_esp_interface.py`:**

    ```python
    # rpi_esp_interface.py
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String  # Or a custom message

    import serial  # If using Serial/UART

    class RPIESPInterface(Node):
        def __init__(self):
            super().__init__('rpi_esp_interface')
            self.publisher_ = self.create_publisher(String, 'esp_data', 10)  # Example
            self.subscriber_ = self.create_subscription(String, 'motor_command', self.command_callback, 10)  # Example

            # Configure Serial Port (adjust as needed)
            try:
                self.ser = serial.Serial('/dev/ttyUSB0', 115200) #Find what the esp32 is using. can use (ls /dev/tty*)
                self.get_logger().info("Serial port connected")
            except serial.SerialException as e:
                self.get_logger().error(f"Error opening serial port: {e}")
                self.ser = None

        def command_callback(self, msg):
            command = msg.data
            self.get_logger().info(f"Received command: {command}")
            if self.ser:
                self.ser.write((command + '\n').encode('utf-8'))  # Send command to ESP32

        def spin(self):
            rclpy.spin(self)
            if self.ser:
                self.ser.close() # Close the serial port when ROS2 is shutdown.

    def main(args=None):
        rclpy.init(args=args)
        node = RPIESPInterface()
        node.spin()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

    *   **`lidar_sensor.py`:**

    ```python
    # lidar_sensor.py
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    import serial
    import rplidar

    class LidarSensor(Node):
        def __init__(self):
            super().__init__('lidar_sensor')
            self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)

            # Configure RPLidar
            PORT_NAME = '/dev/ttyUSB1'  # Adjust the port name for your RPLidar
            try:
                self.lidar = rplidar.RPLidar(PORT_NAME)
                self.lidar.start()

                self.scan_iterator = self.lidar.iter_scans()  # Create an iterator
                self.get_logger().info("RPLidar connected")
            except serial.SerialException as e:
                self.get_logger().error(f"Error connecting to RPLidar: {e}")
                self.lidar = None

            self.timer = self.create_timer(0.1, self.timer_callback)

        def timer_callback(self):
            if self.lidar:
                try:
                    scan = next(self.scan_iterator)
                    # Process the scan data and create a LaserScan message
                    msg = LaserScan()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "laser_frame" # frame_id is laser_frame
                    msg.angle_min = 0.0  # Adjust based on your lidar
                    msg.angle_max = 2 * 3.14159  # 360 degrees in radians
                    msg.angle_increment = 0.0174533 # 1 degree in radians
                    msg.range_min = 0.15  # Minimum range of RPLidar A1 (0.15m)
                    msg.range_max = 6.0  # Maximum range of RPLidar A1 (6m)

                    # Fill ranges and intensities
                    msg.ranges = [reading[2] for reading in scan]  # Distance data
                    msg.intensities = [reading[1] for reading in scan] # signal quality data

                    self.publisher_.publish(msg)
                    self.get_logger().info('Publishing LiDAR data')

                except StopIteration:
                    self.scan_iterator = self.lidar.iter_scans()  # Reset the iterator

                except Exception as e:
                    self.get_logger().error(f"Error processing lidar data: {e}")


        def stop(self):
            if self.lidar:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()

    def main(args=None):
        rclpy.init(args=args)
        node = LidarSensor()
        rclpy.spin(node)
        node.stop()  # Stop the lidar when ROS2 is shutdown
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

    *   **`localization_mapping.py`:** (This will be more complex and require SLAM Toolbox configuration)

    ```python
    # localization_mapping.py
    import rclpy
    from rclpy.node import Node
    # ... Import SLAM Toolbox related stuff here ...

    class LocalizationMapping(Node):
        def __init__(self):
            super().__init__('localization_mapping')
            # ... Initialize SLAM Toolbox components ...
        # ... Implement SLAM and localization logic using SLAM Toolbox ...

    def main(args=None):
        rclpy.init(args=args)
        node = LocalizationMapping()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

    *   **`system_monitor.py`:**

    ```python
    # system_monitor.py
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SystemMonitor(Node):
        def __init__(self):
            super().__init__('system_monitor')
            self.publisher_ = self.create_publisher(String, 'system_status', 10)
            self.timer = self.create_timer(5.0, self.timer_callback) #runs every 5 sec

        def timer_callback(self):
            # Example: Check CPU usage, memory usage, ROS node status
            status_message = "System OK" # Replace with actual checks
            msg = String()
            msg.data = status_message
            self.publisher_.publish(msg)

    def main(args=None):
        rclpy.init(args=args)
        node = SystemMonitor()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

    *   **`motor_control.py`:** (Web interface integration will require a separate framework like Flask or Django)

    ```python
    # motor_control.py
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class MotorControl(Node):
        def __init__(self):
            super().__init__('motor_control')
            self.publisher_ = self.create_publisher(String, 'motor_command', 10)

        def move_forward(self):
            msg = String()
            msg.data = "forward"
            self.publisher_.publish(msg)
            self.get_logger().info('Moving forward')

        def move_backward(self):
            msg = String()
            msg.data = "backward"
            self.publisher_.publish(msg)
            self.get_logger().info('Moving backward')

        def move_left(self):
            msg = String()
            msg.data = "left"
            self.publisher_.publish(msg)
            self.get_logger().info('Turning left')

        def move_right(self):
            msg = String()
            msg.data = "right"
            self.publisher_.publish(msg)
            self.get_logger().info('Turning right')

        def stop(self):
            msg = String()
            msg.data = "stop"
            self.publisher_.publish(msg)
            self.get_logger().info('Stopping')

        # The node would also include a web server integration (using Flask, etc.)
        # to receive commands from a web interface.

    def main(args=None):
        rclpy.init(args=args)
        node = MotorControl()
        # Example usage
        node.move_forward() # Example command
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

4.  **`CMakeLists.txt`:**  Edit the `CMakeLists.txt` file in your `my_robot` package to include your Python nodes.  Add the following to the `CMakeLists.txt`:

```cmake
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(PROGRAMS
  src/rpi_esp_interface.py
  src/lidar_sensor.py
  src/localization_mapping.py
  src/system_monitor.py
  src/motor_control.py
  DESTINATION lib/${PROJECT_NAME}
)
```

5.  **`package.xml`:**  Edit the `package.xml` file to include dependencies:

```xml
  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <!-- Add SLAM Toolbox dependencies if you use them directly in the package -->
  <exec_depend>ros2launch</exec_depend>
```

6.  **Build the Package:**

    ```bash
    cd ~/ros2_ws
    colcon build
    . install/setup.bash
    ```

7.  **Run the Nodes (Example):**

    ```bash
    ros2 run my_robot rpi_esp_interface.py
    ros2 run my_robot lidar_sensor.py
    ros2 run my_robot localization_mapping.py
    ros2 run my_robot system_monitor.py
    ros2 run my_robot motor_control.py
    ```

**IV. Docker Setup**

1.  **Install Docker:** Follow the official Docker installation instructions for your operating system: [https://docs.docker.com/get-docker/](https://docs.docker.com/get-docker/)

2.  **Create a Dockerfile:** Create a `Dockerfile` in your `ros2_ws` directory (or a subdirectory if you prefer).

```dockerfile
# Use an official ROS2 Humble base image
FROM osrf/ros:humble-desktop

# Set the working directory inside the container
WORKDIR /app

# Copy the ROS2 workspace into the container
COPY ./src /app/src

# Install any additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-serial \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies using pip
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install rplidar

# Build the ROS2 workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source the ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /app/install/setup.bash" >> /root/.bashrc
ENV ROS_DOMAIN_ID=30

# Set the entrypoint to run the ROS2 nodes
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /app/install/setup.bash && ros2 launch my_robot robot.launch.py"] # Create a launch file

```

**Explanation of the Dockerfile:**

*   `FROM osrf/ros:humble-desktop`:  Uses the official ROS2 Humble image as the base.  The `-desktop` version includes tools like RViz, which are helpful for development.
*   `WORKDIR /app`:  Sets the working directory inside the container to `/app`.
*   `COPY ./src /app/src`: Copies your ROS2 package source code into the container.
*   `RUN apt-get update && apt-get install -y ...`: Installs dependencies required by your nodes (e.g., `python3-serial`, `ros-humble-slam-toolbox`).
*   `RUN python3 -m pip install rplidar`: Installs the rplidar Python package.
*   `RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install`: Builds your ROS2 workspace inside the container. The `--symlink-install` option speeds up development because changes to your Python code will be reflected without needing to rebuild the entire workspace.
*   `RUN echo "source ... " >> /root/.bashrc`:  Adds the ROS2 setup commands to the container's `bashrc` file so that ROS2 is properly configured when you run the container.
*   `ENV ROS_DOMAIN_ID=30`: Used for multiple robots.
*   `ENTRYPOINT ["/bin/bash", "-c", "source ... && ros2 launch my_robot robot.launch.py"]`: Defines the command that will be executed when the container starts.  This will source the ROS2 environment and then launch your ROS2 nodes.  **Important:** You'll need to create a launch file to start all your nodes together.

3. **Create a Launch file**
Create a launch file named `robot.launch.py` inside of `my_robot` package with the following code.
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='rpi_esp_interface.py',
            name='rpi_esp_interface'
        ),
        Node(
            package='my_robot',
            executable='lidar_sensor.py',
            name='lidar_sensor'
        ),
        Node(
            package='my_robot',
            executable='localization_mapping.py',
            name='localization_mapping'
        ),
        Node(
            package='my_robot',
            executable='system_monitor.py',
            name='system_monitor'
        ),
        Node(
            package='my_robot',
            executable='motor_control.py',
            name='motor_control',
            output='screen'
        ),
    ])
```

4.  **Build the Docker Image:**

    ```bash
    cd ~/ros2_ws
    docker build -t my_robot_image .
    ```

    *   `my_robot_image` is the name you're giving to your Docker image.  The `.` tells Docker to use the current directory as the build context.

5.  **Run the Docker Container:**

    ```bash
    docker run -it --net=host --privileged -v /dev:/dev my_robot_image
    ```

    *   `-it`:  Runs the container in interactive mode (allows you to access the shell).
    *   `--net=host`: Uses the host network.  This is important for ROS2 to communicate properly, especially when using DDS for communication.
    *   `--privileged`:  Gives the container elevated privileges. This is *often* needed to access hardware devices like the RPLidar.  **Use with caution in production environments.**  Consider using more fine-grained capabilities if possible.
    *   `-v /dev:/dev`: Mounts the host's `/dev` directory into the container.  This is *essential* for the container to access serial ports (for the ESP32 and RPLidar).
    *   `my_robot_image`: The name of the Docker image you built.

**V.  Next Steps and Considerations:**

1.  **SLAM Toolbox Configuration:**  This is a significant undertaking.  You'll need to:

    *   Install the `slam_toolbox` ROS2 package: `sudo apt install ros-humble-slam-toolbox`
    *   Create a SLAM Toolbox configuration file (YAML) that specifies parameters like:
        *   `odom_frame_id`: The frame ID of your odometry data (if you add wheel encoders).
        *   `map_frame`:  The name of the map frame.
        *   `base_frame`: The name of the robot's base frame.
        *   `scan_topic`: The name of the Lidar topic (`/scan`).
    *   Use the `slam_toolbox` node in your launch file, providing the path to your configuration file.
    *   Tune the SLAM Toolbox parameters for your specific robot and environment.  This will involve experimentation.

2.  **Navigation2 (Nav2) Setup:** This is another complex area.  You'll need:

    *   Install the `navigation2` ROS2 packages: `sudo apt install ros-humble-navigation2`
    *   Create a Nav2 configuration (a series of YAML files) that define:
        *   Costmaps (static map, obstacle map)
        *   Global and local planners
        *   Controllers
        *   Recovery behaviors
    *   Create a launch file to start the Nav2 stack, including the map server, planner server, controller server, recovery server, etc.
    *   Tune the Nav2 parameters for your robot and environment.

3.  **Web Interface (Motor Control):**

    *   Choose a web framework like Flask or Django.
    *   Create a web page with buttons for "Forward," "Backward," "Left," "Right," "Stop."
    *   Use the framework to create a ROS2 node that listens for commands from the web page.
    *   The node will then publish messages to the `motor_command` topic.

4.  **ESP32 Firmware Enhancements:**

    *   Implement PID control for smoother motor control.
    *   Add support for reading wheel encoders for odometry.
    *   Implement error handling and safety features (e.g., motor shutdown if communication is lost).
    *   Consider using ROSSerial to directly integrate the ESP32 as a ROS2 node, although this can be more complex.

5.  **System Monitoring:**  Expand the `system_monitor.py` node to:

    *   Check the status of other ROS2 nodes.
    *   Monitor CPU usage, memory usage, and disk space.
    *   Log data to files for debugging and analysis.
    *   Implement error reporting mechanisms.

6.  **Testing and Debugging:**

    *   Use `ros2 topic list` to see available topics.
    *   Use `ros2 topic echo <topic_name>` to view messages on a topic.
    *   Use `rviz2` to visualize sensor data (Lidar scans, SLAM map, Nav2 plans).
    *   Use `ros2 doctor` to check for ROS2 configuration issues.

7.  **Security:**

    *   If you are exposing the web interface to the internet, be sure to implement proper security measures (authentication, authorization, HTTPS).
    *   Consider using ROS2 security features (authentication, encryption) to protect your ROS2 network.

**Important Considerations:**

*   **Serial Port Permissions:** Make sure the user running the ROS2 nodes (inside the Docker container) has permission to access the serial ports.  You might need to add the user to the `dialout` group: `sudo usermod -a -G dialout $USER`.  Then, log out and log back in for the changes to take effect. You may need to adjust serial port permissions within the container as well.
*   **Networking:**  The `--net=host` option is often the simplest for getting started, but it has security implications.  Consider using Docker networking features to create a more isolated network for your ROS2 nodes.
*   **Resource Limits:**  On the Raspberry Pi, be mindful of CPU and memory usage.  Optimize your code and configurations to minimize resource consumption.

**How to get feedback from the motors(excluding the functionality of the robot):**

To get feedback from the motors you have two basic options:

1.  **Add Wheel Encoders:**

    *   **Hardware:** Purchase rotary encoders that can be attached to the shafts of your DC motors.
    *   **Wiring:** Connect the encoder signals (usually A, B, and sometimes Z/Index) to GPIO pins on the ESP32.
    *   **ESP32 Firmware:**  Modify your ESP32 firmware to:
        *   Read the encoder signals.
        *   Use interrupts to count the encoder pulses.
        *   Calculate the motor speed and/or position based on the pulse count.
        *   Send the motor speed/position data to the Raspberry Pi via Serial/UART or I2C.
    *   **ROS2 Node:**  Modify your `rpi_esp_interface.py` node to:
        *   Receive the motor speed/position data from the ESP32.
        *   Publish the data as ROS2 messages (e.g., using a custom message type).

2.  **Use Motor Drivers with Current Sensing:**

    *   **Hardware:** Replace your L298N with a motor driver that has current sensing capabilities (e.g., some models of the TB6612FNG or dedicated motor driver chips with current sense outputs).
    *   **Wiring:** Connect the current sense output of the motor driver to an analog input pin on the ESP32.
    *   **ESP32 Firmware:** Modify your ESP32 firmware to:
        *   Read the analog voltage from the current sense output.
        *   Convert the voltage to a current value (using the motor driver's specifications).
        *   Send the motor current data to the Raspberry Pi via Serial/UART or I2C.
    *   **ROS2 Node:** Modify your `rpi_esp_interface.py` node to:
        *   Receive the motor current data from the ESP32.
        *   Publish the data as ROS2 messages.

**Which Method to Choose?**

*   **Wheel Encoders:** Provide the most accurate feedback for motor speed and position, which is essential for precise odometry and control.
*   **Current Sensing:** Provides information about the motor's torque and load, which can be useful for detecting collisions or obstacles.  It's less accurate for speed/position measurement than encoders.

**Important Considerations for Motor Feedback:**

*   **Calibration:** You'll need to calibrate the encoder or current sensor readings to get accurate measurements.
*   **Filtering:**  The raw encoder or current sensor data may be noisy, so you'll likely need to apply filtering techniques (e.g., moving average filter) to smooth the data.
*   **Units:** Ensure that you are using consistent units (e.g., radians per second for motor speed) throughout your system.

This is a complex project, and it will take time and effort to get everything working correctly. Start with the basic hardware and software setup, and then gradually add more features and functionality.  Good luck!  Let me know if you have more specific questions as you progress.
