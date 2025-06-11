# ROS2 HUMBLE


![ros2_humble-1](https://github.com/user-attachments/assets/dc94eda2-53d9-43d9-91d7-1c367b4f40df)
🚀 Introduction to ROS 2 Humble Hawksbill

ROS 2 Humble Hawksbill is the eighth official release of the Robot Operating System (ROS) 2, and it is designated as a Long-Term Support (LTS) release, ensuring support and updates until May 2027. This release is optimized for Ubuntu 22.04 (Jammy Jellyfish) and is designed to enhance the development of robotic applications across various platforms, including Windows 10 and macOS.

ROS 2 (Robot Operating System 2) is not an operating system — it’s a framework that provides tools, libraries, and conventions for building robot applications. It helps different parts of a robot (sensors, motors, processors) communicate easily.
# 🧠 ROS 2 Core Concepts

This document summarizes the key concepts used in ROS 2 (Robot Operating System 2).

| **Concept**   | **Description**                                                                 |
|---------------|----------------------------------------------------------------------------------|
| **Node**      | A single executable that performs computation. <br>💡 *Example: motor_controller_node* |
| **Topic**     | A communication channel used by nodes to publish or subscribe messages (1-to-many). |
| **Publisher** | Sends messages to a topic so others can receive them.                            |
| **Subscriber**| Listens for messages on a topic to receive updates.                              |
| **Message**   | A predefined data structure sent between nodes. <br>📝 *Example: geometry_msgs/msg/Vector3* |
| **Service**   | A two-way synchronous communication. One node sends a request, another replies. |
| **Action**    | Used for long-running tasks that give feedback and can be canceled (e.g., navigation). |
| **Parameter** | Configuration variable set at runtime to modify node behavior.                  |
| **Launch**    | A file that starts multiple nodes and sets parameters automatically.            |
| **Package**   | A directory structure that contains source code, launch files, config, etc.      |
# ⚙️ ROS 2 Architecture

## ✅ DDS (Data Distribution Service)

ROS 2 uses **DDS (Data Distribution Service)** as its communication middleware.  
It provides:

- Fast, real-time, and scalable communication.
- Automatic discovery of nodes and topics.
- Support for distributed systems (nodes can run across different machines).

💡 This makes ROS 2 more suitable for industrial and real-time applications than ROS 1.

---

## ✅ Language Support

ROS 2 supports multiple client libraries:

- 🐍 **Python** via [`rclpy`](https://github.com/ros2/rclpy)
- 💻 **C++** via [`rclcpp`](https://github.com/ros2/rclcpp)

You can write nodes in either language based on your project needs.

> Example:
> - `talker.py` (Python publisher)
> - `listener.cpp` (C++ subscriber)
# 🛠️ Installation

## ✅ System: Ubuntu 22.04 + ROS 2 Humble

Follow the steps below to install ROS 2 Humble on Ubuntu 22.04:


## Update and upgrade system packages
   sudo apt update && sudo apt upgrade

## Install required tools
   sudo apt install curl gnupg2 lsb-release

## Add the ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

## Add the ROS 2 repository to your system
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

## Update package list
sudo apt update

## Install the full desktop version of ROS 2 Humble
sudo apt install ros-humble-desktop

## Source the ROS 2 environment
source /opt/ros/humble/setup.bash

## Create a new workspace and source directory
mkdir -p ~/ros2_ws/src

## Navigate to the workspace root
cd ~/ros2_ws

## Build the workspace (even if empty, this sets it up properly)
colcon build

## Source the workspace environment
source install/setup.bash

## 🧱 Creating a Node (Python Example)
      import rclpy
      from rclpy.node import Node

      class HelloNode(Node):
      def __init__(self):
      super().__init__('hello_node')
      self.get_logger().info("Hello from ROS 2 Node!")

      def main(args=None):
      rclpy.init(args=args)
      node = HelloNode()
      rclpy.spin(node)
      rclpy.shutdown()


## 🚀 Run with:

ros2 run <your_package_name> hello_node

## 📡 Topic Communication

## 📨 Publisher Node (Python)

This example publishes messages on the `chatter` topic every second.

    from std_msgs.msg import String
    from rclpy.node import Node
    import rclpy

    class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher.publish(msg)

   def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


##   🧠 What is a Node in ROS 2?
A Node is the smallest executable unit in ROS 2. Think of it as a single program/module in your robot.
### 🔧 Notes about ROS Nodes

- Each node does one job (e.g., reading sensors, controlling motors, etc.).
- Multiple nodes can run simultaneously and communicate with each other.
- Nodes can be written in **Python** or **C++**.
# 📦 Example from a Drone:
Let’s say your drone has these parts:
| Component         | ROS Node                  |
|-------------------|---------------------------|
| IMU Sensor        | `/imu_node`               |
| GPS               | `/gps_node`               |
| Motor Controller  | `/motorABLE_controller_node` |
| Flight Control    | `/flight_controller_node` |
| Camera            | `/camera_node`            |


Each of these runs as a separate node, and they communicate using topics, services, or actions.
### 💬 Node Communication Types

- **Topics** – For streaming data (like sensor values or commands)  
  👉 *Example:* `/imu/data`, `/cmd_vel`

- **Services** – For request-response actions (like taking a photo)

- **Actions** – For long-running tasks (like navigating to a waypoint)

This will render correctly on GitHub as a highlighted code block:

---

### 🔧 Creating a Node (Python Example)

python
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info("Hello from ROS 2 Node!")

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    rclpy.spin(node)
    rclpy.shutdown()
