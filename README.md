# ROS2 HUMBLE


![ros2_humble-1](https://github.com/user-attachments/assets/dc94eda2-53d9-43d9-91d7-1c367b4f40df)
🚀 Introduction to ROS 2 Humble Hawksbill

ROS 2 Humble Hawksbill is the eighth official release of the Robot Operating System (ROS) 2, and it is designated as a Long-Term Support (LTS) release, ensuring support and updates until May 2027. This release is optimized for Ubuntu 22.04 (Jammy Jellyfish) and is designed to enhance the development of robotic applications across various platforms, including Windows 10 and macOS.

#  Ubuntu installation steps 
step1

# 🧠 What is a Node in ROS 2?
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

```python
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
This will render on GitHub as:

---

 
### 🔧 Creating a Node (Python Example)

```bash
ros2 node list              # List all running nodes
ros2 node info /node_name   # Get info about a specific node
ros2 run <pkg_name> <node_executable>  # Run a node from a package


