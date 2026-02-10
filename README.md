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

