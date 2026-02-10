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
# 🤖 ROS 2 Humble Installation Guide
### Ubuntu 22.04 LTS | Complete Setup Guide

This repository provides a complete step-by-step guide to install **ROS 2 Humble Hawksbill** on Ubuntu 22.04 LTS.

ROS 2 Humble is a Long-Term Support (LTS) distribution designed for robotics, autonomous systems, drones, and industrial applications.

---

## 📌 Table of Contents

- [System Requirements](#-system-requirements)
- [Step 1: Configure Locale](#-step-1-configure-locale)
- [Step 2: Add ROS 2 Repository](#-step-2-add-ros-2-repository)
- [Step 3: Install ROS 2](#-step-3-install-ros-2)
- [Step 4: Setup Environment](#-step-4-setup-environment)
- [Step 5: Install Development Tools](#-step-5-install-development-tools)
- [Step 6: Test Installation](#-step-6-test-installation)
- [Create ROS 2 Workspace](#-create-ros-2-workspace)
- [DDS Overview](#-dds-data-distribution-service)
- [Useful ROS 2 Commands](#-useful-ros-2-commands)
- [Troubleshooting](#-troubleshooting)
- [Official Documentation](#-official-documentation)

---

## 📌 System Requirements

- Ubuntu 22.04 LTS (Jammy)
- 64-bit system (amd64)
- Internet connection
- Minimum 4GB RAM recommended

---

## 🧰 Step 1: Configure Locale

```bash
locale
sudo apt update
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Verify:

```bash
locale
```

---

## 📦 Step 2: Add ROS 2 Repository

Enable universe repository:

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
```

Add ROS 2 GPG key:

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
-o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add ROS 2 repository:

```bash
echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Update package index:

```bash
sudo apt update
```

---

## 🚀 Step 3: Install ROS 2

Upgrade system (recommended):

```bash
sudo apt upgrade -y
```

### Install Desktop Version (Recommended)

Includes:
- RViz
- Demo nodes
- GUI tools
- rqt tools

```bash
sudo apt install ros-humble-desktop -y
```

### Install Minimal Version

```bash
sudo apt install ros-humble-ros-base -y
```

---

## 🔧 Step 4: Setup Environment

Add ROS 2 setup script to your bash:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify installation:

```bash
printenv ROS_DISTRO
```

Expected output:

```
humble
```

---

## 🛠 Step 5: Install Development Tools

```bash
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
```

Optional build tools:

```bash
sudo apt install build-essential cmake git -y
```

---

## ✅ Step 6: Test Installation

Open Terminal 1:

```bash
ros2 run demo_nodes_cpp talker
```

Open Terminal 2:

```bash
ros2 run demo_nodes_py listener
```

If messages are publishing and receiving, ROS 2 is working correctly 🎉

---

## 📂 Create ROS 2 Workspace

Create workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

Build workspace:

```bash
colcon build
```

Source workspace:

```bash
source install/setup.bash
```

To make it permanent:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## ✅ DDS (Data Distribution Service)

ROS 2 uses DDS as its communication middleware.

DDS provides:

- Fast and real-time communication
- Automatic node discovery
- Decentralized architecture (no ROS Master)
- QoS (Quality of Service) configuration
- Multi-machine communication support

This makes ROS 2 more suitable for industrial and distributed robotic systems compared to ROS 1.

---

## 📌 Useful ROS 2 Commands

Li

- 🐍 **Python** via [`rclpy`](https://github.com/ros2/rclpy)
- 💻 **C++** via [`rclcpp`](https://github.com/ros2/rclcpp)

