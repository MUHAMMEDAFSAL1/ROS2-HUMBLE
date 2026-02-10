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

Installing the ros2-apt-source package:

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
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

### Install Minimal Version/Installing Raspberry pi

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

ros 2 supporting lanaguages 

- 🐍 **Python** via [`rclpy`](https://github.com/ros2/rclpy)
- 💻 **C++** via [`rclcpp`](https://github.com/ros2/rclcpp)

