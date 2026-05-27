# Autonomous Adaptive Inspection Bot (AAIB) 🚀

The **Autonomous Adaptive Inspection Bot (AAIB)** is a globally deployable, cloud-connected robotic platform designed for surveying hazardous and confined industrial spaces. Built on **ROS 2 Humble** and simulated in **Gazebo Classic**, this 4WD skid-steer bot leverages a hybrid control architecture, allowing for seamless switching between autonomous map-based navigation and global manual teleoperation.

## 🌟 Key Features

* **Cloud Mission State Register (MSR):** Utilizes Firebase Firestore as a thread-safe, ultra-low-latency command bridge between the web and the robot, bypassing traditional WebSocket limitations.
* **Global Teleoperation via VPN:** Integrates **Tailscale VPN** to establish a secure, encrypted mesh network. Remote operators can command the bot and view telemetry over the internet, completely bypassing corporate firewalls.
* **Hybrid Control Logic:** * *Autonomous Mode:* Leverages the **Nav2 stack** for absolute localization, generating 2D occupancy grids via Lidar and tracking the robot directly on the map.
    * *Adaptive Mode:* Instantaneous manual override capability via a web-connected dashboard joystick.
* **Optimized Hardware Actuation:** Control directives are translated and published directly to the `/diff_drive_controller/cmd_vel_unstamped` topic, bypassing TF-tree computational bottlenecks for instant motor response.

## 🛠️ Tech Stack

* **Robotics Middleware:** ROS 2 Humble
* **Simulation:** Gazebo Classic
* **Navigation & Mapping:** Nav2 Stack, SLAM
* **Cloud & Database:** Firebase Firestore, Python (`firebase-admin` SDK)
* **Networking:** Tailscale VPN, No-IP (Dynamic DNS)
* **Web Dashboard:** HTML/CSS/JavaScript

## ⚙️ Prerequisites

Before cloning the repository, ensure your system (e.g., Ubuntu 22.04) has the following installed:
* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
* Gazebo Classic
* Python 3.10+
* `ros2_control` and `ros2_controllers` packages
* [Tailscale](https://tailscale.com/) (installed and authenticated on both the robot/host and the client device)

## 🚀 Installation & Setup

1. **Clone the repository into your ROS 2 workspace:**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone [https://github.com/yagyesh-Sahu-op/Autonomous-Adaptive-Inspection-Bot-AAIB-](https://github.com/yagyesh-Sahu-op/Autonomous-Adaptive-Inspection-Bot-AAIB-)
   
   
   
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_description spawn_web_robot_in_gazebo.launch.py
ros2 launch robot_description slam.launch.py
ros2 launch robot_description nav2.launch.py
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz



#####packages#####
robot-localization
twist-mux
slam-toolbox
navigation2
nav2-bringup
pointcloud-to-laserscan
