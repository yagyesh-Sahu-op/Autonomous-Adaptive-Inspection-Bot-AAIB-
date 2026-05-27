AAIB: Autonomous Adaptive Inspection Bot

A Cloud-Integrated ROS 2 Framework for Global Industrial Monitoring

📌 Overview

The Autonomous Adaptive Inspection Bot (AAIB) enables global control of ROS 2 robots by replacing local connections with a Cloud Mission State Register (MSR) on Firebase Firestore. This architecture allows secure, real-time command from any web device worldwide, overcoming traditional network limitations for industrial monitoring.

This project successfully implements a Hybrid Control Architecture, allowing seamless transitions between Autonomous mission navigation and Adaptive manual override. This dual-mode capability ensures that while the bot can perform routine coordinate-based patrols independently, human operators can instantly intervene via a virtual joystick when anomalies are detected, ensuring both efficiency and safety in remote inspection tasks.

🚀 Key Features

Cloud-Native Control: Replaces transient WebSockets with a persistent Firebase Firestore MSR, allowing for asynchronous command deployment and global reach.

Low Latency: Achieves sub-100ms end-to-end latency from Web UI to Gazebo simulation.

Hybrid Logic: Seamless switching between P-Control autonomous driving and direct manual teleoperation.

Real-time Telemetry: Live feedback of Robot Pose ($X, Y, Yaw$) and system status streamed back to the operator dashboard.

Industrial Kinematics: 4WD Skid-Steer model configured with ros2_control and validated in Gazebo.

🛠️ Tech Stack

Robotics: ROS 2 (Dashing/Foxy/Humble), URDF/Xacro, Gazebo, ros2_control.

Cloud: Firebase Admin SDK (Python), Cloud Firestore.

Web: React/HTML5, Tailwind CSS, Firebase JS SDK.

Language: Python 3.x, JavaScript.

🏗️ Architecture

The system is divided into three distinct layers:

Operator Layer (Web UI): The dashboard where missions are defined and manual commands are issued.

Cloud Middleware (MSR): The persistent database holding the "Single Source of Truth."

Robotics Layer (ROS 2 Bridge): A custom Python node that interprets cloud state into geometry_msgs/Twist commands.

🔧 Installation & Setup

1. Clone the Repository

mkdir -p ~/rover_ws/src
cd ~/rover_ws/src
git clone [https://github.com/your-username/aaib_robot.git](https://github.com/your-username/aaib_robot.git)




2. Install Dependencies

pip install firebase-admin
sudo apt install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-gazebo-ros2-control




3. Firebase Configuration

Place your serviceAccountKey.json in the my_rover/config/ directory.

Update the SERVICE_ACCOUNT_PATH in firestore_bridge_node.py.

4. Build and Launch

cd ~/rover_ws
colcon build --packages-select my_rover
source install/setup.bash
ros2 launch my_rover my_rover.launch.py




📈 Future Roadmap (Phase 2)

$$$$

 Advanced Perception: Integrating ML models (CNNs) for industrial fault detection (rust, cracks, safety violations).

$$$$

 Full Navigation: Integration of the ROS 2 Navigation Stack (Nav2) for SLAM and obstacle avoidance.

$$$$

 Multi-User Security: Firebase Authentication and control arbitration tokens for team-based inspection.

👤 Developer's Note

This project is a solo pursuit of excellence in robotics and cloud integration. It stands as a refutation of the idea that a single individual cannot tackle large-scale engineering challenges. Driven by a passion for engineering principles—precision, dedication, and harmony—AAIB is the first milestone toward a future of intelligent, global automation.

"Ganbatte!" (Do your best!)

© 2025 Yagyesh Sahu
