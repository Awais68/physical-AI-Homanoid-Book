# Physical AI & Humanoid Robotics

## Focus and Theme
AI Systems in the Physical World. Embodied Intelligence.

## Goal
Bridging the gap between the digital brain and the physical body.  
Students apply their AI knowledge to control Humanoid Robots in simulated and real-world environments.

## Quarter Overview
The future of AI extends beyond digital spaces into the physical world.  
Students learn to design, simulate, and deploy humanoid robots capable of natural human interactions using ROS 2, Gazebo, and NVIDIA Isaac.

---

## Modules

### Module 1: The Robotic Nervous System (ROS 2)
**Focus:** Middleware for robot control  
- ROS 2 Nodes, Topics, and Services  
- Bridging Python Agents to ROS controllers using `rclpy`  
- Understanding URDF (Unified Robot Description Format) for humanoids  

### Module 2: The Digital Twin (Gazebo & Unity)
**Focus:** Physics simulation and environment building  
- Simulating physics, gravity, and collisions in Gazebo  
- High-fidelity rendering and human-robot interaction in Unity  
- Simulating sensors: LiDAR, Depth Cameras, and IMUs  

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
**Focus:** Advanced perception and training  
- NVIDIA Isaac Sim: Photorealistic simulation & synthetic data generation  
- Isaac ROS: Hardware-accelerated VSLAM & navigation  
- Nav2: Path planning for bipedal humanoid movement  

### Module 4: Vision-Language-Action (VLA)
**Focus:** Convergence of LLMs and Robotics  
- Voice-to-Action: OpenAI Whisper for voice commands  
- Cognitive Planning: LLMs translate natural language into ROS 2 actions  
- **Capstone Project:** Autonomous Humanoid receives voice commands, plans paths, navigates, identifies objects via CV, and manipulates them  

---

## Why Physical AI Matters
Humanoid robots share our physical form and can learn from interacting in human environments.  
Transitioning from purely digital AI to embodied intelligence is the core value.

---

## Learning Outcomes
- Understand Physical AI principles and embodied intelligence  
- Master ROS 2 for robotic control  
- Simulate robots with Gazebo & Unity  
- Develop with NVIDIA Isaac AI platform  
- Design humanoid robots for natural interactions  
- Integrate GPT models for conversational robotics  

---

## Weekly Breakdown

**Weeks 1-2:** Introduction to Physical AI  
- Foundations of Physical AI & embodied intelligence  
- Overview of humanoid robotics  
- Sensor systems: LIDAR, cameras, IMUs, force/torque sensors  

**Weeks 3-5:** ROS 2 Fundamentals  
- Nodes, topics, services, actions  
- Building ROS 2 packages with Python  
- Launch files and parameter management  

**Weeks 6-7:** Robot Simulation with Gazebo  
- Gazebo environment setup  
- URDF & SDF robot description formats  
- Physics & sensor simulation  
- Introduction to Unity  

**Weeks 8-10:** NVIDIA Isaac Platform  
- Isaac SDK & Isaac Sim  
- AI-powered perception & manipulation  
- Reinforcement learning  
- Sim-to-real transfer  

**Weeks 11-12:** Humanoid Robot Development  
- Kinematics & dynamics  
- Bipedal locomotion & balance  
- Manipulation & grasping  
- Human-robot interaction  

**Week 13:** Conversational Robotics  
- GPT integration for conversational AI  
- Speech recognition & NLU  
- Multi-modal interaction: speech, gesture, vision  

---

## Assessments
- ROS 2 package project  
- Gazebo simulation implementation  
- Isaac-based perception pipeline  
- Capstone: Simulated humanoid with conversational AI  

---

## Hardware Requirements

### 1. Digital Twin Workstation
- GPU: RTX 4070 Ti (min 12GB VRAM)  
- CPU: Intel i7 (13th Gen+) / Ryzen 9  
- RAM: 64GB DDR5 (32GB minimum)  
- OS: Ubuntu 22.04 LTS  
- Note: Isaac Sim runs better on Linux; dual-boot if needed  

### 2. Physical AI Edge Kit
- Brain: NVIDIA Jetson Orin Nano / Orin NX  
- Eyes: Intel RealSense D435i / D455  
- Inner Ear: USB IMU (BNO055)  
- Voice Interface: USB Mic/Speaker (ReSpeaker)  

### 3. Robot Lab Options
- **Proxy Approach:** Quadruped / robotic arm, ~$1800-$3000  
- **Miniature Humanoid:** Table-top robots, $600-$16k  
- **Premium Lab:** Real bipedal humanoid (Unitree G1)  

### 4. Architecture Summary
| Component   | Hardware                  | Function                                           |
|------------|---------------------------|---------------------------------------------------|
| Sim Rig    | PC w/ RTX 4080 + Ubuntu  | Isaac Sim, Gazebo, Unity, LLM/VLA training       |
| Edge Brain | Jetson Orin Nano          | Runs inference locally                             |
| Sensors    | RealSense + Lidar         | Feeds real-world data to AI                        |
| Actuator   | Unitree Go2 or G1         | Receives motor commands from Jetson               |

---

## Cloud Alternative (High OpEx)
- Cloud Workstations: AWS g5.2xlarge / g6e.xlarge  
- Sim-to-real: Train in cloud → flash weights to local Jetson  
- Edge AI Kit required for physical deployment  

---

## Economy Jetson Student Kit (~$700)
- Brain: Jetson Orin Nano Dev Kit ($249)  
- Eyes: Intel RealSense D435i ($349)  
- Ears: ReSpeaker USB Mic ($69)  
- Misc: SD Card + Wires ($30)  

---

## Notes
- Latency trap: Cloud sim fine, real robot control requires local edge deployment  
- Students train in cloud, deploy locally for safe execution
