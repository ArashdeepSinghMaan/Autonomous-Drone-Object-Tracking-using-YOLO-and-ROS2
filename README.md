# 🧠 Autonomous Drone Object Tracking using YOLO and ROS2

**End-to-end project** for developing, simulating, and deploying an **autonomous drone** that uses **YOLO** for real-time object detection and **ROS2** for intelligent tracking and control.  

The system detects a target (e.g., *person, car, or custom object*) using a camera feed, estimates its position in the image frame, and commands the drone to follow the target while maintaining safe distance and altitude.

---

## 🚀 Project Overview

This project demonstrates the integration of:
- **Computer Vision (YOLOv8 / YOLO-NAS)** for real-time object detection  
- **ROS2** for perception–control communication  
- **Gazebo Simulation** for validation without hardware  
- **Optional PX4 Integration** for real-world deployment  

The drone autonomously detects and tracks a chosen target using onboard inference and visual servoing control.

---

## 🎯 Objectives

- Detect and classify objects using YOLO in real time.  
- Track the centroid of the target and compute velocity/position commands.  
- Command the drone to align and follow the moving target.  
- Maintain safety and stability in simulation (and later, hardware).  

---

## 🧩 System Architecture
┌────────────────────────────────────────────────────┐
│ ROS2 Network │
├────────────────────────────────────────────────────┤
│ │
│ ┌────────────┐ ┌──────────────┐ │
│ │ Camera Sim │───▶│ YOLO Node │──┐ │
│ └────────────┘ └──────────────┘ │ detections │
│ ▼ │
│ ┌────────────┐ │
│ │ Tracker │ │
│ │ + Control │────▶│ /cmd_vel (simulation) │
│ └────────────┘ │
│ │ │
│ (future) ┌────────────┐ │
│ │ MAVROS2 │────▶│ PX4 /setpoints │
│ └────────────┘ │
│ │
└────────────────────────────────────────────────────┘
---

## ⚙️ Project Stages

### **Stage A — Simulation without PX4**
Focus on:
- YOLO integration  
- Target detection & tracking  
- Visual servoing control loop  

Test using **Gazebo/Ignition** and a simple UAV model (e.g., `hector_quadrotor`).

### **Stage B — Integration with PX4**
Add:
- PX4 SITL for flight dynamics  
- MAVROS2 bridge for offboard control  
- Safety and mission logic  

Deploy on Jetson + Pixhawk hardware.

---

## 🧰 Tech Stack

| Component | Technology |
|------------|-------------|
| **Middleware** | ROS2 Humble / Iron |
| **Simulation** | Gazebo / Ignition |
| **Autopilot (optional)** | PX4 + MAVROS2 |
| **Detection** | YOLOv8 / YOLO-NAS (Ultralytics) |
| **Languages** | Python / C++ |
| **Hardware (optional)** | Jetson Nano / Orin + Pixhawk |
| **Deployment** | Docker / Native ROS2 workspace |

---

## 🏗️ Project Structure

```bash
autonomous_drone_yolo/
├── src/
│   ├── yolo_detector/           # YOLO node (subscribes camera, publishes detections)
│   ├── tracker_controller/      # Tracker + control node
│   ├── simple_drone_sim/        # Gazebo model + velocity interface
│   └── common_interfaces/       # Custom ROS2 message types (if any)
├── launch/
│   ├── simulation.launch.py     # Launch YOLO + tracker + sim
│   ├── yolo_only.launch.py
│   └── px4_integration.launch.py
├── config/
│   ├── yolo_params.yaml
│   ├── controller_gains.yaml
│   └── sim_params.yaml
├── worlds/
│   ├── tracking_test.world
│   └── environment.sdf
├── models/
│   ├── drone_model/
│   └── target_object/
├── scripts/
│   ├── setup_yolo.sh
│   ├── convert_to_trt.py
│   └── data_logger.py
├── requirements.txt
├── Dockerfile (optional)
├── README.md
└── LICENSE
```
