# SR-01: Intelligent Ground Vehicle  
**Team SR-01 | G H Patel College of Engineering & Technology**  
**Competition:** Robofest Gujarat 5.0 – Ideation Stage Submission  
**Mentor:** Dr. Vinod Patel (Ph.D., IIT Delhi)  
**Team Members:**  
- Harshil Joshi — Mechanical Design  
- Jash Borad — ROS2 Development & Deployment  
- Kush Patel — Electronics & Embedded Systems  
- Pushti Nagrecha — Computer Vision  

---

## 📘 Overview
**SR-01** is an **Intelligent Ground Vehicle (IGV)** designed for autonomous navigation across varied terrains.  
It integrates **ROS2-based perception, planning, and control** to achieve self-driving capability, obstacle avoidance, and adaptive motion.  
The robot uses a **4-wheel independent swerve drive**, advanced **sensor fusion**, and **vision-based navigation** to operate without human control.

---

## 🚀 Key Features
- Fully autonomous navigation using **ROS2 Humble**
- **EKF-based sensor fusion** of LiDAR, IMU, and encoders
- **4-wheel swerve drive system** for omnidirectional movement
- **Depth and RGB perception** using Intel RealSense D435i
- **Gazebo + RViz simulation** for virtual testing
- **Dynamic Window Approach (DWA)** local planner for obstacle avoidance
- **QR/AR-based waypoint missions** for flexible route selection
- **Hardware-level safety** via E-stop and watchdog timers

---

## ⚙️ Hardware Stack
| Category | Components |
|-----------|-------------|
| **Compute** | NVIDIA Jetson Nano |
| **Microcontroller** | STM32F446ZE |
| **Sensors** | RPLiDAR A1M8, Intel RealSense D435i, BNO086 IMU |
| **Motor Drivers** | Cytron MDD20A (drive), Cytron MDD10A (steer) |
| **Power** | 11.1V 10000mAh Li-ion battery, DC-DC buck converters |
| **Chassis** | Aluminum 6061 with 4 swerve drive modules |

---

## 💻 Software Stack
| Layer | Tools / Packages |
|-------|------------------|
| **OS** | Ubuntu 22.04 + ROS2 Humble |
| **Mapping** | SLAM Toolbox |
| **Navigation** | Nav2 Stack (NavFn + DWA Planner) |
| **Vision** | OpenCV + RealSense SDK |
| **Control** | PID via STM32CubeIDE |
| **Simulation** | Gazebo & RViz2 |
| **Integration** | Custom ROS2 nodes for perception and motor control |

---

## 🧠 Methodology
1. **Concept and Design** – CAD modeling in SolidWorks  
2. **Simulation Validation** – Gazebo & RViz testing of SLAM and navigation  
3. **Mechanical Fabrication** – CNC & 3D-printed parts  
4. **Electronics Integration** – STM32 + motor drivers + sensors  
5. **Software Integration** – ROS2 nodes for sensors and control  
6. **Testing and Optimization** – PID tuning, waypoint missions, field trials  

---

## 🧩 Innovation Highlights
- **Lane-as-Obstacle Algorithm:** Treats lane boundaries as virtual obstacles for precise path planning.  
- **QR/AR Mission Upload:** Allows flexible mission assignments using visual markers.  
- **Hybrid Drive Mode:** Differential for Stage 1 navigation, Swerve for high-precision stages.  

---

## 📊 Project Timeline (Planned)
| Phase | Duration (Days) |
|--------|-----------------|
| Concept & Design | 8 |
| Fabrication & Assembly | 7 |
| Electronics Integration | 7 |
| Control System Programming | 10 |
| Testing & Calibration | 10 |
| Trial Runs | 10 |
| Optimization & Finalization | 7 |
| **Total** | **59 Days** |

---

## 🧰 Tools Used
- **SolidWorks** – CAD modeling  
- **ANSYS** – Structural simulation  
- **Gazebo + RViz2** – Robot simulation  
- **STM32CubeIDE** – Firmware development  
- **OpenCV** – Vision and perception  
- **SLAM Toolbox** – Mapping  
- **Nav2 Stack** – Navigation and control  

---

## 🧾 Status Update
This project was **submitted for the Ideation Stage** of **Robofest Gujarat 5.0** under the senior category.  
Despite meeting all technical and documentation requirements, **Team SR-01 was not selected in Stage 1** due to **organizational selection constraints from the Robofest committee**, not because of any issue from our side.

---

## 🧭 Future Work
- Integration of **stereo vision-based SLAM**  
- Implementation of **AI terrain classification**  
- Upgrade to **ROS2 Galactic + multi-sensor fusion**  
- Participation in **IGVC (International Ground Vehicle Competition) 2026**

---

## 🗂️ Repository Structure
