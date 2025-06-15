
# ARAGOG - Automated Robotic Arachnid Guided Over Gesture

This repository serves as an **early-stage robotics project** to explore ROS2, URDF modeling, and teleoperation.
The robot is a 12 DOF quadruped, controlled via real-time hand gestures, and visualized in RViz2.

**Kinematic modeling and inverse kinematics support will be added in the future** to improve coordinated leg motion and gait planning.

---

## 🚀 Features

- 12x MG90S servos (3 per leg)
- Raspberry Pi 5 control unit (separately powered)
- PCA9685 PWM driver
- OpenCV + MediaPipe gesture detection
- Robot described using URDF
- RViz2 visualization
- ROS2 control stack with joint publishers

---

## 📁 Repository Includes

- URDF robot description
- ROS2 launch files for visualization
- Python gesture-to-velocity script
- Joint state publishing for RViz2 visualization

---
