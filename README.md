# AI-Based Detection Study

**Enhancing Pedestrian Safety at Constructionn Sites through AI-Driven Obstacle Avoidance**
_Lluis Porras Alpañez - Mechatronics Engeneering at DMU_

---

## 🌟 Project Overview

Construction sites are high-risk environments where heavy machinery and workers operate under variable lighting, dust, and constantly changing layouts ([Xia et al., 2024](#references)). This project compares two autonomous obstacle avoidance methods for pedestrian enhanced safety:

1. **YOLO + Depth-Camera System**
   - Uses the YOLO (You Only Look Once) object detection algorithm on RGB frames to spot pedestrians, plus a depth camera to avoid static obstacles.
   - Modular ROS nodes handle image acquisition, YOLO inference, depth processing, and motion control.
2. **Reactive LiDAR-Only System**
   - Employs a 2D LiDAR scanner (360 beams, 3.5 m range) for real-time obstacle avoidance without mapping.
   - Splits scan into front/left/right sectors, steers toward the clearest sector.
  
Both systems run in **ROS Noetic + Gazebo** on a TurtleBot3 Waffle model. I record metrics-detection accuracy, response time, collision-avoidance success-to benchmarck performance and identify strengths, weaknesses, and fusion opportunities.

---

## 📂 Repository Structure


