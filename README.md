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

- LiDAR
  - pycache/ 
  - lidar_navigation.py 
- YOLO 
  - pycache/ 
  - depth_camera.py
  - main.py
  - obstacle_detection.py
  - robot_movement.py
  - yolo_camera.py
  - yolo_demo.py
  - yolo_detection.py  
  - yolo_test.py
- README.md

---

## 🚀 Quickstart

1. **Clone**
   ```bash
   git clone https://github.com/FuriousSleepyOtter/AI-Based-Detection-Study.git
   cd AI-Based-Detection-Study
   ```
2. **Install dependencies**
   ```bash
   sudo apt update
   sudo apt install ros-noetic-desktop-full python3-opencv
   pip3 install ultralytics pyralsense2 numpy
   ```
3. **Build & source**
   ```bash
   colcon build
   source install/setup.bash
   ```
4. **Run simulation**
   - **YOLO + Depth**
     ```bash
     ros2 launch yolo_depth btingup.launch.py
     ```
   - LiDAR-Only
     ```bash
     ros2 launch lidar_reactive bringup.launch.py
     ```

---

## 🔧 Modules 

1. **YOLO + Depth-Camera System**
   - **yolo_detection.py**: Subscrives to `/camera/rgb/image_raw`, runs YOLOv8 inference, publishes bounding boxes and turn directions.
   - **depth_camera.py**: Subscrives to `/camera/depth/image_raw`, divides depth map into sectors, issues low-priority turn directions.
   - **obstacle_detection.py**: Finite-state machine merging YOLO and depth turn directions (YOLO has priority).
   - **robot_movement.py**: Translates FSM decisions into `/cmd_vel` velocity commands.
   - **main.py**: Initializes all the rest.
2. **Reactive LiDAR System**
   - **lidar_navigation.py**: Subscrives to `/scan`, splits 360º scan into three sectors, choses the cleares sector when obstacle blocking the path, publishes velocity commands directly.

---

## 📊 Results

Key findings:

| **System**   | **Avoidance Success**   | **Avg. Response Time**   | **Distance Accuracy**  |
|---------------|---------------|---------------|--------|
| YOLO + Depth  | 100%  | <1s  | ~40% at ~30m |
| LiDAR-Only | 100%  | ~~1.5s   | Precise <3.5m |

---

## 📖 References

1. H. Xia et al., “Modeling the Causes of Urban Traffic Crashes…,” Sustainability, 2024.
2. Z. Liu et al., “Enhancing Planning for Autonomous Driving…,” Remote Sens., 2024.
3. J. Zhang & S. Singh, “LOAM: Lidar Odometry and Mapping…,” ICRA 2014, doi:10.1109/ICRA.2014.6903815.
4. Y. Chen et al., “TF‑YOLO: A Transformer–Fusion‑Based YOLO Detector…,” World Electr. Veh. J., 2023, doi:10.3390/wevj14120352.
5. W.-Y. Hsu & W.-Y. Lin, “Ratio-and-Scale-Aware YOLO…,” IEEE Trans. Image Process., 2021, doi:10.1109/TIP.2020.3039574.
6. V. De Silva, “Robust Fusion of LiDAR and Wide‑Angle Camera Data…,” Sensors, 2018.

---

## 📄 License

This project is released under the **[MIT License](LICENSE)**. See for details.
Feel free to adapt and reuse.
