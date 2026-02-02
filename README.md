# 🤖 4-DOF Gesture-Controlled Robotic Arm

**Embedded Systems | Mechatronics | Distributed Control | Safety-Oriented Robotics**

---

## 📌 Project Overview

This project implements a **4-Degree-of-Freedom (4-DOF) gesture-controlled robotic arm** using a **distributed dual-ESP32 embedded architecture**.  
One ESP32 functions as a **Sensor Node**, capturing human hand motion using **dual MPU6050 IMU sensors**, performing **sensor fusion, calibration, and filtering**, and transmitting control commands wirelessly using the **MQTT protocol**.

A second ESP32 operates as the **Actuation Node**, subscribing to MQTT messages and generating **precise, timing-stable PWM signals via a PCA9685 servo driver** to control four servo motors.  
The system emphasizes **deterministic behavior, motion smoothness, and fail-safe operation**, making it suitable for **human–robot interaction and safety-aware robotic control**.

---

## 🧠 Distributed Node Responsibilities

### ESP32 – Sensor Node

- Acquires motion data from **2× MPU6050 IMU sensors** via I²C  
- Computes **roll and pitch** angles from raw accelerometer and gyroscope data  
- Applies **calibration, complementary filtering, and signal smoothing**  
- Packages servo position commands as **structured JSON messages**  
- Publishes control data over **MQTT via Wi-Fi**

---

### ESP32 – Actuation Node

- Subscribes to MQTT control topics  
- Parses incoming servo position commands  
- Generates stable PWM signals using **PCA9685 (hardware PWM offloading)**  
- Drives **4 servo motors (4-DOF)**  
- Implements **fail-safe home positioning logic**  
- Supports **dual control modes (IMU-based + Web UI)**

---

## 🔁 End-to-End Control Workflow

```
Human Hand Gesture
        ↓
MPU6050 Sensors (Roll & Pitch)
        ↓
ESP32 Sensor Node
  • Calibration
  • Sensor Fusion
  • Filtering
        ↓
MQTT over Wi-Fi
        ↓
ESP32 Actuation Node
  • Command Processing
  • Motion Smoothing
        ↓
PCA9685 PWM Driver
        ↓
4 Servo Motors (4-DOF)
```


---

## ⚙️ Degrees of Freedom (4-DOF Mapping)

| Servo Channel | Controlled By   | Motion Function |
|---------------|-----------------|-----------------|
| CH0           | Sensor-1 Roll   | Base rotation   |
| CH1           | Sensor-2 Roll   | Joint-1         |
| CH2           | Sensor-2 Pitch  | Joint-2         |
| CH3           | Sensor-1 Pitch  | End-Effector    |

**Kinematic Configuration:** Semi-spherical **R-R-R-R** robotic structure

---

## 🔬 Sensor Processing & Control Logic

- Accelerometer-based orientation estimation  
- Gyroscope angular rate integration  
- **Complementary filtering** for drift reduction and noise suppression  
- **Exponential smoothing** to minimize jitter and abrupt motion  
- Gesture input range (±60°) mapped to servo range (0–180°)  
- Dead-band logic and median filtering on the actuation side  

This control strategy ensures **stable, smooth, and low-latency robotic motion**.

---

## 🛡️ Safety & Fail-Safe Features

✔ Automatic **home position (90°)** on startup  
✔ Smooth ramp-up motion to prevent mechanical shock  
✔ Safe fallback pose on **communication loss**  
✔ Servo dead-band to prevent unnecessary oscillations  
✔ **Functional separation of sensing and actuation** for fault isolation  

Safety was treated as a **primary design constraint**, not an afterthought.

---

## 🧩 Hardware Components

- ESP32 Development Boards (2×)  
- MPU6050 IMU Sensors (2×)  
- PCA9685 16-Channel PWM Servo Driver  
- SG90 Servo Motors (4×)  
- Buck Converter for stable power regulation  
- Laser-cut acrylic mechanical structure  

---

## 🧪 Simulation & Validation

- **Wokwi simulation** used to validate control logic and MQTT communication flow  
- Hardware prototype tested with real-time gesture input  
- Demonstration videos included showing smooth, coordinated 4-DOF motion  

---

## 📂 Repository Structure

```
4DOF-Gesture-Controlled-Robotic-Arm/
│
├── Simulation Image/
│   └── wokwi_sim_image.png
│
├── demo/
│   ├── hardware.mp4
│   └── working_prototype.mp4
│
├── docs/
│   └── robotics_project_presentation.pptx
│
├── firmware/
│   ├── esp32_actuation_node/
│       └── mqtt_servo_controller_pca9685.ino
│   └──esp32_sensor_node/
│   │   └── gesture_sensor_mqtt_publisher.ino 
│
├── .gitignore
└── LICENSE
```

---

## 🚀 How to Run

1. Flash **Sensor Node ESP32** with IMU + MQTT publisher firmware  
2. Flash **Actuation Node ESP32** with PCA9685 servo controller firmware  
3. Configure Wi-Fi credentials and MQTT broker IP  
4. Power the system  
5. Keep sensors flat during startup for calibration  
6. Begin gesture-controlled operation  

---

## 🎯 Applications

- Human–robot interaction systems  
- Assistive and rehabilitation robotics  
- Tele-operation platforms  
- Industrial pick-and-place automation  
- Robotics and embedded control research  

---

## 🧑‍💻 Skills & Keywords

Embedded Systems, ESP32, Embedded C/C++, MPU6050, IMU Sensor Fusion, MQTT, IoT, PCA9685, PWM, Servo Control, Real-Time Systems, Robotics, Control Systems, Mechatronics, Wireless Communication, Safety-Oriented Design

---

## 📌 Key Engineering Takeaway

This project demonstrates **end-to-end system engineering**, integrating **sensing, communication, control, safety, and actuation** within a distributed real-time embedded robotics platform using industry-relevant tools and design principles.

---

## 👤 Author

**Manash Pratim Ghosh**  
GitHub: https://github.com/manashpgh

---

## 📄 Disclaimer

This project was developed as part of academic coursework and independent experimentation, with emphasis on **robustness, predictable behavior, and real-world engineering discipline**.


