# 🤖 4-DOF Gesture-Controlled Robotic Arm

**Dual-ESP32 | MQTT | MPU6050 | PCA9685 | Real-Time Control**

## 📌 Project Overview

This project presents a **4-Degree-of-Freedom (4-DOF) gesture-controlled robotic arm** designed using a **distributed dual-ESP32 architecture**. One ESP32 functions as a **Sensor Node**, acquiring human hand gestures using **two MPU6050 IMU sensors**, processing orientation data through **sensor fusion and filtering**, and transmitting control commands wirelessly via the **MQTT protocol**.

A second ESP32 operates as the **Actuation Node**, subscribing to MQTT messages and generating **precise PWM signals using a PCA9685 servo driver** to control four servo motors. The system incorporates **fail-safe home positioning**, **motion smoothing**, and **real-time responsiveness**, making it suitable for safe human–robot interaction.

---

### Node Responsibilities

**ESP32 – Sensor Node**

* Reads motion data from **2× MPU6050 IMU sensors** via I²C
* Computes roll and pitch angles
* Applies **calibration, complementary filtering, and smoothing**
* Publishes servo commands as JSON packets over **MQTT**

**ESP32 – Actuation Node**

* Subscribes to MQTT control topic
* Parses servo position commands
* Generates PWM signals via **PCA9685**
* Drives **4 servo motors (4-DOF)**
* Handles **fail-safe home positioning**
* Supports **dual control modes (IMU + Web UI)**

---

## 🔁 Control Workflow

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

| Servo Channel | Controlled By  | Motion        |
| ------------- | -------------- | ------------- |
| CH0           | Sensor-1 Roll  | Base rotation |
| CH1           | Sensor-2 Roll  | Joint-1       |
| CH2           | Sensor-2 Pitch | Joint-2       |
| CH3           | Sensor-1 Pitch | End-Effector  |

Workspace: **Semi-spherical R-R-R-R configuration**

---

## 🔬 Sensor Processing & Control Logic

* Accelerometer-based angle estimation
* Gyroscope angular rate integration
* **Complementary filter** for drift-free orientation estimation
* **Exponential smoothing** to reduce jitter
* Gesture range (±60°) mapped to servo range (0–180°)
* Median filtering and dead-band logic on actuation side

This ensures **stable, smooth, and low-latency motion control**.

---

## 🛡️ Safety & Fail-Safe Features

✔ Automatic **home position (90°)** on startup
✔ Smooth ramp-up motion to prevent mechanical shock
✔ Safe pose on communication loss
✔ Servo dead-band to avoid unnecessary oscillations
✔ Separation of sensing and actuation for fault isolation

---

## 🧩 Hardware Components

* ESP32 Development Boards (2×)
* MPU6050 IMU Sensors (2×)
* PCA9685 16-Channel PWM Servo Driver
* SG90 Servo Motors (4×)
* Buck Converter for stable power supply
* Laser-cut acrylic mechanical structure

---

## 🧪 Simulation & Validation

* **Wokwi Simulation** used to validate logic and communication flow
* **Hardware prototype** tested with real-time gesture input
* Demonstration video included showing smooth 4-DOF motion

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
5. Keep sensors flat for calibration
6. Begin gesture-controlled operation

---

## 🎯 Applications

* Human–robot interaction systems
* Assistive and rehabilitation robotics
* Tele-operation platforms
* Industrial pick-and-place automation
* Robotics and control system research

---

## 🧑‍💻 Skills & Keywords 

Embedded Systems, ESP32, Embedded C/C++, MPU6050, IMU Sensor Fusion, MQTT, IoT, PCA9685, PWM, Servo Control, Real-Time Systems, FreeRTOS, Robotics, Control Systems, Mechatronics, Wireless Communication, Safety-Critical Design

---

## 📌 Key Takeaway

This project demonstrates **end-to-end system thinking**, combining **sensing, communication, control, safety, and actuation** in a real-time embedded robotics application using industry-relevant tools and protocols.

---

## 👤 Authors
- Manash Pratim Ghosh

---

## 📄 License
This project is licensed under the MIT License – see the LICENSE file for details.


