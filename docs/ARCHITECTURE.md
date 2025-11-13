# SENTIO System Architecture

**Version:** v1.0  
**Division:** DevSora Deep-Tech Research & Robotics  
**Tagline:** *Empathy, Engineered.*

---

## 1. Overview

SENTIO is the unified human–machine empathy platform combining:

- **AETHER** — Cognitive AI stack (reasoning, affect fusion, explainability)
- **SENTIO** — Physical empathy interface (robot head/torso, voice, motion)

Together they form a local, privacy-preserving system capable of real-time multimodal emotion recognition, affect-driven behavior, and expressive communication.

---

## 2. Core Layers

| Layer | Component | Role |
|-------|-----------|------|
| L0 | **Hardware / Sensors** | LiDAR (TFmini-S), MaxSonar, IMU (MPU-9250), HQ Camera |
| L1 | **Embedded Control** | ESP32 + PCA9685 for servo / LED PWM |
| L2 | **ROS 2 Interface** | Humble packages: drivers, perception, policy, motion, tts |
| L3 | **Cognitive AI (AETHER)** | ONNX / TensorRT models for face, audio emotion fusion |
| L4 | **Expression Layer** | Gesture + Light + Voice via policy engine |
| L5 | **Frontend / HUD** | Next.js dashboard via rosbridge & websocket |
| L6 | **Host Services** | Piper TTS HTTP server, systemd watchdogs |

---

## 3. Data Flow Summary

Camera + Mic → Perception (Face + Audio)  
↓  
AETHER Affect Fusion → Policy Engine → Motion / TTS  
↓  
Servo Bridge + LED Driver + Piper TTS  
↓  
Expressive Response + HUD Visualization

---

## 4. Execution Environments

| Environment | OS / Platform | Purpose |
|-------------|----------------|----------|
| **Host** | Ubuntu 24.04 LTS (Legion Pro 5) | GPU compute + Piper TTS service |
| **Container** | Docker (ros:humble-ros-base) | ROS nodes & runtime |
| **Microcontroller** | ESP32 DevKitC + PCA9685 | Servo / LED control |
| **Frontend** | Next.js 20 + roslibjs | Visualization / interaction |

---

## 5. Design Principles

1. **Offline First** — all AI inference local  
2. **Explainable by Design** — every decision logged in `/aether/logs/`  
3. **Modular** — perception, policy, motion, tts isolated packages  
4. **Safety Critical** — torque-limited, IMU supervised servos  
5. **Human-Centered** — prioritizes emotional clarity and comfort  

---

## 6. Future Extensions

- HoloFace 3D projection  
- Haptic feedback skin  
- Cognitive memory (temporal affect recognition)  
- SENTIO SDK for third-party emotion→motion integration

# SENTIO System Architecture

## Data Flow
\`\`\`
Camera + Mic + LiDAR 
  ↓
[sentio_perception] - Feature extraction
  ↓
[sentio_fusion] - AETHER reasoning
  ↓
[sentio_policy] - Behavior selection
  ↓
[sentio_motion] - Servo control
  ↓
[sentio_demo] - Choreography execution
  ↓
Expression (gestures + lights + voice)
  ↓
[UI Dashboard] - Visualization & monitoring
\`\`\`