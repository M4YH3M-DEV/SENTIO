# SENTIO Security & Ethics Policy

---

## 1. Design Philosophy
SENTIO is built to respect privacy, ensure physical safety, and maintain full transparency in all interactions.

---

## 2. Data Privacy

| Data Type | Retention | Storage Location |
|------------|-----------|------------------|
| Video frames | RAM-only / no recording | Edge device (Nano) |
| Audio frames | ephemeral buffers < 3 s | Edge |
| Affect scores | anonymized | Local log |
| Logs | local rotation 24 h max | `/root/aether_sentio_ws/logs/` |

- No cloud upload.  
- Logs use randomized session IDs.  
- User faces or voices never stored persistently.

---

## 3. Physical Safety

- Torque-limited servos.  
- IMU monitoring for fall detection.  
- Emergency Stop (ESP32 GPIO button).  
- “Consent Light”: LED ON when sensing active.

---

## 4. Network Security
- Docker container runs host-only network, no internet.  
- ROS 2 DDS domain ID = 42 (ensures namespace isolation).  
- Web HUD auth token required for control endpoints.

---

## 5. Ethics & AI Transparency
- Models are auditable ONNX files.  
- Every decision logged with timestamp and emotion probabilities.  
- Policy engine is rule + reinforcement hybrid, fully explainable.  
- Dataset sources are licensed academic corpora only.

---

## 6. Compliance
- Hardware CE / EMC safe.  
- GDPR / Indian DPDP Act ready (local only).  
- ISO 13482 (ethical robotics) alignment phase 2.

---

## 7. Responsible Use Notice
SENTIO is a research prototype for demonstrating empathic interfaces.  
It must not be deployed in environments where safety of life depends on it without explicit certification.