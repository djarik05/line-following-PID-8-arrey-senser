# 🚗 Smart Line Follower Robot (PID + Auto Calibration)

An advanced **Arduino-based smart line follower robot** using an **8-sensor IR array** and **TB6612FNG motor driver**.  
This project features **automatic calibration**, **PID control**, **sharp 90° turn detection**, and **robust line-lost recovery**.

> ⚠️ **Note:** Sensor **S5 (A5)** is intentionally disabled due to hardware fault.

---

## ✨ Features

✔ Auto sensor calibration (5 seconds at startup)  
✔ Automatic normalization (0–1000 range)  
✔ PID-based line following  
✔ Accurate **90° left & right turn detection**  
✔ Same-location spin recovery when line is lost  
✔ Works with **one faulty sensor (S5 disabled)**  
✔ Optimized for narrow (≈3 cm) tracks  

---

## 🧠 Control Strategy

### 1. Sensor Normalization
- Each sensor is calibrated for **min (white)** and **max (black)** values
- Sensor output is normalized between **0.0 (white)** and **1.0 (black)**

### 2. PID Line Following
- **Weighted average method** for error calculation
- Uses **PD control** (Proportional + Derivative)
- Automatically slows down on sharp curves

### 3. 90° Turn Detection
- Detects extreme sensor patterns
- Performs **hard in-place rotation**
- Direction memory used for recovery

### 4. Line Lost Recovery
- If all sensors see white → robot spins
- Spins in **last known turn direction**
- Stops spinning once center sensors detect the line

---

## 🔌 Hardware Requirements

- Arduino  Nano
- 8-Channel IR Sensor Array (Analog)
- TB6612FNG Motor Driver
- 2 N20 moter
- Robot chassis + wheels
-  Li-Po battery (7–12 V)

---

## 📌 Sensor Configuration

| Sensor | Arduino Pin | Status |
|------|------------|--------|
| S1 | A7 | Active |
| S2 | A6 | Active |
| S3 | A5 | ❌ Disabled |
| S4 | A4 | Active |
| S5 | A3 | Active |
| S6 | A2 | Active |
| S7 | A1 | Active |
| S8 | A0 | Active |

> Disabled sensor index:
```cpp
#define DEAD_SENSOR 2   // A5
