#Real-Time Physical Fatigue Detection — Edge AI on STM32H7

> **Final Year Project (PFE) — ENICarthage, 2026**  
> Embedded Systems Engineering · In collaboration with Luleå University of Technology (LTU), Sweden

---

## Overview

A wearable embedded system for **real-time physical fatigue detection** using multimodal biosignals, running entirely on the edge — no cloud inference required.

The system acquires biosignals from 4 sensors, processes them in real-time under **FreeRTOS**, runs an **INT8 quantized ML model** via X-CUBE-AI, and transmits results to an admin dashboard via **LoRa**.

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                  STM32H7A3 @ 280 MHz                │
│                                                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │
│  │ PPG/SpO2 │  │   IMU    │  │    EMG / ECG     │   │
│  │MAX86141  │  │ LSM6DSO  │  │    ADS1292R      │   │
│  └────┬─────┘  └────┬─────┘  └────────┬─────────┘   │
│       │              │                 │            │
│  ┌────▼──────────────▼─────────────────▼──────────┐ │
│  │           FreeRTOS Task Scheduler               ││
│  │  • Sensor Acquisition Tasks (IRQ + DMA)         ││
│  │  • NLMS Adaptive Filter (PPG artifact removal)  ││
│  │  • Feature Extraction (sliding window)          ││
│  │  • ML Inference Task (TFLite INT8 / X-CUBE-AI)  ││
│  │  • LoRa Transmission Task                       ││
│  └────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────┘
         │
         │ LoRa
         ▼
   Admin Dashboard
```

---

## Sensors & Signals

| Sensor         | Signal              | Sampling Rate |
|----------------|---------------------|-------------- |
| MAX86141       | PPG / SpO2          | 100 Hz        |
| LSM6DSO        | IMU (Accel + Gyro)  | 104 Hz        |
| ADS1292R       | EMG / ECG           | 500 Hz        |
| TMP117 / SHTC3 | Skin Temperature    | 1 Hz          |


## FreeRTOS Task Architecture

| Task | Priority | Description |
|------|----------|-------------|
| `TaskPPG` | High | MAX86141 acquisition via SPI + DMA |
| `TaskIMU` | High | LSM6DSO acquisition via I2C |
| `TaskEMG` | High | ADS1292R acquisition via SPI |
| `TaskTemp` | Low | TMP117/SHTC3 acquisition |
| `TaskSync` | High | Temporal sync via TIM2 (µs timestamps) |
| `TaskFeatures| Medium | Sliding window feature extraction |
| `TaskInference| Medium | TFLite INT8 inference via X-CUBE-AI |
| `TaskLoRa` | Low | Fatigue result transmission |

---

##  Edge AI Pipeline

```
Raw Biosignals
     │
     ▼
Preprocessing (NLMS adaptive filter for PPG motion artifacts,
               bandpass filters for EMG)
     │
     ▼
Feature Extraction (sliding window — time + frequency domain)
     │
     ▼
TFLite INT8 Model (CNN-TCN architecture)
quantized & deployed via X-CUBE-AI
     │
     ▼
Fatigue Label: [Normal | Moderate | High]
```

- **Model architecture:** CNN-TCN (Temporal Convolutional Network)
- **Quantization:** INT8 post-training quantization
- **Deployment tool:** ST X-CUBE-AI / ST Edge AI Core
- **Validation:** LOSO cross-validation (Leave-One-Subject-Out)

---

##  LoRa Communication

Fatigue detection results and vital signs are transmitted wirelessly to an **admin dashboard** using LoRa, enabling remote monitoring without WiFi/cellular infrastructure — ideal for industrial or outdoor environments.

---

##  Hardware

- **MCU:** STM32H7A3ZIT6 @ 280 MHz (Cortex-M7)
- **Custom PCB:** Designed for wearable form factor (includes all 4 sensors + LoRa module)
- **Memory:** DTCMRAM (512KB) · AXI-SRAM (1MB) · Flash (2MB)

---

##  Tech Stack

| Layer | Technologies |
|-------|-------------|
| Firmware | C / C++, STM32CubeIDE, HAL, bare-metal drivers |
| RTOS | FreeRTOS (tasks, semaphores, DMA sync) |
| Signal Processing | NLMS adaptive filter, bandpass FIR/IIR |
| ML Training | Python, TensorFlow/Keras, TFLite |
| Edge Deployment | X-CUBE-AI, ST Edge AI Core, INT8 quantization |
| Communication | LoRa (UART), SPI, I2C, DMA |
| PCB Design | KiCad / Altium |

---

## Repository Structure

```
fatigue-detection-edge-ai/
├── Core/
│   ├── Src/              # Main application + FreeRTOS tasks
│   └── Inc/              # Headers
├── Drivers/
│   ├── MAX86141/         # PPG/SpO2 driver
│   ├── LSM6DSO/          # IMU driver
│   ├── ADS1292R/         # EMG/ECG driver
│   └── TMP117/           # Temperature driver
├── Middlewares/
│   └── Third_Party/FreeRTOS/
├── ML/                   # TFLite model + X-CUBE-AI generated files
├── PCB/                  # Schematics and PCB layout
└── Acquisition_Biosignaux.ioc  # STM32CubeMX config
```

---

##  Current Status

- [x] FreeRTOS task architecture
- [x] MAX86141 PPG/SpO2 driver (SPI + DMA)
- [x] ADS1292R EMG/ECG driver
- [x] LSM6DSO IMU driver
- [x] NLMS adaptive filter for PPG motion artifact removal
- [x] Temporal synchronization (TIM2 µs timestamps)
- [ ] Feature extraction pipeline (in progress)
- [ ] ML model training & quantization (in progress)
- [ ] LoRa transmission task (in progress)
- [ ] PCB fabrication & testing (in progress)

---

## Author

**Khaoula Houki**  
Embedded Systems Engineering Student — ENICarthage  
---

## Academic Context

- **Institution:** École Nationale d'Ingénieurs de Carthage (ENICarthage), Tunisia
- **Research Partner:** Luleå University of Technology (LTU), Sweden
- **Year:** 2025–2026
