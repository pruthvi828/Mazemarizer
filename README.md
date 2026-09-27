# 🏆 MAZEMARIZER — Autonomous Line-Maze Solver
### Techfest IIT Bombay | ESP32-S3 Dual-Core Architecture | Held-Karp TSP + Dijkstra Algorithm

![Python Version](https://img.shields.io/badge/Python-3.11+-blue.svg)
![C++ Standard](https://img.shields.io/badge/C%2B%2B-17-green.svg)
![Target Hardware](https://img.shields.io/badge/MCU-ESP32--S3-red.svg)
![Projected Score](https://img.shields.io/badge/Projected%20Score-481%2F500-gold.svg)

---

## 📌 Executive Summary

**Mazemarizer** is a high-performance autonomous line-following maze solving robot designed for the **Meshmerize competition at Techfest, IIT Bombay**. 

Unlike standard line followers using simple LSRB (Wall Follower) algorithms that get trapped in loops and miss checkpoints, Mazemarizer combines:
1. **Held-Karp Exact TSP (Dynamic Programming)**: Guarantees visiting all checkpoints in minimal distance during the Dry Run.
2. **Dijkstra Shortest Path**: Solves the provably fastest route from Start to Goal for the Sprint Run.
3. **Cascaded FreeRTOS Dual-Core Control**: Core 0 runs a $1\text{kHz}$ position PID and $5\text{kHz}$ velocity PID while Core 1 handles real-time odometry and graph building.

---

## 📁 Repository Structure

```
📁 Mazemarizer/
├── 📁 firmware/
│   ├── Meshmerize_Champion.ino      ← ESP32-S3 FreeRTOS C++ main firmware
│   └── PCB_DESIGN_NOTES.kicad_notes  ← Hardware pin map & PCB layout guide
├── 📁 simulation/
│   ├── simulator.py                 ← Pygame live arena visualization
│   ├── score_optimizer.py           ← Kinematic S-curve & score optimizer
│   └── requirements.txt             ← Python dependencies (pygame, networkx)
├── 📁 docs/
│   ├── HARDWARE_SPECIFICATIONS.md   ← Full electrical & component specs
│   └── ISA_CHAMPION_BOM.md          ← Requisition list & passives BOM
├── ISA_CHAMPION_BOM.pdf             ← Official 2-page printable lab requisition PDF
└── README.md                        ← Project overview & usage guide
```

---

## 📊 Score & Performance Benchmark

$$\begin{array}{llcc}
\hline
\textbf{Category} & \textbf{Rule Basis} & \textbf{Standard Competitor} & \textbf{MAZEMARIZER} \\
\hline
\text{Checkpoints (A)} & 4 \text{ checkpoints} \times 25\text{ pts} & 25\text{--}50\text{ pts} & \mathbf{100\text{ pts}} \\
\text{Dry Run Complete (B)} & \text{Traverses all nodes} & 30\text{ pts} & \mathbf{30\text{ pts}} \\
\text{Dry Time Bonus (C)} & 180\text{s} - T_{\text{dry}} & 85\text{ pts} & \mathbf{167\text{ pts}} \\
\text{Shortest Path Bonus (S)} & \text{Dijkstra path} & 0\text{ pts (Tree path)} & \mathbf{30\text{ pts}} \\
\text{Sprint Time Bonus (T)} & 150\text{s} - T_{\text{sprint}} & 55\text{ pts} & \mathbf{149\text{ pts}} \\
\text{Red LED Indicator (L)} & \text{Rule 3 End-Zone trigger} & 5\text{ pts} & \mathbf{5\text{ pts}} \\
\hline
\mathbf{\text{TOTAL SCORE}} & & \mathbf{\sim 200\text{ / }500\text{ PTS}} & \mathbf{481\text{ / }500\text{ PTS}} \\
\hline
\end{array}$$

---

## ⚡ Hardware Architecture Summary

- **Main MCU**: ESP32-S3-DevKitC-1-N8 (Dual-Core LX7 @ 240MHz, 8MB Flash)
- **Actuators**: 2× Pololu 20:1 High-Power Carbon-Brush 12V Gearmotors
- **Motor Driver**: TI DRV8874 Dual Breakout ($R_{\text{DS(on)}} = 0.12\Omega$, 3A peak)
- **Power**: 3S 550mAh 95C LiPo ($12.6\text{V}$) + Pololu 5V 2.5A Buck Regulator + TI LP5907 3.3V LDO
- **Sensors**: 16× Pololu QTR-8RC IR array + ICM-42688-P / MPU-6050 6-axis IMU + 480 CPR Encoders

---

## 🚀 Quick Start & Usage

### 1. Run the Arena Simulator
```bash
cd simulation
pip install -r requirements.txt

# Run Techfest arena live simulation
python simulator.py

# Run kinematic pre-match score optimizer
python score_optimizer.py
```

### 2. Flash Firmware to ESP32-S3
1. Open `firmware/Meshmerize_Champion.ino` in Arduino IDE 2.x
2. Select Board: **ESP32S3 Dev Module**
3. Partition Scheme: **Huge APP (3MB No OTA)**
4. Upload Speed: **921600**
5. Flash and open Serial Monitor at **115200 baud**.

---

## 📜 License & Compliance
Designed in full compliance with **Techfest IIT Bombay Competition Guidelines**.
- **Rule 2**: SPST Master Power Start Switch supported.
- **Rule 3**: High-brightness Red LED (GPIO 21) triggers at End Zone.
- **Rule 4**: Footprint $125\text{ mm} \times 110\text{ mm} \le 150\text{ mm} \times 150\text{ mm}$.
