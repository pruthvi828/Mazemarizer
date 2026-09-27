# 🔬 MAZEMARIZER CHAMPION — HARDWARE SPECIFICATIONS

### Techfest IIT Bombay | Autonomous Line-Maze Solver

---

## ⚡ 1. Microcontroller & Power System

| Parameter | Specification | Selection Rationale |
|---|---|---|
| **MCU** | ESP32-S3-DevKitC-1-N8 (Xtensa 32-bit LX7 @ 240MHz) | Dual-core architecture: Core 0 = 1kHz Control Loop, Core 1 = FreeRTOS Dijkstra Solver |
| **Battery** | Tattu 3S 550mAh 95C High-Burst LiPo (12.6V max) | 52A peak burst capacity — zero voltage sag during motor acceleration |
| **5V Step-Down** | Pololu D24V22F5 (5.0V, 2.5A Synchronous Buck) | Fixed 5.0V logic supply with no potentiometer drift |
| **3.3V LDO** | TI LP5907 (3.3V, 250mA) | $6.5\mu\text{V}$ ultra-low noise — essential for clean IMU gyro readings |
| **Reverse Protection** | SS54 Schottky Diode (5A / 40V) | Protects circuit against accidental reverse polarity connection |

---

## 🚗 2. Actuation & Drive System

| Subsystem | Specification | Performance Benefit |
|---|---|---|
| **Motors** | 2× Pololu 20:1 High-Power Carbon Brush 12V (Item #3064) | Carbon brushes withstand $12.6\text{V}$ non-stop without burning out |
| **Motor Driver** | TI DRV8874 Dual Breakout ($R_{\text{DS(on)}} = 0.12\Omega$) | $10\times$ less heat than TB6612FNG; $3\text{A}$ peak current capability |
| **Wheel Encoders** | Pololu Magnetic Encoder Pair (12 CPR, 480 CPR in CHANGE mode) | $0.209\text{ mm/tick}$ resolution for precise junction detection |
| **Wheels** | Pololu 32×7mm High-Traction Silicone Wheels ($\mu \approx 1.0$) | Eliminates wheel slip during high-speed $90^\circ$ pivot turns |
| **Caster** | Pololu 3/8" Stainless Steel Ball Caster | Low-friction 3-point chassis contact |

---

## 🛰️ 3. Sensing & Perception System

| Sensor | Specification | Function |
|---|---|---|
| **Line Sensors** | 2× Pololu QTR-8RC Array (16 IR sensors total) | Direct GPIO RC-discharge timing; $15000$ position resolution |
| **IMU / Gyro** | ICM-42688-P / MPU-6050 6-Axis Motion Sensor | $400\text{kHz}$ I2C stream; $\alpha=0.98$ complementary filter yaw fusion |
| **Voltage Sensor** | $27\text{k}\Omega / 100\text{k}\Omega$ 1% Metal Film Voltage Divider | $V_{\text{adc\_max}} = 2.68\text{V} \le 3.3\text{V}$ for dynamic PWM battery compensation |
| **End-Zone LED** | High-Brightness Red 5mm LED ($>1500\text{mcd}$) | Techfest Rule 3 compliance indicator |
| **Master Switch** | Panel-Mount SPST Toggle Switch ($3\text{A}$ rating) | Techfest Rule 2 compliance start switch |

---

## 📐 4. Physical Dimensions & Weight

- **Chassis Footprint**: $125\text{ mm} \times 110\text{ mm}$ ($\le 150\text{ mm} \times 150\text{ mm}$ Rule 4 boundary)
- **Total Weight**: $\sim 195\text{ g}$
- **Center of Gravity**: Positioned $18\text{ mm}$ ahead of drive axle for maximum traction.
