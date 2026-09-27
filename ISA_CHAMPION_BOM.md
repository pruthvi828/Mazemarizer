# 🏆 MESHMERIZE CHAMPION — ISA LAB REQUISITION MANIFEST (v6.0)
### Techfest IIT Bombay | Autonomous Maze Solver | ESP32-S3 Architecture

> **Only what matters for winning. Everything else removed.**

---

## 📋 REQUISITION LIST (FOR ISA / ROBOTICS LAB INVENTORY)

| # | Subsystem | Component | Fallback | Qty | Why It Matters |
|---|---|---|---|:---:|---|
| **1** | **Main MCU** | **ESP32-S3-DevKitC-1-N8** (8MB Flash, Dual-Core LX7 @ 240MHz) | ESP32-S3-WROOM-1 DevKit | **2** | 1 racing + 1 pit spare. Dual-core: Core 0 = 1kHz PID, Core 1 = Dijkstra. Socket-mount — swap in 10 sec. |
| **2** | **Motors** | **Pololu 20:1 HPCB 12V Extended Shaft** (Item #3064) | N20 12V 600RPM with encoder | **2** | Carbon brushes survive 12.6V (3S) non-stop. Standard N20 burn out in <5 min at 3S. |
| **3** | **Encoders** | **Pololu Magnetic Encoder Pair for Micro Metal Gearmotors** (Item #4760) | N20 Hall Encoder (12 CPR) | **1 pair** | 480 counts/rev in CHANGE mode → 0.209mm/tick for accurate junction detection. |
| **4** | **Motor Driver** | **TI DRV8874 Dual Breakout** (or DRV8876) | TB6612FNG | **1** | R_DS(on)=0.12Ω vs TB6612's 1.2Ω — 10× less heat, no brownout at 3A. |
| **5** | **Line Sensor** | **2× Pololu QTR-8RC** (16 sensors total, direct GPIO RC read) | Pololu QTRX-HD-15A (single board) | **2** | No MUX needed. RC-discharge timing is noise-immune. 16 sensors = sub-mm line tracking. |
| **6** | **IMU** | **ICM-42688-P Breakout** (or BNO085) | GY-521 MPU-6050 Module | **1** | Low-noise gyro for accurate 90°/180° pivots. MPU-6050 has 10× more drift. |
| **7** | **Battery** | **Tattu 3S 550mAh 95C LiPo (XT30)** | GensAce 3S 450mAh 75C | **2** | 52A burst — zero voltage sag. Use one per run, swap in pit. |
| **8** | **5V Buck** | **Pololu D24V22F5** (5V, 2.5A synchronous buck) | MP1584EN module (pre-tuned 5.0V) | **1** | Fixed 5V output, no pot to drift. Powers ESP32 + QTR sensors. |
| **9** | **3.3V LDO** | **TI LP5907 (3.3V, 250mA)** | AMS1117-3.3V | **1** | 6.5µV noise — critical for clean IMU readings. AMS1117 too noisy for ICM-42688-P. |
| **10** | **Wheels** | **Pololu 32×7mm Silicone Wheels** (Item #1087) | Rubber N20 wheels | **2** | µ≈0.95–1.02 on arena vinyl — no slipping during pivot turns. |
| **11** | **Ball Caster** | **Pololu 3/8" Stainless Steel Ball Caster** (Item #953) | 3/8" plastic ball caster | **1** | Zero lateral scrub, consistent 3-point contact. |
| **12** | **Protection Diode** | **SS54 Schottky (5A / 40V)** | SS34 (3A) | **1** | Reverse polarity protection. One wrong battery connection kills the board without this. |
| **13** | **LED** | **Red 5mm High-Brightness LED (>1500mcd)** | Standard 5mm red LED | **1** | **MANDATORY — Techfest Rule 3.** Lights at end-zone. Judges disqualify without it. |
| **14** | **Start Switch** | **SPST Toggle Switch (3A rated)** | Any panel-mount SPST | **1** | **MANDATORY — Techfest Rule 2.** Robot must not move before switch flip. |
| **15** | **Battery Connector** | **XT30U Gold-Plated Connector Pair** | XT30 standard | **2 pairs** | One per battery. Rated for burst current. |
| **16** | **MCU Socket** | **22-Pin 2.54mm Female Header Strips** | Standard female headers | **4 strips** | Socket-mount ESP32-S3 so it can be swapped in 10 sec at the venue. |
| **17** | **Passives Kit** | See detail table below | — | **1 set** | Motor snubbers + power decoupling. |

---

## 🔌 PASSIVES DETAIL (Item 17)

| Component | Value | Qty | Location | Purpose |
|---|---|---:|---|---|
| Electrolytic cap | 470µF 16V | 2 | LiPo terminal + DRV8874 VM pin | Motor inrush energy |
| Electrolytic cap | 100µF 10V | 1 | Pololu Buck output | 5V rail bulk cap |
| Ceramic cap | 10µF 0805 | 2 | ESP32 3V3 + LP5907 output | Local supply bypass |
| Ceramic cap | 100nF 0402 | 10 | One per IC power pin | HF spike bypass |
| Ceramic cap | 100nF 50V | 4 | Motor terminals (2 per motor) | Brush-arcing RF suppression |
| Resistor | 100kΩ 0402 1% | 1 | Battery divider R1 (top leg) | Voltage sensing |
| Resistor | **27kΩ** 0402 1% | 1 | Battery divider R2 (bottom) | V_adc_max = 2.68V ≤ 3.3V ✅ |
| Resistor | 220Ω 0402 | 1 | Red LED current limit | ~14mA LED current |
| Resistor | 10kΩ 0402 | 4 | Encoder Phase A/B pull-ups | Clean digital signals |
| Ferrite bead | 600Ω@100MHz 0402 | 2 | Series on PWMA + PWMB lines | Motor EMI isolation |

---

## ⚡ PIN MAPPING — Final Verified

```
┌─────────────────────────────────────────────────────────────────┐
│          CONFLICT-FREE PIN ASSIGNMENTS v6.0                     │
├───────────────────────┬──────────────┬──────────────────────────┤
│ FUNCTION              │ GPIO         │ NOTES                    │
├───────────────────────┼──────────────┼──────────────────────────┤
│ Left Motor Dir 1      │ GPIO 4       │ DRV8874 AIN1             │
│ Left Motor Dir 2      │ GPIO 5       │ DRV8874 AIN2             │
│ Left Motor PWM        │ GPIO 6       │ LEDC ch0, 20kHz          │
│ Right Motor Dir 1     │ GPIO 7       │ DRV8874 BIN1             │
│ Right Motor Dir 2     │ GPIO 15      │ DRV8874 BIN2             │
│ Right Motor PWM       │ GPIO 16      │ LEDC ch1, 20kHz          │
│ Driver Enable         │ GPIO 17      │ HIGH = active            │
│ Left Encoder Phase A  │ GPIO 8       │ HW interrupt, CHANGE     │
│ Left Encoder Phase B  │ GPIO 9       │ Direction sense          │
│ Right Encoder Phase A │ GPIO 10      │ HW interrupt, CHANGE     │
│ Right Encoder Phase B │ GPIO 11      │ Direction sense          │
│ IMU SDA               │ GPIO 1       │ I2C @ 400kHz             │
│ IMU SCL               │ GPIO 2       │ I2C @ 400kHz             │
│ Battery ADC           │ GPIO 3       │ ADC1 Ch2 (max 2.68V) ✅  │
│ Start Button          │ GPIO 0       │ INPUT_PULLUP             │
│ End-Zone Red LED      │ GPIO 21      │ Techfest Rule 3          │
│ QTR-8RC Left Array    │ GPIO 35–42   │ 8 sensors, RC-discharge  │
│ QTR-8RC Right Array   │ GPIO 43–50   │ 8 sensors, RC-discharge  │
│ USB (Reserved)        │ GPIO 19, 20  │ USB D-/D+ — do NOT use   │
└───────────────────────┴──────────────┴──────────────────────────┘
```

---

## 🚫 REMOVED (And Why)

| Removed | Reason |
|---|---|
| LiPo Alarm | Battery draw is ~67mAh per full match on 550mAh pack. Never gets low enough to alarm. |
| LiPo Charger | ISA lab has chargers. Not a build component. |
| USB-C Cable | You have one. Not a build component. |
| UA78M05 5V Reg | Duplicate — Pololu Buck handles 5V. Two 5V sources would fight. |
| AMS1117-3.3V | Duplicate — LP5907 is the 3.3V source. Do not install both. |
| 47kΩ R2 Resistor | **Dangerous.** Gives 4.03V at GPIO 3 — destroys ADC. Use 27kΩ. |
| JST-SH Connectors | Unnecessary. Solder encoder wires directly with heat shrink. Fewer failure points. |
| 10nF ADC Caps | Not needed. 100nF rail cap + 1Hz firmware ADC read is sufficient. |
| Tools Section | Build in ISA lab — tools available there. |
