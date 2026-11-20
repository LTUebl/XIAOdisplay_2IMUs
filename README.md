# Seeed XIAO 1.14" IPS Display (nRF52840) + Dual ICM-20948 3D Knee & Joint Kinematics Suite

[![CircuitPython 9.x](https://img.shields.io/badge/CircuitPython-9.x-orange.svg)](https://circuitpython.org)
[![Web Bluetooth API](https://img.shields.io/badge/Web%20Bluetooth-iPhone%20Bluefy%20%26%20BLE%205.4-2563eb.svg)](https://developer.mozilla.org/en-US/docs/Web/API/Web_Bluetooth_API)
[![Web Serial API](https://img.shields.io/badge/Web%20Serial-115200%20Baud-0284c7.svg)](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API)
[![Three.js](https://img.shields.io/badge/3D%20Kinematics-Three.js-black.svg)](https://threejs.org)
[![Lab](https://img.shields.io/badge/LTU-Experimental%20Biomechanics%20Lab-0284c7.svg)](https://github.com/LTUebl)

---

## Overview

This system provides a wearable, real-time **Dual-IMU 3D Anatomical Knee & Joint Kinematics Platform** built on the **Seeed Studio XIAO 1.14" IPS Display (nRF52840 Plus)** microcontroller board and **two daisy-chained Adafruit ICM-20948 9-DOF IMU sensors** connected via 4-pin JST SH (STEMMA QT) I2C cables.

### Knee Sleeve & Anatomical Model Placement
* **Main Proximal Unit (Thigh / Femur)**: The **Seeed XIAO 1.14" IPS Display (nRF52840)** and **3.7V LiPo battery** are strapped to the **mid/distal femur** (anterior-lateral thigh above the patella).
* **Sensor #1 (`0x69`, default `AD0` open)**: Mounted on the **lateral distal femur** (upper cuff of the athletic compression knee sleeve).
* **Sensor #2 (`0x68`, back `AD0` jumper soldered)**: Connected via a flexible 4-pin STEMMA QT cable crossing the lateral knee joint line and mounted on the **lateral proximal tibia** (lower cuff of the knee sleeve).
* **Multi-Joint Support**: Defaults to **Knee Joint** (*Lateral Femur S1 `0x69` &rarr; Lateral Tibia S2 `0x68`*), and also supports **Wrist** (*Forearm &rarr; Hand*), **Elbow**, and **Ankle** presets.

---

## Direct iPhone Bluetooth Connection Guide (Bluefy – Web BLE Browser)

Because standard iOS Safari does not expose the Web Bluetooth API, connecting directly from an **iPhone or iPad** over Bluetooth Low Energy uses the free **Bluefy – Web BLE Browser** app:

1. **Install Bluefy on iPhone**: Download **[Bluefy – Web BLE Browser](https://apps.apple.com/us/app/bluefy-web-ble-browser/id1492822055)** (free on the iOS App Store).
2. **Power On the Knee Sleeve**: Slide the XIAO 3.7V LiPo power switch **ON**. The XIAO begins advertising over Bluetooth Low Energy as **`XIAO-IMU`** (Nordic UART Service `6e400001-b5a3-f393-e0a9-e50e24dcca9e`).
3. **Open the Web App in Bluefy**:
   * **Simple Single-Screen iPhone App**: Open [`https://ltuebl.github.io/XIAOdisplay_2IMUs/iphone.html`](https://ltuebl.github.io/XIAOdisplay_2IMUs/iphone.html)
   * **Full Lab Dashboard (iPhone & Desktop)**: Open [`https://ltuebl.github.io/XIAOdisplay_2IMUs/`](https://ltuebl.github.io/XIAOdisplay_2IMUs/)
4. **Connect Bluetooth**: Tap **Connect Bluetooth (`XIAO-IMU`)** and select **`XIAO-IMU`** from the popup sheet.
5. **Stand Still & Tap Start Recording**: Have the subject stand upright in neutral anatomical posture (`0°`) and tap **Start Recording (Auto-Calibrates Standing 0°)**.
6. **AirDrop / Save CSV**: When finished, tap **Stop Recording** and tap **Share / Export CSV** to open the native iOS Share Sheet (AirDrop directly to a Mac/iPad or Save to iPhone Files).

---

## Hardware Architecture & Lateral Sensor Orientation

### 1. Daisy-Chained I2C Sensor Topology (`400 kHz` Fast Mode)

```
┌────────────────────────────────────────────────────────────────┐
│  Seeed Studio XIAO 1.14" IPS Display (nRF52840 Plus)           │
│  • Strapped to Mid/Distal Femur + 3.7V LiPo Battery            │
│  • 1.14" ST7789 IPS LCD (240×135 Landscape Mode)               │
│  • Primary I2C Bus: SDA = D4 (P0.04), SCL = D5 (P0.05)         │
└──────────────────────────────┬─────────────────────────────────┘
                               │ 4-Pin JST-SH (STEMMA QT) Cable
                               ▼
┌────────────────────────────────────────────────────────────────┐
│  Sensor #1: Adafruit ICM-20948 9-DOF IMU (Lateral Distal Femur)│
│  • I2C Address: 0x69 (Default — back AD0 jumper OPEN)          │
│  • PCB X-axis: Along Femoral Shaft  |  PCB Z-axis: Lateral Out │
│  • Internal I2C Master SLV0 polls AK09916 Mag (0x0C)           │
└──────────────────────────────┬─────────────────────────────────┘
                               │ 4-Pin STEMMA QT Cable across Lateral Knee
                               ▼
┌────────────────────────────────────────────────────────────────┐
│  Sensor #2: Adafruit ICM-20948 9-DOF IMU (Lateral Prox. Tibia) │
│  • I2C Address: 0x68 (Back AD0 address jumper SOLDERED)        │
│  • PCB X-axis: Along Tibial Shaft   |  PCB Z-axis: Lateral Out │
│  • Internal I2C Master SLV0 polls AK09916 Mag (0x0C)           │
└────────────────────────────────────────────────────────────────┘
```

### 2. Lateral Femur & Tibia Sensor Axis Convention & Z-Y-X 3D Joint Angle Math

With both Adafruit ICM-20948 breakout boards mounted on the **lateral aspect of the knee** (flat PCB face normal pointing laterally outward):

* **Sensor Local $X$-axis** (*along the two white STEMMA QT connectors*): Aligned **proximally-distally along the femur/tibia shaft** &rarr; measures **Internal / External Axial Tibial Rotation ($\psi_X$)**.
* **Sensor Local $Y$-axis** (*perpendicular to the STEMMA QT axis, toward header pads*): Aligned **anteriorly-posteriorly in the sagittal plane** &rarr; measures **Varus / Valgus Frontal Ab/Adduction ($\theta_Y$)**.
* **Sensor Local $Z$-axis** (*normal to the flat PCB face*): Points **laterally outward along the anatomical knee flexion/extension hinge axis** &rarr; measures **Knee Flexion / Extension ($\phi_Z$)**.

#### Quaternion Fusion & Z-Y-X Intrinsic Euler Decomposition
1. **Sensor Orientation Quaternions ($q_1, q_2$)**:
   Each IMU updates its 3D unit quaternion $q_i = (w_i, x_i, y_i, z_i)$ using bias-compensated 3-axis gyroscope integration fused with a gravity-gated complementary accelerometer correction (and 9-DOF magnetometer yaw correction when available).
2. **Stationary Neutral Pose Calibration ($q_{\text{cal}}$)**:
   When the wearer stands in neutral anatomical extension ($0^\circ, 0^\circ, 0^\circ$), the firmware averages 35 stationary samples (~180 ms) to zero out all 6 gyroscope bias axes $(\mathbf{b}_{g1}, \mathbf{b}_{g2})$ and captures the exact relative mounting offset quaternion between the lateral femur and lateral tibia:
   $$q_{\text{cal}} = \left(q_1^{\text{neutral}}\right)^* \otimes q_2^{\text{neutral}}$$
3. **Calibrated Relative 3D Joint Quaternion ($q_{\text{rel}}(t)$)**:
   At every sample $t$, the orientation of the distal tibia ($S_2$) relative to the calibrated proximal femur reference frame ($S_1 \otimes q_{\text{cal}}$) is:
   $$q_{\text{rel}}(t) = \left(q_1(t) \otimes q_{\text{cal}}\right)^* \otimes q_2(t) = (w, x, y, z)$$
4. **Intrinsic Z-Y-X Anatomical Euler Decomposition**:
   Decomposing $q_{\text{rel}} = R_Z(\phi_{\text{FE}}) \, R_Y(\theta_{\text{VV}}) \, R_X(\psi_{\text{IE}})$ isolates the primary large-angle sagittal knee flexion rotation around $Z$ first, eliminating trigonometric cross-talk into varus/valgus or axial rotation:
   $$\phi_{\text{Flex/Ext}} = \left|\operatorname{atan2}\!\left(2(wz + xy),\; 1 - 2(y^2 + z^2)\right)\right| \cdot \frac{180}{\pi}$$
   $$\theta_{\text{Varus/Valgus}} = \operatorname{asin}\!\left(\operatorname{clamp}\!\left(2(wy - zx), -1, 1\right)\right) \cdot \frac{180}{\pi}$$
   $$\psi_{\text{Int/Ext Rot}} = \operatorname{atan2}\!\left(2(wx + yz),\; 1 - 2(x^2 + y^2)\right) \cdot \frac{180}{\pi}$$

### 3. Automatic Stationary Calibration Right Before Recording (`CMD:REC_START`)

To prevent accumulated gyroscope drift from corrupting recorded trials:
* Whenever the user presses **Start Recording (Auto-Calibrates Standing 0°)** in [`iphone.html`](./iphone.html) or [`index.html`](./index.html) (or presses the physical side button **`BTN_C`** on the XIAO), the system **immediately runs the stationary gyro-bias + neutral standing pose calibration ($0^\circ, 0^\circ, 0^\circ$)** right before sample `#1` is recorded.
* You can also re-zero at any time without recording by tapping **Zero / Calibrate Now (0°)** (`CMD:CAL_NEUTRAL`) or long-pressing **`BTN_A`** (`>1 s`) on the XIAO display.

---

## Seeed XIAO 1.14" IPS Display (nRF52840 Plus) Pin Map & Controls

| XIAO Pin | Chip GPIO | Net Name | Peripheral Function |
| :--- | :--- | :--- | :--- |
| **`D0` / `D1`** | `P0.02` / `P0.03` | `PDM_CLK` / `DATA` | Onboard PDM Microphone (Acoustic Event Marker in `code.py`) |
| **`D2` / `D3`** | `P0.28` / `P0.29` | `LCD_CS` / `LCD_DC` | ST7789 1.14" IPS Display SPI Chip Select & Data/Command |
| **`D4` / `D5`** | `P0.04` / `P0.05` | `SDA` / `SCL` | Primary I2C Bus (`400 kHz` to ICM-20948 `0x69` & `0x68`) |
| **`D6`** | `P1.11` | `BTN_A` | Left Front Button: Short = Next LCD Page &bull; Long (`>1s`) = Zero `0°` |
| **`D7`** | `P1.12` | `BTN_B` | Right Front Button: Short = Pause/Resume Stream &bull; Long (`>1s`) = Reset |
| **`D8` / `D10`** | `P1.13` / `P1.15` | `SCK` / `MOSI` | Hardware SPI (`24 MHz` to ST7789 1.14" IPS Display) |
| **`D16`** | `P0.31` | `VBAT` | LiPo Battery Voltage Divider ADC (`READ_BATT_ENABLE = P0.14`) |
| **`D17` / `D18`** | `P1.03` / `P1.05` | `LCD_RST` / `LCD_BL` | ST7789 Display Hardware Reset & Backlight Enable |
| **`D19`** | `P1.07` | `BTN_C` | Side Button: Short = **Auto-Calibrate `0°` & Start/Stop Recording** |

---

## Files in This Folder (Full vs. Simplified Options)

### CircuitPython Firmware (Choose Either for `CIRCUITPY/code.py`)
* [`code.py`](./code.py) — **Full Production Firmware**:
  * High-speed 9-DOF `FastICM20948` driver with internal auxiliary I2C master (`SLV0`) magnetometer polling, lateral knee Z-Y-X quaternion fusion, automatic stationary calibration on `CMD:REC_START` / `BTN_C`, 4-page 1.14" ST7789 landscape UI, local SRAM ring buffer + Flash CSV logging, battery monitor, PDM acoustic clap marker, and dual-stream telemetry (`D,...` 24-field USB Serial + `B,...` compact 13-field iPhone BLE at ~35 Hz).
* [`code_simple.py`](./code_simple.py) — **Streamlined Teaching & Student Firmware (~260 lines)**:
  * Clean, easy-to-read single-file firmware implementing the exact same lateral knee Z-Y-X 3D relative joint angle math, automatic stationary calibration before recording (`CMD:REC_START` / `BTN_C`), single-screen 1.14" live angle display, and BLE (`XIAO-IMU`) + USB streaming. To use it on the board, copy `code_simple.py` to `CIRCUITPY/code.py`.

### Web Applications (Both Hosted on GitHub Pages)
* [`iphone.html`](./iphone.html) — **Simple Single-Screen iPhone BLE Web App**:
  * Ultra-clean, thumb-friendly single-screen iPhone interface with top 3D anatomical knee/wrist model, 3 high-contrast live angle readouts (`Flexion/Extension`, `Varus/Valgus`, `Int/Ext Rotation`), 4 big control buttons (**Connect Bluetooth**, **Start/Stop Recording with Auto-Zero**, **Zero/Calibrate & Reset**, **Share/Export CSV via iOS Share Sheet**), and a live Chart.js strip chart.
* [`index.html`](./index.html) — **Full Biomechanics Lab Dashboard (iPhone BLE & Laptop USB)**:
  * Complete laboratory dashboard featuring the 3D anatomical knee/limb visualizer (`joint3d.js`), live virtual XIAO 1.14" LCD mirror, multi-tab real-time Chart.js plots (3D Angles, Angular Velocity, Linear Acceleration), full Range of Motion (ROM) analytics table, session data preview table, and bidirectional BLE/USB command console.
* [`joint3d.js`](./joint3d.js) — Three.js 3D anatomical Femur, Patella, Tibia, Fibula, Athletic Shoe, Compression Knee Sleeve, mid-femur XIAO 1.14" display, lateral ICM-20948 sensors (`0x69` & `0x68`), and flexing STEMMA QT wire harness.
* [`app.js`](./app.js) — Bidirectional Web Bluetooth + Web Serial telemetry engine for [`index.html`](./index.html).
* [`style.css`](./style.css) — Mobile-first responsive stylesheet for [`index.html`](./index.html).
* [`boot.py`](./boot.py) — Optional CircuitPython boot modifier enabling standalone onboard Flash CSV logging when `BTN_C` is held at power-on.
