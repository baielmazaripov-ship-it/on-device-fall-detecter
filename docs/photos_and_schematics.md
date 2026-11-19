# Photos and Schematics

## 1. Prototype Photos (Standalone Power Bank Setup)

![Standalone Fall Detector Prototype Powered by Power Bank](../fell_detector.jpg)

- **Autonomous Operation:** The ESP32 DevKit V1 and MPU6050 sensor are powered directly via USB from a portable power bank, enabling tether-free testing during simulated falls and daily activities.
- **Status Feedback:** The external LED on `GPIO 25` provides immediate visual confirmation of power-on initialization and real-time fall alerts.

---

## 2. Wiring Diagram

```text
[Power Bank (USB 5V)]
        │
        ▼ (USB Cable)
[ESP32 DevKit V1]
  ├── 3V3  ───────────────────────► [MPU6050 VCC]
  ├── GND  ───┬───────────────────► [MPU6050 GND]
  │           └───┬───────────────► [MPU6050 AD0]   (I2C address 0x68)
  │               └───► [LED Cathode (-)]
  ├── GPIO 21 (SDA) ──────────────► [MPU6050 SDA]   (I2C 400 kHz)
  ├── GPIO 22 (SCL) ──────────────► [MPU6050 SCL]   (I2C 400 kHz)
  └── GPIO 25 ────► [Resistor] ───► [LED Anode (+)] (Status / Fall Alert)
---

## 3. Mechanical Design & Layout

- **Rigid Mounting:** The ESP32 DevKit V1 and MPU6050 breakout board are mounted securely onto a breadboard/base platform so the sensor moves strictly as a single rigid body with the wearer, preventing false vibration spikes from loose wires.
- **Orientation Independence:** Computing full 3D Euclidean vector magnitudes ($\sqrt{a_x^2 + a_y^2 + a_z^2}$ and $\sqrt{g_x^2 + g_y^2 + g_z^2}$) allows the module to operate reliably without requiring strict axis alignment when worn at the waist or torso.
- **Compact Tethered Power:** The short USB connection to the portable power bank keeps weight balanced and eliminates cable drag during motion testing.
