# Photos and Schematics

1. Prototype Photos (Standalone Power Bank Setup)

[Standalone Fall Detector Prototype Powered by Power Bank](../%D0%B4%D0%B5%D1%82%D0%B5%D0%BA%D1%82%D0%BE%D1%80%20%D0%BF%D0%B0%D0%B4%D0%B5%D0%BD%D0%B8%D1%8F.jpg)

Autonomous Operation: The ESP32 DevKit V1 and MPU6050 sensor are powered directly via USB from a portable power bank, enabling tether-free testing during simulated falls and daily activities.
Status Feedback: The external LED on `GPIO 25` provides immediate visual confirmation of power-on initialization and real-time fall alerts.



 2. Wiring Diagram

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


 3. Mechanical Design & Layout

- Rigid Mounting:The ESP32 DevKit V1 and MPU6050 breakout board are mounted securely on a breadboard/base so the sensor moves as a single unit with the prototype, reducing accidental wire disconnects and noise.
- Orientation Independence: Full 3D Euclidean magnitudes allow flexible positioning without strict axis alignment.
- Standalone Power: The portable power bank sits right underneath/beside the breadboard with a short USB cable for untethered testing.
