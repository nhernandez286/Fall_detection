# Fall_detection
# Fall Detection System (MPU9250 + Blynk IoT)

##  Overview
This project is a microcontroller-based fall detection system that uses an MPU9250 sensor to monitor motion and orientation in real time. When a fall is detected, the system triggers an alert through the Blynk IoT platform.

The system also includes a physical button configured with interrupts, allowing the user to manually trigger an alert or reset an active alarm.

---

##  Features
- Real-time motion and orientation monitoring
- Fall detection using accelerometer/gyroscope data
- Automatic alert system via Blynk IoT
- Manual alert trigger using hardware button (interrupt-based)
- Alert reset functionality
- Remote visualization of sensor data

---

##  Hardware Components
- Microcontroller (ESP32 / Arduino-compatible board)
- MPU9250 (Accelerometer + Gyroscope + Magnetometer)
- Push button (interrupt-based input)
- Power supply

---

##  Software & Technologies
- Embedded C / Arduino framework
- Blynk IoT (mobile/web dashboard)
- I2C communication (MPU9250)

---

##  System Architecture
- MPU9250 collects motion and orientation data
- Microcontroller processes sensor data to detect falls
- Blynk IoT platform displays real-time data and alerts
- Interrupt-based button allows:
  - Manual alert trigger
  - Alert reset

---

##  How It Works

### Fall Detection Logic
- The system continuously reads acceleration and orientation data
- A fall is detected based on sudden changes in acceleration and position
- When thresholds are exceeded:
  - An alert is triggered
  - Notification is sent via Blynk

### Button Interrupt
- Button is configured using hardware interrupt
- Press actions:
  - Trigger manual alert
  - Reset active alert state

---

##  Blynk IoT Dashboard
The system integrates with Blynk IoT for:
- Real-time sensor visualization
- Alert notifications
- Remote monitoring

(Add screenshots of your dashboard here)

---

##  Setup Instructions

### 1. Hardware Setup
- Connect MPU9250 via I2C (SDA, SCL)
- Connect push button to interrupt-capable GPIO
- Ensure proper power supply

### 2. Software Setup
- Install required libraries:
  - MPU9250 library
  - Blynk library
- Configure Blynk credentials:
  - Auth Token
  - WiFi SSID & Password

### 3. Upload Code
- Compile and upload firmware to microcontroller
- Open Serial Monitor (optional for debugging)

---

##  Concepts Demonstrated
- Embedded systems design
- Sensor data processing
- Interrupt handling
- IoT integration
- Real-time monitoring systems

---

##  Future Improvements
- Machine learning-based fall detection
- Battery optimization for wearable use
- GPS tracking for emergency response
- Mobile push notifications enhancement

---

## 👨‍💻 Author
Nicholas Hernandez
