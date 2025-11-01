# Smart-Assistive-Cane-with-Obstacle-Detection-and-Navigation
An assistive system  that enhances mobility and safety for visually impaired individuals through obstacle detection, fall prevention, and location tracking.

## Features
* **Obstacle Detection:** Ultrasonic sensor detects obstacles within 2 meters and provides distance-based alerts
* **Fall Preventation:** MPU6050 accelerometer/ gyroscope detects sudden movements and potential falls
* **Location Tracking:** GPS module provides real-time location data for emergency situations
* **Multi-level Alerts:** Buzzer, LED, and haptic feedback system with intensity based on proximity

## Hardware & Software
* **Microcontroller:** ESP8266 (WEMOS D1 Mini)
* **Sensors:**
  * MPU6050 Accelerometer & Gyroscope (Motion detection)
  * Ultrasonic Sensor (Obstacle detection)
  * NEO-M8N GPS Module (Location tracking)
* **Output Devices:**
  * Buzzer (Audio alerts)
  * LED (Visual indicator)
  * Vibration Motor (Haptic feedback)
* **Communication:** Serial communication for GPS data processing
* **Power Supply:**
  * 9V recharagable battery (stepped down to 5V for GPS)
  * 3.7V rechargable battery (for other components) 
* **Software:** Arduino IDE

## System Architecture

## Setup & Installation
*  **Hardware Connections**
  * **GPS Module (NEO-M8N)**
    * TX (D5/ GPIO14)
    * VCC (5V) 
  * **Ultrasonic Sensor**
    * Trig (D3/ GPIO0)
    * Echo (D6/ GPIO12)
    * VCC (5V) 
  * **MPU6050**
    * SCL (D1/ GPIO5)
    * SDA (D2/ GPIO4)
    * VCC (3.3V) 
  * **Output Devices**
    * LED (D7/ GPIO13)
    * Buzzer (D4/ GPIO2)
    * Vibrating Motor (D8/ GPIO15)
##  Software Configuration
*  **1. Install Required Libraries:**

  *  MPU6050:
  *  GPS: 
*  **2. Network Setup:**
*  **3. Upload Code:**
