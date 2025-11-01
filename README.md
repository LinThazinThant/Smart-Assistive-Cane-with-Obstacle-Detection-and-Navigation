# Smart-Assistive-Cane-with-Obstacle-Detection-and-Navigation
An assistive system  that enhances mobility and safety for visually impaired individuals through obstacle detection, fall prevention, and location tracking.

## Features
* **Obstacle Detection:** Ultrasonic sensor detects obstacles within 2 meters and provides distance-based alerts
* **Fall Prevention:** MPU6050 accelerometer/ gyroscope detects sudden movements and potential falls
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
  * 9V recharageable battery (stepped down to 5V for GPS)
  * 3.7V rechargeable battery (for other components) 
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

   * MPU6050: MPU6050_light by rfetick (https://github.com/rfetick/MPU6050_light)
   * GPS: TinyGPSPlus by Mikal Hart (https://github.com/mikalhart/TinyGPSPlus)
*  **2. Code Setup:**
   * Upload the provided Arduino sketch to Wemos D1 Mini
   * Open Serial Monitor at 115200 baud
   * Ensure proper sensor calibration during initialization  

## Gallery
* Complete setup
  
* Wiring schematic

## Technical Details
* **Sensor Performance**
  * **Ultrasonic range:** 0-200cm
  * **MPU6050 sensitivity:** 10 degree threshold on any axis for motion detection 
 * **Alert Ranges**
  * **≤30 cm:** Continuous beeping (2000 Hz), constant LED illumination
  * **30-100 cm:** Rapid beeping (1500 Hz), fast LED blinking (200 ms intervals)
  * **100-200 cm:** Slow beeping (1000 Hz), slow LED blinking (500 ms intervals)

## Experimental Results & Validation
* **Design Constraints Met**
 * Ultrasonic: HC-SR04 integrated with distance measurement
 * Vibrating motor: Vibration alerts based on distance and motion
 * Wemos D1 Mini: ESP8266 used as main microcontroller
 * Battery power: Dual battery system implemented
 * Distance encoding: Multi-level vibration patterns
 * Environment friendly: Rechargeable batteries used

## Project Outcomes & Technical Insights
* **Challenges Faced**
  * GPS Cloud Integration: Firebase connection failed due to ESP8266's 2.4GHz WiFi limitation
  * Alternative Platforms: Adafruit IO faced TX/ RX pin configuration errors
  * Power Management: Dual battery system required careful voltage regulation 
* **Successful Implementations**
  * MPU6050 motion detection with proper calibration
  * Ultrasonic obstacle detection with multi-level alerts
  * GPS data acquisition (coordinates, altitude, speed, satellites)
  * Multi-output alert system (buzzer, LED, vibration motor)
  * Real-time serial monitoring of all sensor data 

## Project Demonstration
**YouTube Link:** https://youtube.com/shorts/n3jXyvvX5u4?si=4-YmaAFkoqB7-Y_v 

Live demonstration showing sensor functionality and alert systems

## Future Improvements
* **Cloud Integration:** Resolve Firebase/2.4GHz network compatibility issues
* **Mobile Application:** Develop companion app for caregiver monitoring
* **Waterproof Casing:** Create protective enclosure for outdoor use
* **Obstacle Classification:** Add AI-based object recognition

## Contributing
This project demonstrates embedded systems integration for assistive technology. Contributions for improved cloud connectivity, enhanced sensor accuracy, additional safety features, or better user experience are welcome to help the visually impaired community.

## Authors
* Lin Thazin Thant
* Hein Kyaw Nyein
