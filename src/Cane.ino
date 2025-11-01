/*****
*  Smart Cane for Visually Impaired
*  This ESP8266-based system provides obstacle detection, motion alerts, and location tracking
   for visually imparied individuals using ultrasonic sensor, MPU6050, and GPS module
*  Hardware:  ESP8266, HC-SR04 Ultrasonic, MPU6050 Gyroscope, NEO-M8N GPS, Buzzer, LED, Vibration Motor
*  Developed by: [Lin Thazin Thant], [Hein Kyaw Nyein]
*  Date:  [31 March 2025]
*****/

// Library included
#include <Wire.h>
#include <MPU6050_light.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// GPS Sensor Configuration
#define GPS_RX_PIN D5 // GPS TX → ESP8266 D5 (GPIO14)
SoftwareSerial gpsSerial(GPS_RX_PIN, -1); 
TinyGPSPlus gps;

// Ultrasonic Sensor Configuration
const int trigPin = D3; // Trig Pin → D3 (GPIO0) 
const int echoPin = D6; // Echo Pin → D6 (GPIO12)

// Buzzer, LED, and Motor Configuration
const int buzzerPin = D4; // GPIO2 - Audio alerts (1000-2000Hz based on distance)
const int ledPin = D7; // GPIO13 - Visual indicator
const int motorPin = D8; // GPIO15 - Haptic feedback (Vibration motor)

// Ultrasonic Measurement Variables
#define SOUND_SPEED 0.034 // cm/microsecond
long duration;
float distanceCm; // calculated distance in centimeters

// MPU6050 Sensor Setup
MPU6050 mpu(Wire); // MPU6050 with I2C communication
float prevAngleX = 0, prevAngleY = 0, prevAngleZ = 0; // previous X, Y, Z axes angle for change detection

// Timing Variables
unsigned long lastGPSCheck = 0;
const long gpsInterval = 5000; // GPS update interval (5 s)
unsigned long lastUltrasonicRead = 0;
const long ultrasonicInterval = 500; // Ultrasonic update interval (500 ms)
unsigned long lastMPUUpdate = 0; 
const long mpuUpdateInterval = 200; // MPU6050 update interval (200 ms)
updates (slower rate)

/*****
* SETUP FUNCTION - Initializes all components when ESP8266 starts
* This function runs once at startup and prepares all hardware/ software components:
  - Serial communication
  - GPS module 
  - I2C communication for MPU650
  - Pin modes for ultrasonic sensor and output devices
  - MPU6050 calibration and offset calculation
*****/
void setup() {
   // Initialize serial communication (115200 baud rate)
   Serial.begin(115200);

   // Initialize GPS communication (9600 baud rate)
   gpsSerial.begin(9600);

   // Initialize I2C communication 
   Wire.begin();

   // Configure pin modes 
   pinMode(trigPin, OUTPUT);
   pinMode(echoPin, INPUT);
   pinMode(buzzerPin, OUTPUT);
   pinMode(ledPin, OUTPUT);
   pinMode(motorPin, OUTPUT);

   // MPU6050 Initialization and Calibration
   byte status = mpu.begin(); // initialize MPU6050
   Serial.print("MPU6050 status: ");
   Serial.println(status);
   if (status != 0) {
      Serial.println("MPU6050 connection failed!");
   while (true); 
   }

   Serial.println("Calculating offsets...");
   mpu.calcOffsets(); // calibrate MPU6050
   Serial.println("Done!");
}

/*****
* MAIN LOOP - Continuous operattion handing all sensor reading and alert geenration
* This function runs repeatedly and manages:
   - GPS data processing and location updates
   - Ultrasonic obstacle detection and distance measurement
   - MPU6050 motion detection and orientation monitoring
   - Multi-level alert system based on sensor inputs
*****/
void loop() {
   // GPS Data Processing
   while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read()); // process GPS data
   }
   // Update GPS information at specified intervals (5 seconds)
   if (millis() - lastGPSCheck >= gpsInterval) {
      lastGPSCheck = millis(); 
      if (gps.location.isUpdated()) {
         displayGPSInfo(); // display GPS data
      } else {
         Serial.println(" No GPS Fix. Waiting..."); // no satellite
      }

   }

   // Ultrasonic Sensor Reading
   // Measure distance at specified intervals (500 ms)
   if (millis() - lastUltrasonicRead >= ultrasonicInterval) {
      lastUltrasonicRead = millis(); 
      // Generate ultrasonic pulse for distance measurement
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      // Measure echo pulse duration and calculate distance
      duration = pulseIn(echoPin, HIGH);
      distanceCm = duration * SOUND_SPEED / 2; // convert duration to distance
      // Display measured distance
      Serial.print(" Distance: ");
      Serial.print(distanceCm);
      Serial.println(" cm");
   }

   // MPU6050 Reading
   // Update motion data at specified intervals (200 ms)
   if (millis() - lastMPUUpdate >= mpuUpdateInterval) {
      lastMPUUpdate = millis(); 
      mpu.update(); // update MPU6050 data
      float angleX = mpu.getAngleX();
      float angleY = mpu.getAngleY();
      float angleZ = mpu.getAngleZ();
      // Displaying orientation data
      Serial.print(" Angle X: ");
      Serial.print(angleX);
      Serial.print("\tY: ");
      Serial.print(angleY);
      Serial.print("\tZ: ");
      Serial.println(angleZ);
      // Motion detection 
      if (abs(angleX - prevAngleX) > 10 || abs(angleY - prevAngleY) > 10 ||
abs(angleZ - prevAngleZ) > 10) {
         Serial.println(" Motor ON");
         digitalWrite(motorPin, HIGH); // vibration motor activates
         delay(500);
         Serial.println(" Motor OFF");
         digitalWrite(motorPin, LOW); // vibration motor deactivates
      }
      prevAngleX = angleX;
      prevAngleY = angleY;
      prevAngleZ = angleZ;
   }

   // Alert System
   if (distanceCm > 100 && distanceCm <= 200) {
      // Far range (100-200cm): Slow beeping and blinking
      tone(buzzerPin, 1000, 500); // 1000Hz tone for 500ms
      digitalWrite(ledPin, HIGH);
      delay(500);
      digitalWrite(ledPin, LOW);
   } else if (distanceCm > 30 && distanceCm <= 100) {
      // Medium range (30-100cm): Fast beeping and blinking
      tone(buzzerPin, 1500, 200); // 1500Hz tone for 200ms
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
   } else if (distanceCm <= 30 && distanceCm > 0) {
      // Critical range (0-30cm): Continuous alarm
      tone(buzzerPin, 2000); // 2000Hz tone 
      digitalWrite(ledPin, HIGH);
   } else {
      // No obstacle: No alert
      noTone(buzzerPin);
      digitalWrite(ledPin, LOW);
   }

   yield(); // Allow ESP8266 to handle background tasks
}

/*****
* GPS DATA DISPLAY - Shows location and navigation information
* This function prints formatted GPS data to serial monitor when valid location is available
* Displays: Latitude, Longitude, Altitude, Speed, and Satellite count
*****/
void displayGPSInfo() {
   Serial.print(" Latitude: ");
   Serial.println(gps.location.lat(), 6);  // 6 decimal places
   Serial.print(" Longitude: ");
   Serial.println(gps.location.lng(), 6); // 6 decimal places
   Serial.print(" Altitude: ");
   Serial.println(gps.altitude.meters()); // in meters
   Serial.print(" Speed: ");
   Serial.println(gps.speed.kmph()); // in km/ h
   Serial.print(" Satellites: ");
   Serial.println(gps.satellites.value());

   Serial.println();
}
