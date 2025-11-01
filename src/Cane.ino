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

*****/
void setup() {
Serial.begin(115200);
gpsSerial.begin(9600);
Wire.begin();

// Pin Modes
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
pinMode(buzzerPin, OUTPUT);
pinMode(ledPin, OUTPUT);
pinMode(motorPin, OUTPUT);

// MPU6050 Initialization
byte status = mpu.begin();
Serial.print("MPU6050 status: ");
Serial.println(status);
if (status != 0) {
Serial.println("MPU6050 connection failed!");

while (true); // Stop execution if MPU6050 is not connected
}

Serial.println("Calculating offsets...");
mpu.calcOffsets(); // Calibrate MPU6050
Serial.println("Done!");
}

void loop() {
// ----- GPS Data Processing -----
while (gpsSerial.available() > 0) {
gps.encode(gpsSerial.read()); // Process GPS data
}

if (millis() - lastGPSCheck >= gpsInterval) {
lastGPSCheck = millis(); // Reset the timer
if (gps.location.isUpdated()) {
displayGPSInfo(); // Display GPS data
} else {
Serial.println(" No GPS Fix. Waiting...");
}

}

// ----- Ultrasonic Sensor Reading -----
if (millis() - lastUltrasonicRead >= ultrasonicInterval) {
lastUltrasonicRead = millis(); // Reset the timer

digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distanceCm = duration * SOUND_SPEED / 2;

Serial.print(" Distance: ");
Serial.print(distanceCm);
Serial.println(" cm");
}

// ----- MPU6050 Reading (with Delay) -----
if (millis() - lastMPUUpdate >= mpuUpdateInterval) {

lastMPUUpdate = millis(); // Reset the timer for MPU6050 update

mpu.update(); // Update MPU6050 data
float angleX = mpu.getAngleX();
float angleY = mpu.getAngleY();
float angleZ = mpu.getAngleZ();

// ----- Debugging MPU6050 data -----
Serial.print(" Angle X: ");
Serial.print(angleX);
Serial.print("\tY: ");
Serial.print(angleY);
Serial.print("\tZ: ");
Serial.println(angleZ);

// ----- Motion Detection -----
if (abs(angleX - prevAngleX) > 10 || abs(angleY - prevAngleY) > 10 ||
abs(angleZ - prevAngleZ) > 10) {
Serial.println(" Motor ON");
digitalWrite(motorPin, HIGH);
delay(500);
Serial.println(" Motor OFF");

digitalWrite(motorPin, LOW);
}

prevAngleX = angleX;
prevAngleY = angleY;
prevAngleZ = angleZ;
}

// ----- Buzzer & LED Alerts -----
if (distanceCm > 100 && distanceCm <= 200) {
tone(buzzerPin, 1000, 500);
digitalWrite(ledPin, HIGH);
delay(500);
digitalWrite(ledPin, LOW);
} else if (distanceCm > 30 && distanceCm <= 100) {
tone(buzzerPin, 1500, 200);
digitalWrite(ledPin, HIGH);
delay(200);
digitalWrite(ledPin, LOW);
} else if (distanceCm <= 30 && distanceCm > 0) {
tone(buzzerPin, 2000);

digitalWrite(ledPin, HIGH);
} else {
noTone(buzzerPin);
digitalWrite(ledPin, LOW);
}

yield(); // Allow ESP8266 background tasks
}

// Display GPS Data
void displayGPSInfo() {
Serial.print(" Latitude: ");
Serial.println(gps.location.lat(), 6);
Serial.print(" Longitude: ");
Serial.println(gps.location.lng(), 6);
Serial.print(" Altitude: ");
Serial.println(gps.altitude.meters());
Serial.print(" Speed: ");
Serial.println(gps.speed.kmph());
Serial.print(" Satellites: ");
Serial.println(gps.satellites.value());

Serial.println();
}
