#include <Servo.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>

// Initialize GPS module
SoftwareSerial SoftSerial(2, 3);  // GPS module: RX = pin 2, TX = pin 3
unsigned char buffer[64];         // Buffer for receiving GPS data
int count = 0;                    // Buffer counter
String latitude = "N/A";          // Latitude
String longitude = "N/A";         // Longitude
String elevation = "0";           // Default elevation (set to 0 unless altitude sensor is added)

// Sensor values
int sensorValue;                  // MQ135 value (Air Quality Index)
int gsrValue;                     // GSR value
const int THRESHOLD = 75;         // AQI threshold

// SD card
const int chipSelect = 4;         // SD card CS pin

// LED setup
const int RED_LED_PIN = 5;
const int GREEN_LED_PIN = 6;

// Servo setup
Servo servo1;
Servo servo2;
int servoPos1 = 90;
int servoPos2 = 90;
bool isHigh = false;

// GPS timing control
unsigned long lastGPSTime = 0;
unsigned long gpsUpdateInterval = 1000;  // Update every 1 second

// Data record counter
int recordNumber = 1;

void setup() {
  Serial.begin(9600);             // Main serial monitor
  SoftSerial.begin(9600);         // GPS serial

  servo1.attach(7);               // Servo 1 on pin 7
  servo2.attach(10);              // Servo 2 on pin 10
  servo1.write(servoPos1);
  servo2.write(servoPos2);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    while (1); // Stop running
  }
  Serial.println("SD card initialized.");

  // Write CSV header
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Number,lat,lon,ele,time,AQI,GSR");
    dataFile.close();
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // Update GPS data every second
  if (currentMillis - lastGPSTime >= gpsUpdateInterval) {
    processGPSData();
    lastGPSTime = currentMillis;
  }

  sensorValue = analogRead(A0); // MQ135 sensor (AQI)
  gsrValue = analogRead(A1);    // GSR sensor

  // Print sensor and GPS data to serial monitor
  Serial.print("AQI: "); Serial.print(sensorValue);
  Serial.print(" | GSR: "); Serial.print(gsrValue);
  Serial.print(" | Lat: "); Serial.print(latitude);
  Serial.print(" | Lng: "); Serial.println(longitude);

  // LED indicator logic
  if (sensorValue > THRESHOLD) {
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
  } else {
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
  }

  // Servo control logic
  if (sensorValue > THRESHOLD && !isHigh) {
    moveServosToPosition(180, 0);
    isHigh = true;
  } else if (sensorValue <= THRESHOLD && isHigh) {
    moveServosToPosition(90, 90);
    isHigh = false;
  }

  saveDataToSD();
  delay(1000); // Delay between data logs
}

void moveServosToPosition(int pos1, int pos2) {
  int step = (pos1 > servoPos1) ? 1 : -1;
  for (int pos = servoPos1; pos != pos1; pos += step) {
    servo1.write(pos);
    servo2.write(180 - pos);
    delay(15);
  }
  servoPos1 = pos1;
  servoPos2 = pos2;
}

void processGPSData() {
  if (SoftSerial.available()) {
    while (SoftSerial.available()) {
      buffer[count++] = SoftSerial.read();
      if (count == 64) break;
    }
    parseGPSData();
    clearBufferArray();
    count = 0;
  }
}

void parseGPSData() {
  String rawData = "";
  for (int i = 0; i < count; i++) {
    rawData += char(buffer[i]);
  }
  int latIndex = rawData.indexOf("N");
  int lngIndex = rawData.indexOf("E");

  if (latIndex != -1 && lngIndex != -1) {
    latitude = rawData.substring(latIndex - 10, latIndex);
    longitude = rawData.substring(lngIndex - 11, lngIndex);
  }
}

void clearBufferArray() {
  for (int i = 0; i < count; i++) {
    buffer[i] = NULL;
  }
}

void saveDataToSD() {
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    // Convert millis to hh:mm:ss format
    unsigned long currentMillis = millis();
    int seconds = (currentMillis / 1000) % 60;
    int minutes = (currentMillis / 60000) % 60;
    int hours = (currentMillis / 3600000) % 24;
    char timeBuffer[9];
    sprintf(timeBuffer, "%02d:%02d:%02d", hours, minutes, seconds);

    // Write data line
    dataFile.print(recordNumber++); dataFile.print(",");
    dataFile.print(latitude); dataFile.print(",");
    dataFile.print(longitude); dataFile.print(",");
    dataFile.print(elevation); dataFile.print(",");
    dataFile.print(timeBuffer); dataFile.print(",");
    dataFile.print(sensorValue); dataFile.print(",");
    dataFile.println(gsrValue);
    dataFile.close();

    Serial.println("Data saved to SD card.");
  } else {
    Serial.println("Unable to open file!");
  }
}
