#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "globals.h"
#include "sensors.h"

// BNO055, sadece bu modul icinde kullaniliyor.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

void initSensors() {
  Wire.begin();
  Wire.setClock(400000);
  if (!bno.begin()) {
    Serial.println("Error initializing BNO055! Check connections.");
    while (1);
  }
  delay(2000);
  Serial.println("BNO055 OK.");

  double sum = 0;
  for (int i = 0; i < 300; i++) {
    sensors_event_t e;
    bno.getEvent(&e);
    sum += e.orientation.z;
    delay(5);
  }
  angleOffset = sum / 300.0;
  Serial.print("angleOffset = ");
  Serial.println(angleOffset, 3);
}

void readSensors() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData);
  double rawInput = orientationData.orientation.z - angleOffset;
  filteredInput = filteredInput * (1.0 - INPUT_FILTER) + rawInput * INPUT_FILTER;
  input = filteredInput;

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyroX = gyro.x();
  gyroY = gyro.y();
  gyroZ = gyro.z();
}
