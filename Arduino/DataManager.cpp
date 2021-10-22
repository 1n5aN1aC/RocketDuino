//
// File:    DataManager.cpp
// Author:  1n5aN1aC (Joshua Villwock)
// Purpose: Handles storing and updating all internal data variables for Rocketduino functions
//
#include "DataManager.h"
#include "runningAverage.h"
#include "Arduino.h"
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

// Pin designations
#define VSENSE_PIN A0

// Define barometer object. Basic I2C SCL & SDA
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

// The TinyGPS++ object
TinyGPSPlus gps_object;

// Task Frequency for battery checks
#define FREQUENCY_UPDATE_BATTERY     250
RunningAverage average_battery(10);

// Task Frequency for barometer checks
#define FREQUENCY_UPDATE_BAROMETER   250
RunningAverage average_altitude_baro(4);
RunningAverage average_speed_baro(4);
RunningAverage average_accelerarion_baro(4);
RunningAverage average_temperature(4);

// Task Frequency for gps checks
#define FREQUENCY_UPDATE_GPS_HEIGHT  250
RunningAverage average_altitude_gps(4);
RunningAverage average_speed_gps(4);
RunningAverage average_acceleration_gps(4);

// Task Frequency for location calculations
#define FREQUENCY_CALCULATE_LOCATION 250
RunningAverage average_altitude(4);
RunningAverage average_speed(4);
RunningAverage average_acceleration(4);

DataManager::DataManager(void) {
  //Initialize the starting values
  last_altitude_gps         = 0;
  last_altitude_baro        = 0;
  
  average_altitude_baro.clear();
  average_altitude_gps.clear();
  average_speed_gps.clear();
  average_speed_baro.clear();
  average_acceleration_gps.clear();
  average_accelerarion_baro.clear();

  average_battery.clear();
  average_altitude.clear();
  average_speed.clear();
  average_acceleration.clear();
  average_temperature.clear();

  // Time counters
  lastBatteryUpdate       = 0UL;
  lastBarometerUpdate     = 0UL;
  lastGpsHeightUpdate     = 0UL;
  lastLocationCalculation = 0UL;
}

int get_average_battery() {
  return average_battery.getAverage();
}

float get_average_altitude() {
  return average_altitude.getAverage();
}

float get_average_speed() {
  return average_speed.getAverage();
}

float get_average_acceleration() {
  return average_acceleration.getAverage();
}

float get_average_temperature() {
  return average_temperature.getAverage();
}

//
// 
//
void DataManager::setup_baro() {
  Serial.print("Giving time for barometer to start up...  ");
  delay(1500);
  baro.begin();
  delay(500);
  Serial.print("Done");
}

//
// Updates the battery voltage running average
//
void DataManager::update_battery() {
  if (millis() - lastBatteryUpdate > FREQUENCY_UPDATE_BATTERY) {
    float sample = analogRead(VSENSE_PIN);
    int centivolts = (sample * 5.015) / 1024.0 * 5.7 * 100;   //5.7 = voltage divider ratio
    average_battery.addValue(centivolts);

    lastBatteryUpdate = millis();
  }
}

//
// Queries the barometer for elevation (and temp), stores it, and updates the running average
//
void DataManager::update_barometer() {
  if (millis() - lastBarometerUpdate > FREQUENCY_UPDATE_BAROMETER) {
    last_altitude_baro = baro.getAltitude();
    average_altitude_baro.addValue(last_altitude_baro);
    
    Serial.println(last_altitude_baro);


    //Now we update the temperature:
    float tempF = (baro.getTemperature()*1.8 + 32); //convert from C to F.
    average_temperature.addValue(tempF);

    lastBarometerUpdate = millis();
  }
}

//
// Updates the altitude from the passed in value, and updates the running average
//
void DataManager::update_gps_altitude() {
  if (millis() - lastGpsHeightUpdate > FREQUENCY_UPDATE_GPS_HEIGHT) {
    last_altitude_gps = gps_object.altitude.meters();
    average_altitude_gps.addValue(last_altitude_gps);

    lastGpsHeightUpdate = millis();
  }
}

//
//
//
void DataManager::calculate_location() {
  if (millis() - lastLocationCalculation > FREQUENCY_CALCULATE_LOCATION) {
    //TODO: Actually calculate speed, accell, etc.
    //TODO: Interleave baro & gps

    average_altitude = average_altitude_baro;
    
    lastLocationCalculation = millis();
  }
}
