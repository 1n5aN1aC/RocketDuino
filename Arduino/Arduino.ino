//
// File:    main.ino
// Author:  1n5aN1aC (Joshua Villwock)
// Purpose: Handles all logic for the RocketDuino
//
#include "Arduino.h"
#include "runningAverage.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

// Version
#define ROCKETDUINO_VERSION "0.3.4"

// Pin designations
#define VSENSE_PIN A0
#define GPS_RX_PIN 14
#define GPS_TX_PIN 12

//Serial Settings
#define TX_BAUD        9600

// GPS Settings
#define GPS_BAUD       9600
SoftwareSerial serialGPS(GPS_RX_PIN, GPS_TX_PIN); // RX, TX
TinyGPSPlus gps_object;    //The TinyGPS++ object

// Task Frequency (remember these are minimums)
#define FREQUENCY_SEND_TELEMETRY 1200
#define FREQUENCY_SEND_STATUS    2000

// Battery Variables
#define FREQUENCY_UPDATE_BATTERY     250
RunningAverage average_battery(10);
unsigned long lastBatteryUpdate;

// Barometer Variables
#define FREQUENCY_UPDATE_BAROMETER   250
RunningAverage average_altitude_baro(4);
RunningAverage average_temperature(4);
unsigned long lastBarometerUpdate;
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2(); // Define barometer object. Basic I2C SCL & SDA



//
// Run Initial setup Tasks
//
void setup() {
  // Initialize variables & objects
  setup_variables();
  
	//Initialize both serial ports
  Serial.begin(TX_BAUD);
  serialGPS.begin(GPS_BAUD);
  
  // Setup running averages
  
	// Send version information
	// Setup barometer
  
}

//
// Main Loop Tasks
//
void loop() {
	//Processing incoming Ground Control data
	//Process incoming GPS data
  process_gps_serial();
	//Gather new (baro) data
	//Calculate on data (flight modes)
	//Send data to Ground Control
	//Update LEDs / display

  if (gps_object.time.isUpdated())
    Serial.println(gps_object.time.value());
  
  if (gps_object.altitude.isUpdated())
    Serial.println(gps_object.altitude.meters());
  
  if (gps_object.satellites.isUpdated())
    Serial.println(gps_object.satellites.value());

  if (gps_object.location.isUpdated()) {
    Serial.println(gps_object.location.lat(), 6); // Latitude in degrees (double)
    Serial.println(gps_object.location.lng(), 6); // Longitude in degrees (double)
  }
}

//
// Initialize all the various 
void setup_variables() {
  average_battery.clear();
  average_altitude_baro.clear();
  average_temperature.clear();
}

//
// Updates the battery voltage running average
void update_battery() {
  if (millis() - lastBatteryUpdate > FREQUENCY_UPDATE_BATTERY) {
    float sample = analogRead(VSENSE_PIN);
    int centivolts = (sample * 5.015) / 1024.0 * 5.7 * 100;   //5.7 = voltage divider ratio
    average_battery.addValue(centivolts);

    lastBatteryUpdate = millis();
  }
}

//
// Queries the barometer for elevation (and temp), stores it, and updates the running average
void update_barometer() {
  if (millis() - lastBarometerUpdate > FREQUENCY_UPDATE_BAROMETER) {
    average_altitude_baro.addValue(baro.getAltitude());

    //Now we update the temperature:
    float tempF = (baro.getTemperature()*1.8 + 32); //convert from C to F.
    average_temperature.addValue(tempF);

    lastBarometerUpdate = millis();
  }
}

void calculate_location() {
  
}

void calculate_mode() {
  
}

void transmit_telemetry() {
  
}

void transmit_status() {
  
}

void update_leds() {
  
}

void update_diaply() {
  
}

//
// Recieves & Processes ground control commands
//
// Valid packets:
// P = 'ping' Keeps track of last RX from Ground Control
// A = arm the flight controller for takeoff
void process_gc_serial() {
  if (Serial.available() ) {
    byte ch = Serial.read();
    
    if (ch == 'P') {
//      lastRX = millis();
    }
    //else if (ch == 'A' && currMode < 0) {
    //  currMode = 0;
    //}
    else {
      //something went wrong?  Bad packet.
    }
  }
}

//
// Recieves & Processes gps lines
void process_gps_serial() {
  //TODO: Receive GPS
  while (serialGPS.available() > 0) {
    gps_object.encode(serialGPS.read());
  }
}
