//
// File:    main.ino
// Author:  1n5aN1aC (Joshua Villwock)
// Purpose: Handles all logic for the RocketDuino
//
#include "Arduino.h"
#include "runningAverage.h"
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

// Version
#define ROCKETDUINO_VERSION "0.3.2"

// Pin designations
#define VSENSE_PIN A0

// Baud rate settings
#define GPS_BAUD       9600 //57600
#define TX_BAUD        9600
#define ACTUAL_TX_BAUD 1200
#define TARGET_TX_BAUD 1000

// Task Frequency (remember these are minimums)
#define FREQUENCY_SEND_TELEMETRY 1111
#define FREQUENCY_SEND_STATUS    2222

// Last send variables
unsigned long lastTelemetryTX;
unsigned long lastStatusTX;
unsigned long lastRX;

// Define barometer object. Basic I2C SCL & SDA
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

// The TinyGPS++ object
TinyGPSPlus gps_object;

// Task Frequency for battery checks
#define FREQUENCY_UPDATE_BATTERY     250
RunningAverage average_battery(10);
unsigned long lastBatteryUpdate;

// Task Frequency for barometer checks
#define FREQUENCY_UPDATE_BAROMETER   250
RunningAverage average_altitude_baro(4);
RunningAverage average_speed_baro(4);
RunningAverage average_accelerarion_baro(4);
RunningAverage average_temperature(4);
unsigned long lastBarometerUpdate;

// Task Frequency for gps checks
#define FREQUENCY_UPDATE_GPS_HEIGHT  250
RunningAverage average_altitude_gps(4);
RunningAverage average_speed_gps(4);
RunningAverage average_acceleration_gps(4);
unsigned long lastGpsHeightUpdate;

// Task Frequency for location calculations
#define FREQUENCY_CALCULATE_LOCATION 250
RunningAverage average_altitude(4);
RunningAverage average_speed(4);
RunningAverage average_acceleration(4);
unsigned long lastLocationCalculation;

//
// Run Initial setup Tasks
//
// 1. Initialize serial buffers
// 2. Send Version information
// 3. Setup barometer
//
void setup()
{
  //Initialize the starting values
  lastTelemetryTX = 0UL;
  lastStatusTX    = 0UL;
  lastRX          = 0UL;
  
  //Initialize both serial ports:
  Serial.begin(TX_BAUD);
  //Serial1.begin(GPS_BAUD);
  
  //Print welcome message
  Serial.println();
  Serial.print(F("RocketDuino v."));
  Serial.print(ROCKETDUINO_VERSION);
  Serial.print(F(" & TinyGPS++ v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();

  //Setup barometer
  Serial.print("Giving time for barometer to start up...  ");
  delay(1500);
  baro.begin();
  delay(500);
  Serial.print("Done");
  
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

//
// Main Loop Tasks
//
// 1. Recieve & Process GPS data
// 2. Recieve & Process Ground Control Packets
// 3. Process data & calculate averages (& Update Modes)
// 4. Send Telemetry if needed
// 5. Send Status information if needed.
// 6. Update LEDs
//
void loop()
{
  // 1. Recieve & Process GPS data
  // 2. Recieve & Process Ground Control Packets
  process_gps_serial();
  process_gc_serial();

  // 3. Process data & calculate averages (& Update Modes)
  update_battery();
  update_barometer();
  update_gps_altitude();
  calculate_location();

  // 4. Send Telemetry if needed
  // 5. Send Status information if needed.
  transmit_telemetry();
  transmit_status();

  // 6. Update LEDs
  update_leds();
}

//
// Updates the battery voltage running average
//
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
//
void update_barometer() {
  if (millis() - lastBarometerUpdate > FREQUENCY_UPDATE_BAROMETER) {
    average_altitude_baro.addValue(baro.getAltitude());

    //Now we update the temperature:
    float tempF = (baro.getTemperature()*1.8 + 32); //convert from C to F.
    average_temperature.addValue(tempF);

    lastBarometerUpdate = millis();
  }
}

//
// Updates the altitude from the passed in value, and updates the running average
//
void update_gps_altitude() {
  if (millis() - lastGpsHeightUpdate > FREQUENCY_UPDATE_GPS_HEIGHT) {
    average_altitude_gps.addValue(gps_object.altitude.meters());

    lastGpsHeightUpdate = millis();
  }
}

//
//
//
void calculate_location() {
  if (millis() - lastLocationCalculation > FREQUENCY_CALCULATE_LOCATION) {
    //TODO: Actually calculate speed, accell, etc.
    //TODO: Interleave baro & gps

    average_altitude = average_altitude_baro;
    
    lastLocationCalculation = millis();
  }
}

//
// 
//
void transmit_telemetry() {
  if (millis() - lastTelemetryTX > FREQUENCY_SEND_TELEMETRY) {
    //TODO:Transmit Telemetry

    int tacos = average_temperature.getAverage();
    
    Serial.println(tacos);

    lastTelemetryTX = millis();
  }
}

//
// 
//
void transmit_status() {
  if (millis() - lastStatusTX > FREQUENCY_SEND_STATUS) {
    //Transmit Status

    lastStatusTX = millis();
  }
}

//
// Recieves & Processes gps lines
//
void process_gps_serial() {
  //TODO: Receive GPS
  //while (Serial1.available() > 0) {
  //  gps.encode(Serial1.read());
  //}
}

//
// Recieves & Processes ground control commands
//
// Valid packets:
// P = 'ping' Keeps track of last RX from Ground Control
// A = arm the flight controller for takeoff
//
void process_gc_serial() {
  if (Serial.available() ) {
    byte ch = Serial.read();
    
    if (ch == 'P') {
      lastRX = millis();
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
// 
//
void update_leds() {
  
}
