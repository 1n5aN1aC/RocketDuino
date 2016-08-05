//
// File:    main.cpp
// Author:  1n5aN1aC (Joshua Villwock)
// Purpose: Handles primary logic for the RocketDuino
//

#include <TinyGPS++.h>
#define ROCKETDUINO_VERSION "0.1.5"

// Baud rate settings
#define GPS_BAUD       57600
#define TX_BAUD        9600
#define ACTUAL_TX_BAUD 1200
#define TARGET_TX_BAUD 1000

// Task Frequency
#define TELEMETRY_FREQUENCY 1111
#define STATUS_FREQUENCY    2222
#define F_INFO_FREQUENCY    250

// Pin designations
#define VSENSE_PIN A2


// The TinyGPS++ object
TinyGPSPlus gps;

// Moving Average Values
float avgBattery   = 0;
float avgAltitude  = 0;
float avgSpeed     = 0;
float avgAccel     = 0;

// Alphas
#define BATTERY_ALPHA  0.05
#define ALTITUDE_ALPHA 0.1
#define SPEED_ALPHA    0.1
#define ACCEL_ALPHA    0.1

// Current Mode of flight we are in.
// <0 = failsafe
// 0  = prep
// 1  = powered flight
// 2  = coast
// 3  = apex
// 4  = drouge deployed
// 5  = main deployment
// 6  = recovery
int currMode = -1;

// Time counters
unsigned long lastInfoUpdate  = 0UL;
unsigned long lastTelemetryTX = 0UL;
unsigned long lastStatusTX    = 0UL;
unsigned long lastRX          = 0UL;

// Counters for last packet statistics send
unsigned int lastBadSum  = 0;
unsigned int lastGoodSum = 0;
unsigned int lastWithFix = 0;


//
// Run Initial setup Tasks
//
// 0. Initialize serial buffers
// 1. Send Version information
//
void setup() {
  // initialize both serial ports:
  Serial.begin(TX_BAUD);
  Serial1.begin(GPS_BAUD);

  Serial.println();
  Serial.print(F("RocketDuino v."));
  Serial.print(ROCKETDUINO_VERSION);
  Serial.print(F(" & TinyGPS++ v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
}


//
// Main Loop Tasks
//
// 1. Recieve & Process GPS data
// 2. Recieve & Process Ground Control Packets
// 3. Send Telemetry if needed
// 4. Send Status information if needed.
// 5. Calculate 'averages'
//
byte ch;
void loop() {
  //Process GPS updates
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  //Process GC updates
  if (Serial.available() ) {
    ch = Serial.read();
    process_command(ch);
  }

  //Send telemetry info (If needed)
  if (millis() - lastTelemetryTX > TELEMETRY_FREQUENCY)
    telemetryTX();

  //Send status info (If needed)
  if (millis() - lastStatusTX > STATUS_FREQUENCY)
    statusTX();

  //Update flight info & mode
  if (millis() - lastInfoUpdate > F_INFO_FREQUENCY)
    update_flight_info();
}


//
// Updates all the averages for altitude, speed, flight mode, etc.
//
void update_flight_info() {
  //Update altitude average
  float tempAlt = gps.altitude.meters();  //This should use pressure?
  oldAlt = avgAltitude;
  avgAltitude = (avgAltitude * (1 - ALTITUDE_ALPHA)) + (tempAlt * ALTITUDE_ALPHA);

  //Update speed average
  float tempspeed = avgAltitude - oldAlt; //Or perhaps combine gps alt speed change & pressure change speed?
  oldSpeed = avgSpeed;
  avgSpeed = (avgSpeed * (1 - SPEED_ALPHA)) + (abs(tempspeed) * SPEED_ALPHA);

  //Update acceleration average
  float tempaccel = avgSpeed - oldSpeed;
  avgAccel = (avgAccel * (1 - ACCEL_ALPHA)) + (avgAccel * ACCEL_ALPHA);

  //Update flight mode
  update_flight_mode();

  //Update battery voltage
  float sample = analogRead(VSENSE_PIN);
  int centivolts = (sample * 5.015) / 1024.0 * 5.7 * 100;   //5.7 = voltage divider ratio
  avgBattery = (avgBattery * (1 - BATTERY_ALPHA)) + (centivolts * BATTERY_ALPHA);
}

void update_flight_mode() {
  //takeoff
  //if acceleration > blah && mode == 0:
    //mode = 1

  //coast
  //if acceleration < blah && mode == 1:
    //mode = 2

  //apex
  //if speed < 0 && mode == 2:
    //mode = 3

  //drouge deployment...
  //
    //

  //main deployment
  //if alt < xxxx && mode > 3:
    //mode = 5

  //hit ground
  //if speed ~0 && mode > 3:
    //mode = 6
}
