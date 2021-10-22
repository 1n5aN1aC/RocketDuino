//
// File:    CommunicationManager.cpp
// Author:  1n5aN1aC (Joshua Villwock)
// Purpose: Handles communicationg with GPS and LoRa
//
#include "CommunicationManager.h"
#include "DataManager.h"

#include "Arduino.h"
#include <TinyGPS++.h>

// Baud rate settings
#define GPS_BAUD       9600 //57600
#define TX_BAUD        9600
#define ACTUAL_TX_BAUD 1200
#define TARGET_TX_BAUD 1000

// Task Frequency (remember these are minimums)
#define FREQUENCY_SEND_TELEMETRY 1111
#define FREQUENCY_SEND_STATUS    2222

CommunicationManager::CommunicationManager(DataManager data_manager_in) {
  //Initialize the starting values
  lastTelemetryTX = 0UL;
  lastStatusTX    = 0UL;
  lastRX          = 0UL;
  data_manager    = data_manager_in;
}

//
// 
//
void CommunicationManager::setup_connections() {
  // initialize both serial ports:
  Serial.begin(TX_BAUD);
  //Serial1.begin(GPS_BAUD);
}

//
// 
//
void CommunicationManager::transmit_telemetry() {
  if (millis() - lastTelemetryTX > FREQUENCY_SEND_TELEMETRY) {
    //TODO:Transmit Telemetry

    float tacos = data_manager.get_average_temperature();
    
    Serial.println(tacos);

    lastTelemetryTX = millis();
  }
}

//
// 
//
void CommunicationManager::transmit_status() {
  if (millis() - lastStatusTX > FREQUENCY_SEND_STATUS) {
    //Transmit Status

    lastStatusTX = millis();
  }
}

//
// Recieves & Processes gps lines
//
void CommunicationManager::process_gps_serial() {
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
void CommunicationManager::process_gc_serial() {
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
void CommunicationManager::update_leds() {
  
}
