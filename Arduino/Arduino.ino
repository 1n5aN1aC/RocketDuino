//
// File:    main.ino
// Author:  1n5aN1aC (Joshua Villwock)
// Purpose: Handles all logic for the RocketDuino
//
#include "Arduino.h"
#include <ESP8266WiFi.h>
#include "runningAverage.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <SimpleKalmanFilter.h>
#include <TinyGPS++.h>

// Version
#define ROCKETDUINO_VERSION "0.4.0"

// Pin designations
#define BUZZER D8
#define CHUTE1 D7

// Barometer Variables
#define FREQUENCY_UPDATE_BAROMETER 100
unsigned long lastBarometerUpdate = 0;
Adafruit_BMP085 baro; // Define barometer object. Basic I2C SCL & SDA
SimpleKalmanFilter altitudeFiltered(1, 1, 0.01); //Measurement Uncertainty, Estimation Uncertainty, Noise
float currentFilteredAltitude     = 0;

// Mode variables
#define FREQUENCY_UPDATE_MODE   250
unsigned long lastModeUpdate    = 0;
unsigned int currentMode        = 0;

// Buzzer Variables
#define FREQUENCY_UPDATE_BUZZER 3000
unsigned long lastBuzzerUpdate  = 0;

// Serial Variables
#define SERIALBAUDRATE           9600
#define FREQUENCY_UPDATE_SERIAL  500
unsigned long lastSerialUpdate   = 0;
unsigned int serialCounter       = 0;

// GPS Variables
TinyGPSPlus gps_object;

// Saved Values
unsigned int maxAltitudeSeen    = 0;  //Highest we've been to
unsigned int startingAltitude   = 0;  //what elevation we consider 'ground'
unsigned int failsafeAltitude   = 100; //minimum alt we need to reach to arm
unsigned int extraDeploymentAlt = 25; //Extra meters we have to fall below max alt before deployment (to be safe from jiggle)

//
// Run Initial setup Tasks
//
void setup() {
  // Initialize variables & objects
  pinMode(BUZZER, OUTPUT);
  pinMode(CHUTE1, OUTPUT);
  
	//Initialize both serial ports
  Serial.begin(SERIALBAUDRATE);
  
  delay(1000);
  
	// Send version information
  Serial.print("\n\nRocketDuino: v");
  Serial.print(ROCKETDUINO_VERSION);
  Serial.print(F(" & TinyGPS++ v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
  
  //Make sure the barometer has started
  if (!baro.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    tone(BUZZER, 2000); // Send 2KHz sound signal...
    while (1) {}
  }
}

//
// Main Loop Tasks
//
void loop() {
	//Gather new (baro) data
  update_barometer();
  
	//Calculate on data (flight modes)
  calculate_mode();
  
  //Update Serial / wireless
  update_serial_send();

  //Handle incoming gps packets
  update_serial_receive();
  
  //Update LEDs / display
  update_buzzer();
  
  //Cool down CPU
  delay(1);
}

//
// Queries the barometer for elevation (and temp), stores it, and updates the running average
void update_barometer() {
  if (millis() - lastBarometerUpdate > FREQUENCY_UPDATE_BAROMETER) {
    lastBarometerUpdate = millis();
    
    float thisAlt = baro.readAltitude();
    float currentFilteredAltitude = altitudeFiltered.updateEstimate(thisAlt);
    
    //Update max altitude if needed
    if (currentFilteredAltitude > maxAltitudeSeen) {
      maxAltitudeSeen = currentFilteredAltitude;
    }
  }
}

//
// Mode 0 = Boot
// Mode 1 = Below Failsafe Alt
// Mode 2 = Flying
// Mode 3 = Eject!
//
void calculate_mode() {
  if (millis() - lastModeUpdate > FREQUENCY_UPDATE_MODE) {
    lastModeUpdate = millis();
    
    // Mode 0 = Boot
    if (currentMode == 0) {
      //Once 20 seconds have passed on boot, lets call that the starting height.
      if (millis() > 20000) {
        startingAltitude = currentFilteredAltitude;
        currentMode = 1;
        Serial.println("Mode 0->1");
      }
      return;
    }
    
    //Mode 1 = Below Failsafe Alt
    if (currentMode == 1) {
      //If we are above the failsafe height, go to flight mode.
      if (currentFilteredAltitude > startingAltitude + failsafeAltitude) {
        currentMode = 2;
        Serial.println("Mode 1->2");
      }
      return;
    }
    
    //Mode 2 = Flying
    if (currentMode == 2) {
      //If we're more than the safety distance below max alt, go to eject mode
      if (maxAltitudeSeen > currentFilteredAltitude + extraDeploymentAlt) {
        //WAIT!! If we're below the failsafe altitude, don't.
        if (currentFilteredAltitude < startingAltitude + failsafeAltitude) {
          return;
        }
        currentMode = 3;
        Serial.println("Mode 2->3");
        digitalWrite(CHUTE1, HIGH);
        delay(500);
        digitalWrite(CHUTE1, LOW);
      }
      return;
    }
    
    //Mode 3 = Eject & Recovery
    if (currentMode == 3) {
      //We're already done, so do nothing I guess?
    }
  }
}

void update_serial_send() {
  if (millis() - lastSerialUpdate > FREQUENCY_UPDATE_SERIAL) {
    lastSerialUpdate = millis();

    if (serialCounter == 0) {
      //Simple Packet
      Serial.print(currentMode);
      Serial.print(',');
      Serial.print(currentFilteredAltitude);
      Serial.println();

      serialCounter = 1; //next time, do advanced packet
    } else {
      //Advanced packet
      Serial.print(currentMode);
      Serial.print(',');
      Serial.print(currentFilteredAltitude);
      Serial.print(',');
      Serial.print(startingAltitude);
      Serial.print(',');
      Serial.print(maxAltitudeSeen);
      Serial.print(',');
      Serial.print(gps_object.location.age()); // Do we have a fix?    //Check if is greater than some number, and use a single bit
      Serial.print(',');
      Serial.print(gps_object.location.lat(), 6); // Latitude in degrees (double)
      Serial.print(',');
      Serial.print(gps_object.location.lng(), 6); // Longitude in degrees (double)
      Serial.print(',');
      Serial.print(gps_object.speed.mph()); // Speed in miles per hour (double)
      Serial.print(',');
      Serial.print(gps_object.altitude.meters()); // Altitude in meters (double)
      Serial.print(',');
      Serial.print(gps_object.satellites.value()); // Number of satellites in use (u32)
      Serial.println();

      serialCounter = 0; //next time, do simple packet
    }
  }
}

void update_serial_receive() {
  while (Serial.available() > 0) {
    gps_object.encode(Serial.read());
  }
}

void update_buzzer() {
  if (millis() - lastBuzzerUpdate > FREQUENCY_UPDATE_BUZZER) {
    lastBuzzerUpdate = millis();
    if (currentMode == 0) {
      tone(BUZZER, 2000);  // Send 2KHz sound signal...
      delay(75);          // ...for 1 sec
      noTone(BUZZER);      // Stop sound...
      delay(50);
      tone(BUZZER, 4000);  // Send 4KHz sound signal...
      delay(75);          // ...for 1 sec
      noTone(BUZZER);      // Stop sound...
      delay(50);
      tone(BUZZER, 2000);  // Send 2KHz sound signal...
      delay(75);          // ...for 1 sec
      noTone(BUZZER);      // Stop sound...
    } else if (currentMode == 1) {
      tone(BUZZER, 4000);  // Send 4KHz sound signal...
      delay(25);          // ...for 1 sec
      noTone(BUZZER);      // Stop sound...
    } else if (currentMode == 3) {
      tone(BUZZER, 4000);  // Send 4KHz sound signal...
      delay(100);          // ...for 1 sec
      noTone(BUZZER);      // Stop sound...
      delay(100);
      tone(BUZZER, 4000);  // Send 4KHz sound signal...
      delay(100);          // ...for 1 sec
      noTone(BUZZER);      // Stop sound...
    }
  }
}
