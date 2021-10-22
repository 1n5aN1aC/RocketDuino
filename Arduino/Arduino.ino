//
// File:    main.ino
// Author:  1n5aN1aC (Joshua Villwock)
// Purpose: Handles all logic for the RocketDuino
//
#include "Arduino.h"
#include <Wire.h>
#include <BMP280_DEV.h>
#include <SimpleKalmanFilter.h>
#include <TinyGPS++.h>

// Pin designations
#define BUZZER 17
#define CHUTE1 8
#define CHUTE2 7
#define LED1   4
#define LED2   3


// Barometer Variables
BMP280_DEV bmp280; // Define barometer object. Basic I2C SCL & SDA
SimpleKalmanFilter altitudeFilterObject(4, 1, 0.015); //Measurement Uncertainty, Estimation Uncertainty, Noise
float    temporaryAltitude       = 0;
float    currentFilteredAltitude = 0;
elapsedMillis lastBarometerRead;

// Mode variables
#define  FREQUENCY_UPDATE_MODE   250
uint8_t  currentMode             = 0;
elapsedMillis lastModeUpdate;

// Buzzer Variables
#define  FREQUENCY_UPDATE_BUZZER 3000
elapsedMillis lastBuzzerUpdate;

// Serial Variables
#define  LORABAUDRATE            57600
#define  FREQUENCY_TX_LORA       350
uint16_t telemetryPacketsSent    = 0;
elapsedMillis lastLoraTX;

// GPS Variables
TinyGPSPlus   gps_object;
TinyGPSCustom gpsStatus(gps_object, "GPGGA", 6);
#define  GPSBAUDRATE             57600
bool     GPSforward              = false;
elapsedMillis lastGPSReceive;

typedef struct __attribute__((packed)) packet_normal { //Maximum 51 / 58 Bytes
  int16_t  currentAlt;        //2 bytes
  int16_t  maxAlt;            //2 bytes
  uint8_t  currentMode;       //1 byte
  float    latitude;          //4 bytes
  float    longitude;         //4 bytes
  uint16_t gpsAlt;            //2 bytes
  uint16_t gpsSpeedMPS;       //2 bytes
  uint8_t  sats;              //1 byte
  uint8_t  gps_mode;          //1 byte (only 2 bits needed)
  uint8_t  gps_recent;        //1 byte (only 1 bit needed)
};

typedef struct __attribute__((packed)) packet_ground { //Maximum 51 / 58 Bytes
  int16_t startingAlt;        //2 bytes
};


// Saved Values
int16_t  maxAltitudeSeen    = -32768;   //Highest we've been to
int16_t  startingAltitude   = 0;   //what elevation we consider 'ground'
uint16_t failsafeAltitude   = 100; //minimum alt we need to reach to arm
uint8_t  extraDeploymentAlt = 25;  //Extra meters we have to fall below max alt before deployment (to be safe from jiggle)

//
// Run Initial setup Tasks
//
void setup() {
  // Initialize variables & objects
  pinMode(BUZZER, OUTPUT);
  pinMode(CHUTE1, OUTPUT);
  pinMode(CHUTE2, OUTPUT);
  pinMode(LED1,   OUTPUT);
  pinMode(LED2,   OUTPUT);
  
	//Initialize both serial ports
  Serial1.begin(LORABAUDRATE);
  Serial2.begin(GPSBAUDRATE);
  
  delay(1000);
  
  bmp280.begin(SLEEP_MODE, BMP280_I2C_ALT_ADDR);  // Default initialisation with alternative I2C address (0x76), place the BMP280 into SLEEP_MODE
  bmp280.setPresOversampling(OVERSAMPLING_X2);    // Options are OVERSAMPLING_SKIP, _X1, _X2, _X4, _X8, _X16
  bmp280.setTempOversampling(OVERSAMPLING_X1);    // Options are OVERSAMPLING_SKIP, _X1, _X2, _X4, _X8, _X16
  bmp280.setIIRFilter(IIR_FILTER_OFF);            // Options are IIR_FILTER_OFF, _2, _4, _8, _16
  bmp280.setTimeStandby(TIME_STANDBY_62MS);       // Options are TIME_STANDBY_05MS, _62MS, _125MS, _250MS, _500MS, _1000MS, 2000MS, 4000MS
  bmp280.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE
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
  update_lora_send();
  
  //Handle incoming gps packets
  update_gps_receive();

  //Handle incoming lora packets
  update_lora_receive();
  
  //Update LEDs / display
  update_buzzer();
}

//
// Queries the barometer for elevation (and temp), stores it, and updates the running average
void update_barometer() {
  if (bmp280.getAltitude(temporaryAltitude)) {   // Check if the measurement is complete
    currentFilteredAltitude = altitudeFilterObject.updateEstimate(temporaryAltitude);
    
    //Update max altitude if needed and out of startup mode
    if (currentMode > 0 && currentFilteredAltitude > maxAltitudeSeen) {
      maxAltitudeSeen = currentFilteredAltitude;
    }

    //Take note that we've sucessfully read the altitude:
    lastBarometerRead = 0;
  }
}

//
// Mode 0 = Boot
// Mode 1 = Below Failsafe Alt
// Mode 2 = Flying
// Mode 3 = Eject!
//
void calculate_mode() {
  if (lastModeUpdate > FREQUENCY_UPDATE_MODE) {
    lastModeUpdate = lastModeUpdate - FREQUENCY_UPDATE_MODE;
    
    // Mode 0 = Boot
    if (currentMode == 0) {
      //Once 20 seconds have passed on boot, lets call that the starting height.
      if (millis() > 30000) {
        startingAltitude = currentFilteredAltitude;
        currentMode = 1;
        //Serial.println("Mode 0->1");
      }
      //Also, if we haven't read the barometer recently, abort:
      if (millis() > 5000 && lastBarometerRead > 1000) {
        tone(BUZZER, 4000);
        delay(90000);
      }
      return;
    }
    
    //Mode 1 = Below Failsafe Alt
    if (currentMode == 1) {
      //If we are above the failsafe height, go to flight mode.
      if (currentFilteredAltitude > startingAltitude + failsafeAltitude) {
        currentMode = 2;
        //Serial.println("Mode 1->2");
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
        //Serial.println("Mode 2->3");
        digitalWrite(CHUTE1, HIGH);
        digitalWrite(CHUTE2, HIGH);
        delay(500);
        digitalWrite(CHUTE1, LOW);
        digitalWrite(CHUTE2, LOW);
      }
      return;
    }
    
    //Mode 3 = Eject & Recovery
    if (currentMode == 3) {
      //We're already done, so do nothing I guess?
    }
  }
}

//Process any incoming bytes from the GPS
void update_gps_receive() {
  while (Serial2.available() > 0) {
    byte b = Serial2.read();
    gps_object.encode(b);
    lastGPSReceive = 0;
    
    //If GPSforward is enabled, then pass all GPSdata through lora to the basestation
    if (GPSforward) {
      Serial1.write(b);
    }
  }
}

void update_buzzer() {
  if (lastBuzzerUpdate > FREQUENCY_UPDATE_BUZZER) {
    lastBuzzerUpdate = lastBuzzerUpdate - FREQUENCY_UPDATE_BUZZER;
    if (currentMode == 0) {
      tone(BUZZER, 2000);  // Send 2KHz sound signal...
      delay(75);          // ...for 75ms
      noTone(BUZZER);      // Stop sound...
      delay(50);
      tone(BUZZER, 4000);  // Send 4KHz sound signal...
      delay(75);          // ...for 75ms
      noTone(BUZZER);      // Stop sound...
      delay(50);
      tone(BUZZER, 2000, 25);  // Send 2KHz sound signal for 75ms
    } else if (currentMode == 1) {
      tone(BUZZER, 4000, 25);  // Send 4KHz sound signal for 25ms
    } else if (currentMode == 3) {
      tone(BUZZER, 4000);  // Send 4KHz sound signal...
      delay(100);          // ...for 100ms
      noTone(BUZZER);      // Stop sound...
      delay(100);
      tone(BUZZER, 4000, 100);  // Send 4KHz sound signafor 100ms
    }
  }
}

void update_lora_receive() {
  while (Serial1.available() > 0) {
    byte b = Serial2.read();
  }
}

void update_lora_send() {
  if (lastLoraTX > FREQUENCY_TX_LORA) {
    lastLoraTX = lastLoraTX - FREQUENCY_TX_LORA;

    if (telemetryPacketsSent % 5 == 0) {
      send_packet_ground();
    } else {
      send_packet_normal();
    }

    //Increment counter for next time
    telemetryPacketsSent++;
  }
}

void send_packet_normal() {
  //When was last gps data?  If it was valid, but longer than a couple seconds, override the gps info pkt:
  uint8_t gps_mode;
  if (atoi(gpsStatus.value()) != 0 && gps_object.location.age() > 1500) {
    gps_mode = 3;
  } else {
    gps_mode = atoi(gpsStatus.value());
  }

  //When was the last time we got GPS packets?
  uint8_t gps_recent = 0;
  if (lastGPSReceive < 1000) {
    gps_recent = 1;
  }
  
  packet_normal packet = {static_cast<int16_t>(currentFilteredAltitude), maxAltitudeSeen, currentMode, gps_object.location.lat(), gps_object.location.lng(), gps_object.altitude.meters(), gps_object.speed.mps(), gps_object.satellites.value(), gps_mode, gps_recent};
  Serial1.write(reinterpret_cast<char*>(&packet), sizeof packet);
}

void send_packet_ground() {
  packet_ground packet = {startingAltitude};
  Serial1.write(reinterpret_cast<char*>(&packet), sizeof packet);
}
