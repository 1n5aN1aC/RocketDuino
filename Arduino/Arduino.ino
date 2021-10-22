//
// File:    main.ino
// Author:  1n5aN1aC (Joshua Villwock)
// Purpose: Handles all logic for the RocketDuino
//
#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <BMP280_DEV.h>
#include <SimpleKalmanFilter.h>
#include <TinyGPS++.h>

// Pin designations
#define BUZZER D8
#define CHUTE1 D6
#define CHUTE2 D7

// Barometer Variables
BMP280_DEV bmp280; // Define barometer object. Basic I2C SCL & SDA
SimpleKalmanFilter altitudeFilterObject(4, 1, 0.015); //Measurement Uncertainty, Estimation Uncertainty, Noise
float    temporaryAltitude       = 0;
float    currentFilteredAltitude = 0;
uint32_t lastBarometerRead       = 0;

// Mode variables
#define  FREQUENCY_UPDATE_MODE   250
uint32_t lastModeUpdate          = 0;
uint8_t  currentMode             = 0;

// Buzzer Variables
#define  FREQUENCY_UPDATE_BUZZER 3000
uint32_t lastBuzzerUpdate        = 0;

// Serial Variables
#define  SERIALBAUDRATE          57600
#define  FREQUENCY_UPDATE_SERIAL 350
uint32_t lastSerialSend          = 0;
uint32_t lastSerialReceive       = 0;
uint16_t telemetryPacketsSent    = 0;

// GPS Variables
TinyGPSPlus   gps_object;
TinyGPSCustom gpsStatus(gps_object, "GPGGA", 6);

//We only have 1172 bps to work with...
typedef struct __attribute__((packed)) packet_basic { //[5 BYTES]
  int16_t currentAlt         : 16;  // -32768..32767    [16 bits]
  int16_t maxAlt             : 16;  // -32768..32767    [16 bits]
  uint8_t currentMode        : 8;   // 0..3             [2  bits]
};

typedef struct __attribute__((packed)) packet_gps {   //[13 BYTES]
  float    latitude          ;      // float            [32 bits]
  float    longitude         ;      // float            [32 bits]
  uint16_t gpsAlt            : 16;  // 0..65535         [16 bits]
  uint16_t gpsSpeedMPS       : 16;  // 0..65535         [16 bits]
  uint8_t  sats              : 4;   // 0..15            [4  bits] (4  saved)
  uint8_t  gps_mode          : 2;   // 0..3             [2  bits] (6  saved)
  uint8_t  gps_recent        : 1;   // 0..1             [1  bit ] (7  saved)
  uint8_t  padding           : 1;
};

typedef struct __attribute__((packed)) packet_extra { //[1 BYTE]
  int16_t startingAlt       : 16;  // -32768..32767     [16 bits]
};


// Saved Values
int16_t maxAltitudeSeen    = -32768;   //Highest we've been to
int16_t startingAltitude   = 0;   //what elevation we consider 'ground'
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
  
	//Initialize both serial ports
  Serial.begin(SERIALBAUDRATE);
  
  delay(1000);
  
  bmp280.begin(SLEEP_MODE, BMP280_I2C_ALT_ADDR);  // Default initialisation with alternative I2C address (0x76), place the BMP280 into SLEEP_MODE
  bmp280.setPresOversampling(OVERSAMPLING_X1);    // Options are OVERSAMPLING_SKIP, _X1, _X2, _X4, _X8, _X16
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
  update_serial_send();
  
  //Handle incoming gps packets
  update_serial_receive();
  
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
    lastBarometerRead = millis();

    //Serial.println();
    //Serial.print(static_cast<int16_t>(currentFilteredAltitude));
    //Serial.print(",");
    //Serial.print(maxAltitudeSeen);
    //Serial.print(",");
    //Serial.print(currentMode);
    //Serial.print(",");
    //Serial.print(maxAltitudeSeen);
    //Serial.print(",");
    //Serial.print(maxAltitudeSeen - extraDeploymentAlt);
    //Serial.println();
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
      if (millis() > 30000) {
        startingAltitude = currentFilteredAltitude;
        currentMode = 1;
        //Serial.println("Mode 0->1");
      }
      //Also, if we haven't read the barometer recently, abort:
      if (millis() > 5000 && millis() - lastBarometerRead > 1000) {
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

void update_serial_receive() {
  while (Serial.available() > 0) {
    gps_object.encode(Serial.read());
    lastSerialReceive = millis();
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

void update_serial_send() {
  if (millis() - lastSerialSend > FREQUENCY_UPDATE_SERIAL) {
    lastSerialSend = millis();

    if (telemetryPacketsSent % 51 == 0) {
      send_packet_extra();
    } else if (telemetryPacketsSent % 5 == 0) {
      send_packet_gps();
    } else {
      send_packet_basic();
    }

    //Increment counter for next time
    telemetryPacketsSent++;
  }
}

void send_packet_basic() {
  packet_basic packet = {static_cast<int16_t>(currentFilteredAltitude), maxAltitudeSeen, currentMode};
  Serial.write(reinterpret_cast<char*>(&packet), sizeof packet);
}

void send_packet_gps() {
  //When was last gps data?  If it was valid, but longer than a couple seconds, override the gps info pkt:
  uint8_t gps_mode;
  if (atoi(gpsStatus.value()) != 0 && gps_object.location.age() > 1500) {
    gps_mode = 3;
  } else {
    gps_mode = atoi(gpsStatus.value());
  }

  //When was the last time we got GPS packets?
  uint8_t gps_recent = 0;
  if (millis() - lastSerialReceive < 1000) {
    gps_recent = 1;
  }
  
  packet_gps packet = {gps_object.location.lat(), gps_object.location.lng(), gps_object.altitude.meters(), gps_object.speed.mps(), gps_object.satellites.value(), gps_mode, gps_recent, 0};
  Serial.write(reinterpret_cast<char*>(&packet), sizeof packet);
}

void send_packet_extra() {
	packet_extra packet = {startingAltitude};
  Serial.write(reinterpret_cast<char*>(&packet), sizeof packet);
}
